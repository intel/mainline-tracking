// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Platform Monitory Technology Telemetry driver
 *
 * Copyright (c) 2020, Intel Corporation.
 * All Rights Reserved.
 *
 * Author: "David E. Box" <david.e.box@linux.intel.com>
 */

#include <linux/bits.h>
#include <linux/cdev.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/xarray.h>

#define TELEM_DEV_NAME		"pmt_telemetry"

/* Telemetry access types */
#define TELEM_ACCESS_FUTURE	1
#define TELEM_ACCESS_BARID	2
#define TELEM_ACCESS_LOCAL	3

#define TELEM_GUID_OFFSET	0x4
#define TELEM_BASE_OFFSET	0x8
#define TELEM_TBIR_MASK		GENMASK(2, 0)
#define TELEM_ACCESS(v)		((v) & GENMASK(3, 0))
#define TELEM_TYPE(v)		(((v) & GENMASK(7, 4)) >> 4)
/* size is in bytes */
#define TELEM_SIZE(v)		(((v) & GENMASK(27, 12)) >> 10)

#define TELEM_XA_START		0
#define TELEM_XA_MAX		INT_MAX
#define TELEM_XA_LIMIT		XA_LIMIT(TELEM_XA_START, TELEM_XA_MAX)

/* Used by client hardware to identify a fixed telemetry entry*/
#define TELEM_CLIENT_FIXED_BLOCK_GUID	0x10000000

static DEFINE_XARRAY_ALLOC(telem_array);

struct pmt_telem_priv;

struct telem_header {
	u8	access_type;
	u8	telem_type;
	u16	size;
	u32	guid;
	u32	base_offset;
	u8	tbir;
};

struct pmt_telem_entry {
	struct pmt_telem_priv		*priv;
	struct telem_header		header;
	struct resource			*header_res;
	unsigned long			base_addr;
	void __iomem			*disc_table;
	struct cdev			cdev;
	dev_t				devt;
	int				devid;
};

struct pmt_telem_priv {
	struct pmt_telem_entry		*entry;
	int				num_entries;
	struct device			*dev;
};

/*
 * devfs
 */
static int pmt_telem_open(struct inode *inode, struct file *filp)
{
	struct pmt_telem_priv *priv;
	struct pmt_telem_entry *entry;
	struct pci_driver *pci_drv;
	struct pci_dev *pci_dev;

	if (!perfmon_capable())
		return -EPERM;

	entry = container_of(inode->i_cdev, struct pmt_telem_entry, cdev);
	priv = entry->priv;
	pci_dev = to_pci_dev(priv->dev->parent);

	pci_drv = pci_dev_driver(pci_dev);
	if (!pci_drv)
		return -ENODEV;

	filp->private_data = entry;
	get_device(&pci_dev->dev);

	if (!try_module_get(pci_drv->driver.owner)) {
		put_device(&pci_dev->dev);
		return -ENODEV;
	}

	return 0;
}

static int pmt_telem_release(struct inode *inode, struct file *filp)
{
	struct pmt_telem_entry *entry = filp->private_data;
	struct pci_dev *pci_dev = to_pci_dev(entry->priv->dev->parent);
	struct pci_driver *pci_drv = pci_dev_driver(pci_dev);

	put_device(&pci_dev->dev);
	module_put(pci_drv->driver.owner);

	return 0;
}

static int pmt_telem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct pmt_telem_entry *entry = filp->private_data;
	struct pmt_telem_priv *priv;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long phys = entry->base_addr;
	unsigned long pfn = PFN_DOWN(phys);
	unsigned long psize;

	priv = entry->priv;
	psize = (PFN_UP(entry->base_addr + entry->header.size) - pfn) * PAGE_SIZE;
	if (vsize > psize) {
		dev_err(priv->dev, "Requested mmap size is too large\n");
		return -EINVAL;
	}

	if ((vma->vm_flags & VM_WRITE) || (vma->vm_flags & VM_MAYWRITE))
		return -EPERM;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start, pfn, vsize,
			       vma->vm_page_prot))
		return -EINVAL;

	return 0;
}

static const struct file_operations pmt_telem_fops = {
	.owner =	THIS_MODULE,
	.open =		pmt_telem_open,
	.mmap =		pmt_telem_mmap,
	.release =	pmt_telem_release,
};

/*
 * sysfs
 */
static ssize_t guid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pmt_telem_entry *entry = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", entry->header.guid);
}
static DEVICE_ATTR_RO(guid);

static ssize_t size_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct pmt_telem_entry *entry = dev_get_drvdata(dev);

	/* Display buffer size in bytes */
	return sprintf(buf, "%u\n", entry->header.size);
}
static DEVICE_ATTR_RO(size);

static ssize_t offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pmt_telem_entry *entry = dev_get_drvdata(dev);

	/* Display buffer offset in bytes */
	return sprintf(buf, "%lu\n", offset_in_page(entry->base_addr));
}
static DEVICE_ATTR_RO(offset);

static struct attribute *pmt_telem_attrs[] = {
	&dev_attr_guid.attr,
	&dev_attr_size.attr,
	&dev_attr_offset.attr,
	NULL
};
ATTRIBUTE_GROUPS(pmt_telem);

static struct class pmt_telem_class = {
	.owner	= THIS_MODULE,
	.name	= "pmt_telemetry",
	.dev_groups = pmt_telem_groups,
};

/*
 * driver initialization
 */
static const struct pci_device_id pmt_telem_early_client_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x9a0d) }, /* TGL */
	{ }
};

static bool pmt_telem_is_early_client_hw(struct device *dev)
{
	struct pci_dev *parent = to_pci_dev(dev->parent);

	return !!pci_match_id(pmt_telem_early_client_pci_ids, parent);
}

static int pmt_telem_create_dev(struct pmt_telem_priv *priv,
				struct pmt_telem_entry *entry)
{
	struct pci_dev *pci_dev;
	struct device *dev;
	int ret;

	cdev_init(&entry->cdev, &pmt_telem_fops);
	ret = cdev_add(&entry->cdev, entry->devt, 1);
	if (ret) {
		dev_err(priv->dev, "Could not add char dev\n");
		return ret;
	}

	pci_dev = to_pci_dev(priv->dev->parent);
	dev = device_create(&pmt_telem_class, &pci_dev->dev, entry->devt,
			    entry, "telem%d", entry->devid);
	if (IS_ERR(dev)) {
		dev_err(priv->dev, "Could not create device node\n");
		cdev_del(&entry->cdev);
	}

	return PTR_ERR_OR_ZERO(dev);
}

static void pmt_telem_populate_header(void __iomem *disc_offset,
				      struct telem_header *header)
{
	header->access_type = TELEM_ACCESS(readb(disc_offset));
	header->telem_type = TELEM_TYPE(readb(disc_offset));
	header->size = TELEM_SIZE(readl(disc_offset));
	header->guid = readl(disc_offset + TELEM_GUID_OFFSET);
	header->base_offset = readl(disc_offset + TELEM_BASE_OFFSET);

	/*
	 * For non-local access types the lower 3 bits of base offset
	 * contains the index of the base address register where the
	 * telemetry can be found.
	 */
	header->tbir = header->base_offset & TELEM_TBIR_MASK;
	header->base_offset ^= header->tbir;
}

static int pmt_telem_add_entry(struct pmt_telem_priv *priv,
			       struct pmt_telem_entry *entry)
{
	struct resource *res = entry->header_res;
	struct pci_dev *pci_dev = to_pci_dev(priv->dev->parent);
	int ret;

	pmt_telem_populate_header(entry->disc_table, &entry->header);

	/* Local access and BARID only for now */
	switch (entry->header.access_type) {
	case TELEM_ACCESS_LOCAL:
		if (entry->header.tbir) {
			dev_err(priv->dev,
				"Unsupported BAR index %d for access type %d\n",
				entry->header.tbir, entry->header.access_type);
			return -EINVAL;
		}

		/*
		 * For access_type LOCAL, the base address is as follows:
		 * base address = header address + header length + base offset
		 */
		entry->base_addr = res->start + resource_size(res) +
				   entry->header.base_offset;
		break;

	case TELEM_ACCESS_BARID:
		entry->base_addr = pci_dev->resource[entry->header.tbir].start +
				   entry->header.base_offset;
		break;

	default:
		dev_err(priv->dev, "Unsupported access type %d\n",
			entry->header.access_type);
		return -EINVAL;
	}

	ret = alloc_chrdev_region(&entry->devt, 0, 1, TELEM_DEV_NAME);
	if (ret) {
		dev_err(priv->dev,
			"PMT telemetry chrdev_region error: %d\n", ret);
		return ret;
	}

	ret = xa_alloc(&telem_array, &entry->devid, entry, TELEM_XA_LIMIT,
		       GFP_KERNEL);
	if (ret)
		goto fail_xa_alloc;

	ret = pmt_telem_create_dev(priv, entry);
	if (ret)
		goto fail_create_dev;

	entry->priv = priv;
	priv->num_entries++;
	return 0;

fail_create_dev:
	xa_erase(&telem_array, entry->devid);
fail_xa_alloc:
	unregister_chrdev_region(entry->devt, 1);

	return ret;
}

static bool pmt_telem_region_overlaps(struct platform_device *pdev,
				      void __iomem *disc_table)
{
	u32 guid;

	guid = readl(disc_table + TELEM_GUID_OFFSET);

	return guid == TELEM_CLIENT_FIXED_BLOCK_GUID;
}

static void pmt_telem_remove_entries(struct pmt_telem_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_entries; i++) {
		device_destroy(&pmt_telem_class, priv->entry[i].devt);
		cdev_del(&priv->entry[i].cdev);
		xa_erase(&telem_array, priv->entry[i].devid);
		unregister_chrdev_region(priv->entry[i].devt, 1);
	}
}

static int pmt_telem_probe(struct platform_device *pdev)
{
	struct pmt_telem_priv *priv;
	struct pmt_telem_entry *entry;
	bool early_hw;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;

	priv->entry = devm_kcalloc(&pdev->dev, pdev->num_resources,
				   sizeof(struct pmt_telem_entry), GFP_KERNEL);
	if (!priv->entry)
		return -ENOMEM;

	if (pmt_telem_is_early_client_hw(&pdev->dev))
		early_hw = true;

	for (i = 0, entry = priv->entry; i < pdev->num_resources;
	     i++, entry++) {
		int ret;

		entry->header_res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!entry->header_res) {
			pmt_telem_remove_entries(priv);
			return -ENODEV;
		}

		entry->disc_table = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(entry->disc_table)) {
			pmt_telem_remove_entries(priv);
			return PTR_ERR(entry->disc_table);
		}

		if (pmt_telem_region_overlaps(pdev, entry->disc_table) &&
		    early_hw)
			continue;

		ret = pmt_telem_add_entry(priv, entry);
		if (ret) {
			pmt_telem_remove_entries(priv);
			return ret;
		}
	}

	return 0;
}

static int pmt_telem_remove(struct platform_device *pdev)
{
	struct pmt_telem_priv *priv = platform_get_drvdata(pdev);

	pmt_telem_remove_entries(priv);

	return 0;
}

static const struct platform_device_id pmt_telem_table[] = {
	{
		.name = "pmt_telemetry",
	},
	{}
};
MODULE_DEVICE_TABLE(platform, pmt_telem_table);

static struct platform_driver pmt_telem_driver = {
	.driver = {
		.name   = TELEM_DEV_NAME,
	},
	.probe  = pmt_telem_probe,
	.remove = pmt_telem_remove,
	.id_table = pmt_telem_table,
};

static int __init pmt_telem_init(void)
{
	int ret = class_register(&pmt_telem_class);

	if (ret)
		return ret;

	ret = platform_driver_register(&pmt_telem_driver);
	if (ret)
		class_unregister(&pmt_telem_class);

	return ret;
}
module_init(pmt_telem_init);

static void __exit pmt_telem_exit(void)
{
	platform_driver_unregister(&pmt_telem_driver);
	class_unregister(&pmt_telem_class);
	xa_destroy(&telem_array);
}
module_exit(pmt_telem_exit);

MODULE_AUTHOR("David E. Box <david.e.box@linux.intel.com>");
MODULE_DESCRIPTION("Intel PMT Telemetry driver");
MODULE_ALIAS("platform:" TELEM_DEV_NAME);
MODULE_LICENSE("GPL v2");
