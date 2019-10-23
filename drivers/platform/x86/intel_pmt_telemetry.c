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

#include "intel_pmt_telem.h"

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

#define TELEM_XA_START		1
#define TELEM_XA_MAX		INT_MAX
#define TELEM_XA_LIMIT		XA_LIMIT(TELEM_XA_START, TELEM_XA_MAX)

/* Used by client hardware to identify a fixed telemetry entry*/
#define TELEM_CLIENT_FIXED_BLOCK_GUID	0x10000000

static DEFINE_XARRAY_ALLOC(telem_array);
static DEFINE_MUTEX(list_lock);
static BLOCKING_NOTIFIER_HEAD(telem_notifier);

#define NUM_BYTES_DWORD(v)		((v) << 2)
#define NUM_BYTES_QWORD(v)		((v) << 3)
#define SAMPLE_ID_OFFSET(v)		((v) << 3)

struct telem_endpoint {
	struct pci_dev			*parent;
	struct telem_header		header;
	void __iomem			*base;
	struct resource			res;
	bool				present;
	struct kref			kref;
};

struct pmt_telem_priv;

struct pmt_telem_entry {
	struct pmt_telem_priv		*priv;
	struct telem_endpoint		*ep;
	struct telem_header		header;
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

/* Called when all users unregister and the device is removed */
static void pmt_telem_ep_release(struct kref *kref)
{
	struct telem_endpoint *ep;

	ep = container_of(kref, struct telem_endpoint, kref);
	iounmap(ep->base);
	release_mem_region(ep->res.start, resource_size(&ep->res));
	kfree(ep);
}

/*
 * driver api
 */
int pmt_telem_get_next_endpoint(int start)
{
	struct telem_endpoint *ep;
	unsigned long found_idx;

	mutex_lock(&list_lock);
	xa_for_each_start(&telem_array, found_idx, ep, start) {
		/*
		 * Return first found index after start.
		 * 0 is not valid id.
		 */
		if (found_idx > start)
			break;
	}
	mutex_unlock(&list_lock);

	return found_idx == start ? 0 : found_idx;
}
EXPORT_SYMBOL_GPL(pmt_telem_get_next_endpoint);

struct telem_endpoint *pmt_telem_register_endpoint(int devid)
{
	struct telem_endpoint *ep;
	unsigned long index = devid;

	mutex_lock(&list_lock);
	ep = xa_find(&telem_array, &index, index, XA_PRESENT);
	if (!ep) {
		mutex_unlock(&list_lock);
		return ERR_PTR(-ENXIO);
	}

	kref_get(&ep->kref);

	mutex_unlock(&list_lock);

	return ep;
}
EXPORT_SYMBOL_GPL(pmt_telem_register_endpoint);

void pmt_telem_unregister_endpoint(struct telem_endpoint *ep)
{
	kref_put(&ep->kref, pmt_telem_ep_release);
}
EXPORT_SYMBOL(pmt_telem_unregister_endpoint);

int pmt_telem_get_endpoint_info(int devid,
				struct telem_endpoint_info *info)
{
	struct telem_endpoint *ep;
	unsigned long index = devid;
	int err = 0;

	if (!info)
		return -EINVAL;

	mutex_lock(&list_lock);
	ep = xa_find(&telem_array, &index, index, XA_PRESENT);
	if (!ep) {
		err = -ENXIO;
		goto unlock;
	}

	info->pdev = ep->parent;
	info->header = ep->header;

unlock:
	mutex_unlock(&list_lock);
	return err;

}
EXPORT_SYMBOL_GPL(pmt_telem_get_endpoint_info);

int
pmt_telem_read32(struct telem_endpoint *ep, u32 offset, u32 *data, u32 count)
{
	void __iomem *base;
	u32 size;

	if (!ep->present)
		return -ENODEV;

	/*
	 * offset is relative to the BAR base address, not the counter
	 * base address.
	 */
	if (offset < ep->header.base_offset)
		return -EINVAL;

	offset -= ep->header.base_offset;
	base = ep->base;
	size = ep->header.size;

	if ((offset + NUM_BYTES_DWORD(count)) > size)
		return -EINVAL;

	memcpy_fromio(data, base + offset, NUM_BYTES_DWORD(count));

	return ep->present ? 0 : -EPIPE;
}
EXPORT_SYMBOL_GPL(pmt_telem_read32);

int
pmt_telem_read64(struct telem_endpoint *ep, u32 offset, u64 *data, u32 count)
{
	void __iomem *base;
	u32 size;

	if (!ep->present)
		return -ENODEV;

	/*
	 * offset is relative to the BAR base address, not the counter
	 * base address.
	 */
	if (offset < ep->header.base_offset)
		return -EINVAL;

	offset -= ep->header.base_offset;
	base = ep->base;
	size = ep->header.size;

	if ((offset + NUM_BYTES_QWORD(count)) > size)
		return -EINVAL;

	memcpy_fromio(data, base + offset, NUM_BYTES_QWORD(count));

	return ep->present ? 0 : -EPIPE;
}
EXPORT_SYMBOL_GPL(pmt_telem_read64);

int
pmt_telem_read(struct telem_endpoint *ep, u32 id, u64 *data, u32 count)
{
	u32 offset, size;

	if (!ep->present)
		return -ENODEV;

	offset = SAMPLE_ID_OFFSET(id);
	size = ep->header.size;

	if ((offset + NUM_BYTES_QWORD(count)) > size)
		return -EINVAL;

	memcpy_fromio(data, ep->base + offset, NUM_BYTES_QWORD(count));

	return ep->present ? 0 : -EPIPE;
}
EXPORT_SYMBOL_GPL(pmt_telem_read);

int pmt_telem_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&telem_notifier, nb);
}
EXPORT_SYMBOL(pmt_telem_register_notifier);

int pmt_telem_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&telem_notifier, nb);
}
EXPORT_SYMBOL(pmt_telem_unregister_notifier);

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

static int pmt_telem_add_endpoint(struct pmt_telem_priv *priv,
				  struct pmt_telem_entry *entry)
{
	struct telem_endpoint *ep;
	struct resource *req, *res;
	int err;

	/*
	 * Endpoint lifetimes are managed by kref, not devres.
	 */
	entry->ep = kzalloc(sizeof(*(entry->ep)), GFP_KERNEL);
	if (!entry->ep)
		return -ENOMEM;

	ep = entry->ep;
	ep->header = entry->header;
	ep->parent = to_pci_dev(priv->dev->parent);

	res = &ep->res;
	res->start = entry->base_addr;
	res->end = res->start + (entry->header.size) - 1;

	req = request_mem_region(res->start, resource_size(res),
				 dev_name(priv->dev));
	if (!req) {
		dev_err(priv->dev, "Failed to claim memory for region %pR\n",
			res);
		err = -EIO;
		goto fail_request_mem_region;
	}

	ep->base = ioremap(res->start, resource_size(res));
	if (!ep->base) {
		dev_err(priv->dev, "Failed to ioremap device region\n");
		err = -EIO;
		goto fail_ioremap;
	}

	ep->present = true;

	kref_init(&ep->kref);

	return 0;

fail_ioremap:
	release_mem_region(res->start,
			   resource_size(res));
fail_request_mem_region:
	kfree(ep);

	return err;
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
			       struct pmt_telem_entry *entry,
			       struct resource *header_res)
{
	struct pci_dev *pci_dev = to_pci_dev(priv->dev->parent);
	struct telem_header *header = &entry->header;
	struct device *dev = &pci_dev->dev;
	int ret;

	pmt_telem_populate_header(entry->disc_table, &entry->header);

	/* Ony Local access and BARID access modes only for now */
	switch (entry->header.access_type) {
	case TELEM_ACCESS_LOCAL:
		if (header->tbir) {
			dev_err(priv->dev,
				"Unsupported BAR index %d for access type %d\n",
				header->tbir, header->access_type);
			return -EINVAL;
		}

		/*
		 * For access_type LOCAL, the base address is as follows:
		 * base address = header address + header length + base offset
		 */
		entry->base_addr = header_res->start + resource_size(header_res) +
				   header->base_offset;
		break;

	case TELEM_ACCESS_BARID:
		entry->base_addr = pci_dev->resource[header->tbir].start +
				   header->base_offset;
		break;

	default:
		dev_err(dev, "Unsupported access type %d\n",
			entry->header.access_type);
		return -EINVAL;
	}

	ret = alloc_chrdev_region(&entry->devt, 0, 1, TELEM_DEV_NAME);
	if (ret) {
		dev_err(dev, "PMT telemetry chrdev_region error: %d\n", ret);
		return ret;
	}

	ret = pmt_telem_add_endpoint(priv, entry);
	if (ret)
		goto fail_add_endpoint;

	ret = xa_alloc(&telem_array, &entry->devid, entry->ep, TELEM_XA_LIMIT,
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
	kref_put(&entry->ep->kref, pmt_telem_ep_release);
fail_add_endpoint:
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
		priv->entry->ep->present = false;
		device_destroy(&pmt_telem_class, priv->entry[i].devt);
		cdev_del(&priv->entry[i].cdev);
		xa_erase(&telem_array, priv->entry[i].devid);
		unregister_chrdev_region(priv->entry[i].devt, 1);
		kref_put(&priv->entry[i].ep->kref, pmt_telem_ep_release);
	}
}

static int pmt_telem_probe(struct platform_device *pdev)
{
	struct pmt_telem_priv *priv;
	struct pmt_telem_entry *entry;
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

	for (i = 0, entry = priv->entry; i < pdev->num_resources;
	     i++, entry++) {
		struct resource *res;
		int ret;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			pmt_telem_remove_entries(priv);
			return -ENODEV;
		}

		entry->disc_table = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(entry->disc_table)) {
			pmt_telem_remove_entries(priv);
			return PTR_ERR(entry->disc_table);
		}

		if (pmt_telem_is_early_client_hw(&pdev->dev) &&
		    pmt_telem_region_overlaps(pdev, entry->disc_table))
			continue;

		ret = pmt_telem_add_entry(priv, entry, res);
		if (ret) {
			pmt_telem_remove_entries(priv);
			return ret;
		}
	}

	for (i = 0, entry = priv->entry; i < priv->num_entries; i++, entry++)
		blocking_notifier_call_chain(&telem_notifier,
					     PMT_TELEM_NOTIFY_ADD,
					     &entry->devid);

	return 0;
}

static int pmt_telem_remove(struct platform_device *pdev)
{
	struct pmt_telem_priv *priv = platform_get_drvdata(pdev);
	struct pmt_telem_entry *entry;
	int i;

	for (i = 0, entry = priv->entry; i < priv->num_entries; i++, entry++)
		blocking_notifier_call_chain(&telem_notifier,
					     PMT_TELEM_NOTIFY_REMOVE,
					     &entry->devid);

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
