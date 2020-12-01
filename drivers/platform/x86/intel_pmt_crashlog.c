// SPDX-License-Identifier: GPL-2.0
/*
 * Intel Platform Monitoring Technology Crashlog driver
 *
 * Copyright (c) 2020, Intel Corporation.
 * All Rights Reserved.
 *
 * Authors: "Alexander Duyck" <alexander.h.duyck@linux.intel.com>
 */

#include <linux/cdev.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "intel_pmt_core.h"

#define DRV_NAME		"pmt_crashlog"

/* Crashlog discovery header types */
#define CRASH_TYPE_OOBMSM	1

/* Control Flags */
#define CRASHLOG_FLAG_DISABLE	BIT(27)
#define CRASHLOG_FLAG_CLEAR	BIT(28)
#define CRASHLOG_FLAG_EXECUTE	BIT(29)
#define CRASHLOG_FLAG_COMPLETE	BIT(31)
#define CRASHLOG_FLAG_MASK	GENMASK(31, 28)

#define CONTROL_OFFSET		0x0

static DEFINE_IDA(crashlog_devid_ida);

struct pmt_crashlog_priv;

struct crashlog_entry {
	struct pmt_crashlog_priv	*priv;
	struct pmt_header		header;
	void __iomem			*disc_table;
	unsigned long			crashlog_data;
	size_t				crashlog_data_size;
	struct cdev			cdev;
	dev_t				devt;
	int				devid;
	struct ida			*ida;
};

struct pmt_crashlog_priv {
	struct device		*dev;
	struct pci_dev		*parent;
	struct crashlog_entry	*entry;
	int			num_entries;
};

/*
 * I/O
 */
static bool pmt_crashlog_complete(struct crashlog_entry *entry)
{
	u32 control = readl(entry->disc_table + CONTROL_OFFSET);

	/* return current value of the crashlog complete flag */
	return !!(control & CRASHLOG_FLAG_COMPLETE);
}

static bool pmt_crashlog_disabled(struct crashlog_entry *entry)
{
	u32 control = readl(entry->disc_table + CONTROL_OFFSET);

	/* return current value of the crashlog disabled flag */
	return !!(control & CRASHLOG_FLAG_DISABLE);
}

static void pmt_crashlog_set_disable(struct crashlog_entry *entry, bool disable)
{
	u32 control = readl(entry->disc_table + CONTROL_OFFSET);

	/* clear control bits */
	control &= ~(CRASHLOG_FLAG_MASK | CRASHLOG_FLAG_DISABLE);
	if (disable)
		control |= CRASHLOG_FLAG_DISABLE;

	writel(control, entry->disc_table + CONTROL_OFFSET);
}

static void pmt_crashlog_set_clear(struct crashlog_entry *entry)
{
	u32 control = readl(entry->disc_table + CONTROL_OFFSET);

	/* clear control bits */
	control &= ~CRASHLOG_FLAG_MASK;
	control |= CRASHLOG_FLAG_CLEAR;

	writel(control, entry->disc_table + CONTROL_OFFSET);
}

static void pmt_crashlog_set_execute(struct crashlog_entry *entry)
{
	u32 control = readl(entry->disc_table + CONTROL_OFFSET);

	/* clear control bits */
	control &= ~CRASHLOG_FLAG_MASK;
	control |= CRASHLOG_FLAG_EXECUTE;

	writel(control, entry->disc_table + CONTROL_OFFSET);
}

/*
 * devfs
 */
static int pmt_crashlog_open(struct inode *inode, struct file *filp)
{
	struct crashlog_entry *entry;
	struct pci_driver *pci_drv;
	struct pmt_crashlog_priv *priv;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	entry = container_of(inode->i_cdev, struct crashlog_entry, cdev);
	priv = entry->priv;
	pci_drv = pci_dev_driver(priv->parent);

	if (!pci_drv)
		return -ENODEV;

	filp->private_data = entry;
	get_device(&priv->parent->dev);

	if (!try_module_get(pci_drv->driver.owner)) {
		put_device(&priv->parent->dev);
		return -ENODEV;
	}

	return 0;
}

static int pmt_crashlog_release(struct inode *inode, struct file *filp)
{
	struct crashlog_entry *entry = filp->private_data;
	struct pmt_crashlog_priv *priv;
	struct pci_driver *pci_drv;

	priv = entry->priv;
	pci_drv = pci_dev_driver(priv->parent);

	put_device(&priv->parent->dev);
	module_put(pci_drv->driver.owner);

	return 0;
}

static int
pmt_crashlog_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct crashlog_entry *entry = filp->private_data;
	struct pmt_crashlog_priv *priv;
	unsigned long phys = entry->crashlog_data;
	unsigned long pfn = PFN_DOWN(phys);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize;

	if ((vma->vm_flags & VM_WRITE) ||
	    (vma->vm_flags & VM_MAYWRITE))
		return -EPERM;

	priv = entry->priv;

	if (!entry->crashlog_data_size) {
		dev_err(priv->dev, "Crashlog data not accessible\n");
		return -EAGAIN;
	}

	psize = (PFN_UP(entry->crashlog_data + entry->crashlog_data_size) - pfn) *
		PAGE_SIZE;
	if (vsize > psize) {
		dev_err(priv->dev, "Requested mmap size is too large\n");
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (io_remap_pfn_range(vma, vma->vm_start, pfn,
		vsize, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations pmt_crashlog_fops = {
	.owner =	THIS_MODULE,
	.open =		pmt_crashlog_open,
	.mmap =		pmt_crashlog_mmap,
	.release =	pmt_crashlog_release,
};

/*
 * sysfs
 */
static ssize_t
guid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct crashlog_entry *entry;

	entry = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", entry->header.guid);
}
static DEVICE_ATTR_RO(guid);

static ssize_t size_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct crashlog_entry *entry;

	entry = dev_get_drvdata(dev);

	return sprintf(buf, "0x%lu\n", entry->crashlog_data_size);
}
static DEVICE_ATTR_RO(size);

static ssize_t
offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct crashlog_entry *entry;

	entry = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", offset_in_page(entry->crashlog_data));
}
static DEVICE_ATTR_RO(offset);

static ssize_t
enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct crashlog_entry *entry;
	int enabled;

	entry = dev_get_drvdata(dev);
	enabled = !pmt_crashlog_disabled(entry);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
enable_store(struct device *dev, struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct crashlog_entry *entry;
	bool enabled;
	int result;

	entry = dev_get_drvdata(dev);

	result = kstrtobool(buf, &enabled);
	if (result)
		return result;

	pmt_crashlog_set_disable(entry, !enabled);

	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(enable);

static ssize_t
trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct crashlog_entry *entry;
	int trigger;

	entry = dev_get_drvdata(dev);
	trigger = pmt_crashlog_complete(entry);

	return sprintf(buf, "%d\n", trigger);
}

static ssize_t
trigger_store(struct device *dev, struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct crashlog_entry *entry;
	bool trigger;
	int result;

	entry = dev_get_drvdata(dev);

	result = kstrtobool(buf, &trigger);
	if (result)
		return result;

	if (trigger) {
		/* we cannot trigger a new crash if one is still pending */
		if (pmt_crashlog_complete(entry))
			return -EEXIST;

		/* if device is currently disabled, return busy */
		if (pmt_crashlog_disabled(entry))
			return -EBUSY;

		pmt_crashlog_set_execute(entry);
	} else {
		pmt_crashlog_set_clear(entry);
	}

	return strnlen(buf, count);
}
static DEVICE_ATTR_RW(trigger);

static struct attribute *pmt_crashlog_attrs[] = {
	&dev_attr_guid.attr,
	&dev_attr_size.attr,
	&dev_attr_offset.attr,
	&dev_attr_enable.attr,
	&dev_attr_trigger.attr,
	NULL
};
ATTRIBUTE_GROUPS(pmt_crashlog);

static struct class pmt_crashlog_class = {
	.name = "pmt_crashlog",
	.owner = THIS_MODULE,
	.dev_groups = pmt_crashlog_groups,
};

/*
 * initialization
 */
static int pmt_crashlog_make_dev(struct pmt_crashlog_priv *priv,
				 struct crashlog_entry *entry)
{
	struct device *dev;
	int ret;

	ret = alloc_chrdev_region(&entry->devt, 0, 1, DRV_NAME);
	if (ret < 0) {
		dev_err(priv->dev, "alloc_chrdev_region err: %d\n", ret);
		return ret;
	}

	/* Create a character device for Samplers */
	cdev_init(&entry->cdev, &pmt_crashlog_fops);

	ret = cdev_add(&entry->cdev, entry->devt, 1);
	if (ret) {
		dev_err(priv->dev, "Could not add char dev\n");
		return ret;
	}

	dev = device_create(&pmt_crashlog_class, &priv->parent->dev, entry->devt,
			    entry, "%s%d", "crashlog", entry->devid);

	if (IS_ERR(dev)) {
		dev_err(priv->dev, "Could not create device node\n");
		cdev_del(&entry->cdev);
	}

	return PTR_ERR_OR_ZERO(dev);
}

static int pmt_crashlog_add_entry(struct pmt_crashlog_priv *priv,
				  struct crashlog_entry *entry,
				  struct resource *header_res)
{
	int ret;

	pmt_populate_header(PMT_CAP_CRASHLOG, entry->disc_table,
			    &entry->header);

	ret = pmt_get_base_address(priv->dev, &entry->header, header_res,
				   &entry->crashlog_data);
	if (ret)
		return ret;

	entry->crashlog_data_size = entry->header.size * 4;

	if (entry->header.type != CRASH_TYPE_OOBMSM) {
		dev_err(priv->dev, "Unsupported crashlog header type %d\n",
			entry->header.type);
		return -EINVAL;
	}

	if (entry->header.crashlog_version != 0) {
		dev_err(priv->dev, "Unsupported version value %d\n",
			entry->header.crashlog_version);
		return -EINVAL;
	}

	entry->ida = &crashlog_devid_ida;

	entry->devid = ida_simple_get(entry->ida, 0, 0, GFP_KERNEL);
	if (entry->devid < 0)
		return entry->devid;

	ret = pmt_crashlog_make_dev(priv, entry);
	if (ret) {
		ida_simple_remove(entry->ida, entry->devid);
		return ret;
	}

	return 0;
}

static void pmt_crashlog_remove_entries(struct pmt_crashlog_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_entries; i++) {
		device_destroy(&pmt_crashlog_class, priv->entry[i].devt);
		cdev_del(&priv->entry[i].cdev);

		unregister_chrdev_region(priv->entry[i].devt, 1);
		ida_simple_remove(priv->entry[i].ida, priv->entry[i].devid);
	}
}
static int pmt_crashlog_probe(struct platform_device *pdev)
{
	struct pmt_crashlog_priv *priv;
	struct crashlog_entry *entry;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;
	priv->parent  = to_pci_dev(priv->dev->parent);

	priv->entry = devm_kcalloc(&pdev->dev, pdev->num_resources,
				   sizeof(*(priv->entry)), GFP_KERNEL);

	for (i = 0, entry = priv->entry; i < pdev->num_resources;
	     i++, entry++) {
		struct resource *res;
		int ret;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			pmt_crashlog_remove_entries(priv);
			return -ENODEV;
		}

		dev_info(&pdev->dev, "%d res start: 0x%llx, end 0x%llx\n", i,
			 res->start, res->end);

		entry->disc_table = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(entry->disc_table)) {
			pmt_crashlog_remove_entries(priv);
			return PTR_ERR(entry->disc_table);
		}

		ret = pmt_crashlog_add_entry(priv, entry, res);
		if (ret) {
			pmt_crashlog_remove_entries(priv);
			return ret;
		}

		entry->priv = priv;
		priv->num_entries++;
	}

	return 0;
}

static int pmt_crashlog_remove(struct platform_device *pdev)
{
	struct pmt_crashlog_priv *priv;

	priv = (struct pmt_crashlog_priv *)platform_get_drvdata(pdev);

	pmt_crashlog_remove_entries(priv);

	return 0;
}

static struct platform_driver pmt_crashlog_driver = {
	.driver = {
		.name   = DRV_NAME,
	},
	.probe  = pmt_crashlog_probe,
	.remove = pmt_crashlog_remove,
};

static int __init pmt_crashlog_init(void)
{
	int ret = class_register(&pmt_crashlog_class);

	if (ret)
		return ret;

	ret = platform_driver_register(&pmt_crashlog_driver);
	if (ret) {
		class_unregister(&pmt_crashlog_class);
		return ret;
	}

	return 0;
}

static void __exit pmt_crashlog_exit(void)
{
	platform_driver_unregister(&pmt_crashlog_driver);
	class_unregister(&pmt_crashlog_class);
	ida_destroy(&crashlog_devid_ida);
}

module_init(pmt_crashlog_init);
module_exit(pmt_crashlog_exit);

MODULE_AUTHOR("Alexander Duyck <alexander.h.duyck@linux.intel.com>");
MODULE_DESCRIPTION("Intel PMT Crashlog driver");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL v2");
