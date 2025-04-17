// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023-2025 Intel Corporation */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/idr.h>
#include <linux/issei.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "issei_dev.h"
#include "cdev.h"

struct class *issei_class;
static dev_t issei_devt;

#define ISSEI_MAX_DEVS MINORMASK

static DEFINE_MUTEX(issei_minor_lock);
static DEFINE_IDR(issei_idr);

static ssize_t fw_ver_show(struct device *device,
			   struct device_attribute *attr, char *buf)
{
	struct issei_device *idev = dev_get_drvdata(device);

	return sysfs_emit(buf, "%u.%u.%u.%u\n", idev->fw_version[0], idev->fw_version[1],
			  idev->fw_version[2], idev->fw_version[3]);
}
static DEVICE_ATTR_RO(fw_ver);

static struct attribute *issei_attrs[] = {
	&dev_attr_fw_ver.attr,
	NULL
};
ATTRIBUTE_GROUPS(issei);

static const struct file_operations issei_fops = {
	.owner = THIS_MODULE,
};

static int issei_minor_get(struct issei_device *idev)
{
	int ret;

	guard(mutex)(&issei_minor_lock);

	ret = idr_alloc(&issei_idr, idev, 0, ISSEI_MAX_DEVS, GFP_KERNEL);
	if (ret >= 0)
		idev->minor = ret;
	else if (ret == -ENOSPC)
		dev_err(idev->dev, "too many issei devices\n");

	return ret;
}

static void issei_minor_free(struct issei_device *idev)
{
	guard(mutex)(&issei_minor_lock);

	idr_remove(&issei_idr, idev->minor);
}

/**
 * issei_register: register issei character device
 * @idev: the device structure
 * @parent: parent device
 *
 * Return: 0 on sucess, <0 on failure
 */
int issei_register(struct issei_device *idev, struct device *parent)
{
	struct device *clsdev;
	int ret, devno;

	ret = issei_minor_get(idev);
	if (ret < 0)
		return ret;

	devno = MKDEV(MAJOR(issei_devt), idev->minor);
	cdev_init(&idev->cdev, &issei_fops);
	if (parent->driver)
		idev->cdev.owner = parent->driver->owner;

	ret = cdev_add(&idev->cdev, devno, 1);
	if (ret) {
		dev_err(parent, "unable to add device %d:%d\n",
			MAJOR(issei_devt), idev->minor);
		goto err_dev_add;
	}

	clsdev = device_create_with_groups(issei_class, parent, devno,
					   idev, issei_groups,
					   "issei%d", idev->minor);
	if (IS_ERR(clsdev)) {
		dev_err(parent, "unable to create device %d:%d\n",
			MAJOR(issei_devt), idev->minor);
		ret = PTR_ERR(clsdev);
		goto err_dev_create;
	}

	return 0;

err_dev_create:
	cdev_del(&idev->cdev);
err_dev_add:
	issei_minor_free(idev);

	return ret;
}
EXPORT_SYMBOL_GPL(issei_register);

/**
 * issei_deregister: remove issei character device
 * @idev: the device structure
 */
void issei_deregister(struct issei_device *idev)
{
	int devno;

	devno = idev->cdev.dev;
	cdev_del(&idev->cdev);

	device_destroy(issei_class, devno);

	issei_minor_free(idev);
}
EXPORT_SYMBOL_GPL(issei_deregister);

static int __init issei_cdev_init(void)
{
	int ret;

	issei_class = class_create("issei");
	if (IS_ERR(issei_class)) {
		pr_err("couldn't create class\n");
		return PTR_ERR(issei_class);
	}

	ret = alloc_chrdev_region(&issei_devt, 0, ISSEI_MAX_DEVS, "issei");
	if (ret < 0) {
		pr_err("unable to allocate char dev region\n");
		class_destroy(issei_class);
		return ret;
	}

	return 0;
}

static void __exit issei_cdev_exit(void)
{
	unregister_chrdev_region(issei_devt, ISSEI_MAX_DEVS);
	class_destroy(issei_class);
}

module_init(issei_cdev_init);
module_exit(issei_cdev_exit);

MODULE_DESCRIPTION("Intel(R) Silicon Security Engine Interface");
MODULE_LICENSE("GPL");
