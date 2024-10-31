// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023-2025 Intel Corporation */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/idr.h>
#include <linux/issei.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>

#include "issei_dev.h"
#include "host_client.h"
#include "cdev.h"

struct class *issei_class;
static dev_t issei_devt;

#define ISSEI_MAX_DEVS MINORMASK

static DEFINE_MUTEX(issei_minor_lock);
static DEFINE_IDR(issei_idr);

static int issei_open(struct inode *inode, struct file *fp)
{
	struct issei_host_client *cl;
	struct issei_device *idev;

	idev = container_of(inode->i_cdev, struct issei_device, cdev);

	cl = issei_cl_create(idev, fp);
	if (IS_ERR(cl))
		return PTR_ERR(cl);
	fp->private_data = cl;

	return nonseekable_open(inode, fp);
}

static int issei_release(struct inode *inode, struct file *fp)
{
	struct issei_host_client *cl = fp->private_data;

	issei_cl_remove(cl);

	return 0;
}

/**
 * issei_ioctl - the IOCTL function
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to issei message structure
 *
 * Return: 0 on success , <0 on error
 */
static long issei_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
	struct issei_host_client *cl = file->private_data;
	struct issei_connect_client_data conn;
	struct issei_device *idev = cl->idev;
	int ret;

	switch (cmd) {
	case IOCTL_ISSEI_CONNECT_CLIENT:
		dev_dbg(idev->dev, "IOCTL_ISSEI_CONNECT_CLIENT\n");

		if (idev->rst_state != ISSEI_RST_STATE_DONE) {
			dev_dbg(idev->dev, "Device is in transition\n");
			return -ENODEV;
		}

		if (copy_from_user(&conn, (char __user *)data, sizeof(conn))) {
			dev_dbg(idev->dev, "failed to copy data from userland\n");
			return -EFAULT;
		}

		ret = issei_cl_connect(cl, (uuid_t *)&conn.in_client_uuid,
				       &conn.out_client_properties.max_msg_length,
				       &conn.out_client_properties.protocol_version,
				       &conn.out_client_properties.flags);
		if (ret)
			return ret;

		if (copy_to_user((char __user *)data, &conn, sizeof(conn))) {
			dev_dbg(idev->dev, "failed to copy data to userland\n");
			return -EFAULT;
		}
		break;

	case IOCTL_ISSEI_DISCONNECT_CLIENT:
		dev_dbg(idev->dev, "IOCTL_ISSEI_DISCONNECT_CLIENT\n");

		if (idev->rst_state != ISSEI_RST_STATE_DONE) {
			dev_dbg(idev->dev, "Device is in transition\n");
			return -ENODEV;
		}

		ret = issei_cl_disconnect(cl);
		if (ret)
			return ret;
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static ssize_t issei_write(struct file *file, const char __user *ubuf,
			   size_t length, loff_t *offset)
{
	struct issei_host_client *cl = file->private_data;
	struct issei_device *idev = cl->idev;
	ssize_t ret;
	u8 *buf;

	if (!length)
		return 0;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(idev->dev, "Device is in transition\n");
		return -EBUSY;
	}

	/* sanity check */
	if (length > idev->dma.length.h2f) {
		dev_dbg(idev->dev, "Write is too big %zu > %zu\n",
			length, idev->dma.length.h2f);
		return -EFBIG;
	}

	buf = memdup_user(ubuf, length);
	if (IS_ERR(buf)) {
		dev_dbg(idev->dev, "failed to copy data from userland\n");
		return PTR_ERR(buf);
	}

	do {
		ret = issei_cl_write(cl, buf, length);
		if (ret < 0 && ret != -EAGAIN) {
			kfree(buf);
			return ret;
		}
		if (wait_event_interruptible(cl->write_wait, (issei_cl_check_write(cl) != 1))) {
			if (signal_pending(current))
				return -EINTR;
			return -ERESTARTSYS;
		}
	} while (ret == -EAGAIN);

	return ret;
}

static ssize_t issei_read(struct file *file, char __user *ubuf,
			  size_t length, loff_t *offset)
{
	struct issei_host_client *cl = file->private_data;
	struct issei_device *idev = cl->idev;
	size_t data_size = length;
	u8 *data = NULL;
	ssize_t ret;

	if (!length)
		return 0;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(idev->dev, "Device is in transition\n");
		return -EBUSY;
	}

	/* sanity check */
	if (length > idev->dma.length.f2h) {
		dev_dbg(idev->dev, "Read is too big %zu > %zu\n",
			length, idev->dma.length.f2h);
		return -EFBIG;
	}

	ret = issei_cl_read(cl, &data, &data_size);
	if (!ret)
		goto copy;
	if (ret != -ENOENT)
		return ret;

	if (wait_event_interruptible(cl->read_wait, (issei_cl_check_read(cl) != 0))) {
		if (signal_pending(current))
			return -EINTR;
		return -ERESTARTSYS;
	}

	ret = issei_cl_read(cl, &data, &data_size);
	if (ret)
		return ret;

copy:
	if (copy_to_user(ubuf, data, data_size)) {
		dev_dbg(idev->dev, "failed to copy data to userland\n");
		ret = -EFAULT;
	} else {
		*offset = 0;
		ret = data_size;
	}

	kfree(data);

	return ret;
}

static __poll_t issei_poll(struct file *file, poll_table *wait)
{
	__poll_t req_events = poll_requested_events(wait);
	struct issei_host_client *cl = file->private_data;
	struct issei_device *idev = cl->idev;
	__poll_t mask = 0;
	int ret;

	if (idev->rst_state != ISSEI_RST_STATE_DONE) {
		dev_dbg(idev->dev, "Device is in transition\n");
		return EPOLLERR;
	}

	if (req_events & (EPOLLIN | EPOLLRDNORM)) {
		poll_wait(file, &cl->read_wait, wait);
		ret = issei_cl_check_read(cl);
		if (ret == 1)
			mask |= EPOLLIN | EPOLLRDNORM;
		else if (ret < 0)
			mask |= EPOLLERR;
	}

	if (req_events & (EPOLLOUT | EPOLLWRNORM)) {
		poll_wait(file, &cl->write_wait, wait);
		ret = issei_cl_check_write(cl);
		if (ret == 0)
			mask |= EPOLLOUT | EPOLLWRNORM;
		else if (ret < 0)
			mask |= EPOLLERR;
	}

	return mask;
}

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
	.open = issei_open,
	.unlocked_ioctl = issei_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.write = issei_write,
	.read = issei_read,
	.release = issei_release,
	.poll = issei_poll,
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
