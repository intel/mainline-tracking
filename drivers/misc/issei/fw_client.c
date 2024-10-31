// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023-2026 Intel Corporation */
#include <linux/bug.h>
#include <linux/cleanup.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/sprintf.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uuid.h>

#include "issei_dev.h"
#include "fw_client.h"

static ssize_t id_show(struct device *device,
		       struct device_attribute *attr, char *buf)
{
	struct issei_fw_client *fw_cl =
		container_of(attr, struct issei_fw_client, id_attr);

	return sysfs_emit(buf, "%u\n", fw_cl->id);
}
static DEVICE_ATTR_RO(id);

static ssize_t ver_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct issei_fw_client *fw_cl =
		container_of(attr, struct issei_fw_client, ver_attr);

	return sysfs_emit(buf, "%u\n", fw_cl->ver);
}
static DEVICE_ATTR_RO(ver);

static ssize_t uuid_show(struct device *device,
			 struct device_attribute *attr, char *buf)
{
	struct issei_fw_client *fw_cl =
		container_of(attr, struct issei_fw_client, uuid_attr);

	return sysfs_emit(buf, "%pUb\n", &fw_cl->uuid);
}
static DEVICE_ATTR_RO(uuid);

static ssize_t mtu_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct issei_fw_client *fw_cl =
		container_of(attr, struct issei_fw_client, mtu_attr);

	return sysfs_emit(buf, "%u\n", fw_cl->mtu);
}
static DEVICE_ATTR_RO(mtu);

static void issei_fw_cl_init(struct issei_fw_client *fw_cl, u16 id, u8 ver, const uuid_t *uuid,
			     u32 mtu, u32 flags)
{
	memset(fw_cl, 0, sizeof(*fw_cl));
	INIT_LIST_HEAD(&fw_cl->list);
	fw_cl->id = id;
	fw_cl->ver = ver;
	fw_cl->uuid = *uuid;
	fw_cl->mtu = mtu;
	fw_cl->flags = flags;
}

/**
 * issei_fw_cl_create - create firmware client object and add to list
 * @idev: issei device object
 * @id: firmware client id
 * @ver: firmware client version
 * @uuid: firmware client unique id
 * @mtu: firmware client maximum message size
 * @flags: firmware client flags
 *
 * Return: pointer to newly created object on success, ERR_PTR on failure
 */
struct issei_fw_client *issei_fw_cl_create(struct issei_device *idev, u16 id, u8 ver,
					   const uuid_t *uuid, u32 mtu, u32 flags)
{
	int ret;

	struct issei_fw_client *fw_cl __free(kfree) = kzalloc(sizeof(*fw_cl), GFP_KERNEL);
	if (!fw_cl)
		return ERR_PTR(-ENOMEM);

	guard(mutex)(&idev->fw_client_lock);

	issei_fw_cl_init(fw_cl, id, ver, uuid, mtu, flags);

	fw_cl->attrs[0] = (struct attribute *)&fw_cl->id_attr;
	fw_cl->attrs[1] = (struct attribute *)&fw_cl->ver_attr;
	fw_cl->attrs[2] = (struct attribute *)&fw_cl->uuid_attr;
	fw_cl->attrs[3] = (struct attribute *)&fw_cl->mtu_attr;
	fw_cl->attrs[4] = NULL;

	fw_cl->id_attr = dev_attr_id;
	fw_cl->ver_attr = dev_attr_ver;
	fw_cl->uuid_attr = dev_attr_uuid;
	fw_cl->mtu_attr = dev_attr_mtu;
	fw_cl->attr_grp.name = kasprintf(GFP_KERNEL, "fw_client:%02u", id);
	if (!fw_cl->attr_grp.name)
		return ERR_PTR(-ENOMEM);
	fw_cl->attr_grp.attrs = fw_cl->attrs;

	list_add_tail(&fw_cl->list, &idev->fw_client_list);

	ret = sysfs_create_group(&idev->dev.kobj, &fw_cl->attr_grp);
	if (ret)
		dev_err(&idev->dev, "Attr group for client %pUb failed %d\n", uuid, ret);

	dev_dbg(&idev->dev, "FW client %pUb created\n", uuid);
	return no_free_ptr(fw_cl);
}

static void __issei_fw_cl_remove(struct issei_device *idev, struct issei_fw_client *fw_cl)
{
	WARN(fw_cl->cl, "Removing connected client!\n");

	dev_dbg(&idev->dev, "FW client %pUb will be removed\n", &fw_cl->uuid);

	sysfs_remove_group(&idev->dev.kobj, &fw_cl->attr_grp);
	kfree(fw_cl->attr_grp.name);
	list_del(&fw_cl->list);
	kfree(fw_cl);
}

/**
 * issei_fw_cl_remove_all - remove all firmware client objects
 * @idev: issei device object
 */
void issei_fw_cl_remove_all(struct issei_device *idev)
{
	struct issei_fw_client *fw_cl, *next;

	guard(mutex)(&idev->fw_client_lock);

	list_for_each_entry_safe(fw_cl, next, &idev->fw_client_list, list)
		__issei_fw_cl_remove(idev, fw_cl);
}

static struct issei_fw_client *__issei_fw_cl_find_by_uuid(struct issei_device *idev,
							  const uuid_t *uuid)
{
	struct issei_fw_client *fw_cl;

	list_for_each_entry(fw_cl, &idev->fw_client_list, list)
		if (uuid_equal(&fw_cl->uuid, uuid))
			return fw_cl;
	return NULL;
}

/**
 * issei_fw_cl_find_by_uuid - find firmware client by uuid
 * @idev: issei device object
 * @uuid: uuid to search by it
 *
 * Return: pointer to firmware client object if found, NULL on failure
 */
struct issei_fw_client *issei_fw_cl_find_by_uuid(struct issei_device *idev, const uuid_t *uuid)
{
	guard(mutex)(&idev->fw_client_lock);

	return __issei_fw_cl_find_by_uuid(idev, uuid);
}
