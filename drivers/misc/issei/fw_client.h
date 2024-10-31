/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023-2025 Intel Corporation */
#ifndef _ISSEI_FW_CLIENT_H_
#define _ISSEI_FW_CLIENT_H_

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/uuid.h>

struct issei_host_client;

/**
 * struct issei_fw_client - represents firmware queue
 * @list: link in firmware clients list
 * @id: firmware client id
 * @ver: firmware client version
 * @uuid: firmware client protocol id
 * @mtu: firmware client maximum buffer size
 * @flags: firmware client flags
 * @cl: pointer to host client, if connected
 * @attr_grp: client attribute group in sysfs
 * @id_attr: id attribute
 * @ver_attr: version attribute
 * @uuid_attr: protocol id attribute
 * @mtu_attr: maximum buffer size attribute
 * @attrs: array of loaded attributes (one position for end null)
 */
struct issei_fw_client {
	struct list_head list;
	u16 id;
	u8 ver;
	uuid_t uuid;
	u32 mtu;
	u32 flags;
	struct issei_host_client *cl;
	struct attribute_group attr_grp;
	struct device_attribute id_attr;
	struct device_attribute ver_attr;
	struct device_attribute uuid_attr;
	struct device_attribute mtu_attr;
	struct attribute *attrs[5];
};

struct issei_fw_client *issei_fw_cl_create(struct issei_device *idev, u16 id, u8 ver,
					   const uuid_t *uuid, u32 mtu, u32 flags);
void issei_fw_cl_remove(struct issei_device *idev, struct issei_fw_client *fw_cl);
void issei_fw_cl_remove_all(struct issei_device *idev);
struct issei_fw_client *issei_fw_cl_find_by_uuid(struct issei_device *idev, const uuid_t *uuid);

#endif /* _ISSEI_FW_CLIENT_H_ */
