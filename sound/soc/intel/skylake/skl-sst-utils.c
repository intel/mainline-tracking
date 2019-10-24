// SPDX-License-Identifier: GPL-2.0-only
/*
 *  skl-sst-utils.c - SKL sst utils functions
 *
 *  Copyright (C) 2016 Intel Corp
 */

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uuid.h>
#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl.h"

/* FW Extended Manifest Header id = $AE1 */
#define SKL_EXT_MANIFEST_HEADER_MAGIC   0x31454124

struct skl_module_entry *skl_find_module(struct skl_dev *skl,
		const guid_t *uuid)
{
	struct skl_module_entry *module_entries;
	int i;

	module_entries = skl->fw_modules_info->module_entry;

	for (i = 0; i < skl->fw_modules_info->count; i++)
		if (guid_equal(&module_entries[i].uuid, uuid))
			return &module_entries[i];

	return NULL;
}
EXPORT_SYMBOL(skl_find_module);

int skl_get_module_id(struct skl_dev *skl, const guid_t *uuid)
{
	struct skl_module_entry *module = skl_find_module(skl, uuid);

	return module ? module->module_id : -ENOENT;
}
EXPORT_SYMBOL(skl_get_module_id);

struct skl_ext_manifest_hdr {
	u32 id;
	u32 len;
	u16 version_major;
	u16 version_minor;
	u32 entries;
};

/*
 * some firmware binary contains some extended manifest. This needs
 * to be stripped in that case before we load and use that image.
 *
 * Get the module id for the module by checking
 * the table for the UUID for the module
 */
int skl_dsp_strip_extended_manifest(struct firmware *fw)
{
	struct skl_ext_manifest_hdr *hdr;

	/* check if fw file is greater than header we are looking */
	if (fw->size < sizeof(hdr)) {
		pr_err("%s: Firmware file small, no hdr\n", __func__);
		return -EINVAL;
	}

	hdr = (struct skl_ext_manifest_hdr *)fw->data;

	if (hdr->id == SKL_EXT_MANIFEST_HEADER_MAGIC) {
		fw->size -= hdr->len;
		fw->data += hdr->len;
	}

	return 0;
}

int skl_prepare_lib_load(struct skl_dev *skl, struct skl_lib_info *linfo,
		struct firmware *stripped_fw,
		unsigned int hdr_offset, int index)
{
	int ret;

	if (linfo->fw == NULL) {
		ret = request_firmware(&linfo->fw, linfo->name,
					skl->dev);
		if (ret < 0) {
			dev_err(skl->dev, "Request lib %s failed:%d\n",
				linfo->name, ret);
			return ret;
		}
	}

	stripped_fw->data = linfo->fw->data;
	stripped_fw->size = linfo->fw->size;
	skl_dsp_strip_extended_manifest(stripped_fw);

	return 0;
}

void skl_release_library(struct skl_lib_info *linfo, int lib_count)
{
	int i;

	/* library indices start from 1 to N. 0 represents base FW */
	for (i = 1; i < lib_count; i++) {
		if (linfo[i].fw) {
			release_firmware(linfo[i].fw);
			linfo[i].fw = NULL;
		}
	}
}

