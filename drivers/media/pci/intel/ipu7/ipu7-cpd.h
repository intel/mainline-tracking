/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2015 - 2024 Intel Corporation
 */

#ifndef IPU7_CPD_H
#define IPU7_CPD_H

struct ipu7_device;

struct ipu7_cpd_hdr {
	u32 hdr_mark;
	u32 ent_cnt;
	u8 hdr_ver;
	u8 ent_ver;
	u8 hdr_len;
	u8 rsvd;
	u8 partition_name[4];
	u32 crc32;
} __packed;

struct ipu7_cpd_ent {
	u8 name[12];
	u32 offset;
	u32 len;
	u8 rsvd[4];
} __packed;

struct ipu7_cpd_metadata_hdr {
	u32 type;
	u32 len;
} __packed;

struct ipu7_cpd_metadata_attr {
	struct ipu7_cpd_metadata_hdr hdr;
	u8 compression_type;
	u8 encryption_type;
	u8 rsvd[2];
	u32 uncompressed_size;
	u32 compressed_size;
	u32 module_id;
	u8 hash[48];
} __packed;

struct ipu7_cpd_metadata_ipl {
	struct ipu7_cpd_metadata_hdr hdr;
	u32 param[4];
	u8 rsvd[8];
} __packed;

struct ipu7_cpd_metadata {
	struct ipu7_cpd_metadata_attr attr;
	struct ipu7_cpd_metadata_ipl ipl;
} __packed;

int ipu7_cpd_validate_cpd_file(struct ipu7_device *isp,
			       const void *cpd_file,
			       unsigned long cpd_file_size);

int ipu7_cpd_copy_binary(const void *cpd, const char *name,
			 void *code_region, u32 *entry);
#endif /* IPU7_CPD_H */
