/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_COMMOM_ABI_H
#define IPU7_FW_COMMOM_ABI_H

#include <linux/types.h>

#define CHECK_SIZE_ALIGNMENT(struct_type, alignment) do { \
	const u8 arr[((sizeof(struct_type) % (alignment)) == 0U) ? \
			  1 : -1]; (void)arr;\
	} while (false)

#define CHECK_ALIGN16(struct_type) \
	CHECK_SIZE_ALIGNMENT(struct_type, sizeof(u16))
#define CHECK_ALIGN32(struct_type) \
	CHECK_SIZE_ALIGNMENT(struct_type, sizeof(u32))
#define CHECK_ALIGN64(struct_type) \
	CHECK_SIZE_ALIGNMENT(struct_type, sizeof(u64))

#define IA_GOFO_CL_SIZE		(64U)

#define CHECK_ALIGN_CL(struct_type) \
	CHECK_SIZE_ALIGNMENT(struct_type, IA_GOFO_CL_SIZE)

#define IA_GOFO_PAGE_SIZE	(4096U)
#define IA_GOFO_SRAM_PAGE_SIZE	(256U)

#define CHECK_ALIGN_PAGE(struct_type) \
	CHECK_SIZE_ALIGNMENT(struct_type, IA_GOFO_PAGE_SIZE)

#define CHECK_EQUAL(lval, rval) do { \
	const u8 arr[((lval) == (rval)) ? 1 : -1]; (void)arr;\
	} while (false)

#define CHECK_SMALLER_OR_EQUAL(lval, rval) do { \
	const u8 arr[((lval) <= (rval)) ? 1 : -1]; (void)arr;\
	} while (false)

#define CHECK_SIZE(struct_type, size)	CHECK_EQUAL(sizeof(struct_type), size)

#pragma pack(push, 1)
typedef u32	ia_gofo_addr_t;

#define IA_GOFO_ADDR_NULL	(0U)

struct ia_gofo_version_s {
	u8 patch;
	u8 subminor;
	u8 minor;
	u8 major;
};

#define IA_GOFO_MSG_VERSION_INIT(major_val, minor_val, subminor_val, patch_val)\
	{.major = (major_val), .minor = (minor_val), .subminor = \
	(subminor_val), .patch = (patch_val)}

#define IA_GOFO_MSG_VERSION_LIST_MAX_ENTRIES	(3U)
#define IA_GOFO_MSG_RESERVED_SIZE		(3U)

struct ia_gofo_msg_version_list {
	u8 num_versions;
	u8 reserved[IA_GOFO_MSG_RESERVED_SIZE];
	struct ia_gofo_version_s versions[IA_GOFO_MSG_VERSION_LIST_MAX_ENTRIES];
};

#pragma pack(pop)

static inline void ia_gofo_common_abi_test_func(void)
{
	CHECK_ALIGN32(struct ia_gofo_version_s);
	CHECK_ALIGN32(struct ia_gofo_msg_version_list);
}

#define TLV_TYPE_PADDING		(0U)

#pragma pack(push, 1)

#define IA_GOFO_ABI_BITS_PER_BYTE	(8U)

struct ia_gofo_tlv_header {
	u16 tlv_type;
	u16 tlv_len32;
};

struct ia_gofo_tlv_list {
	u16 num_elems;
	u16 head_offset;
};

#define TLV_ITEM_ALIGNMENT	((u32)sizeof(u32))
#define TLV_MSG_ALIGNMENT	((u32)sizeof(u64))
#define TLV_LIST_ALIGNMENT	TLV_ITEM_ALIGNMENT
#pragma pack(pop)

static inline void ia_gofo_msg_tlv_test_func(void)
{
	CHECK_ALIGN16(struct ia_gofo_tlv_header);
	CHECK_ALIGN32(struct ia_gofo_tlv_list);
}

#define IA_GOFO_MODULO(dividend, divisor) ((dividend) % (divisor))

#define IA_GOFO_MSG_ERR_MAX_DETAILS		(4U)
#define IA_GOFO_MSG_ERR_OK			(0U)
#define IA_GOFO_MSG_ERR_UNSPECIFED		(0xffffffffU)
#define IA_GOFO_MSG_ERR_GROUP_UNSPECIFIED	(0U)
#define IA_GOFO_MSG_ERR_IS_OK(err)	(IA_GOFO_MSG_ERR_OK == (err).err_code)

#pragma pack(push, 1)
struct ia_gofo_msg_err {
	u32 err_group;
	u32 err_code;
	u32 err_detail[IA_GOFO_MSG_ERR_MAX_DETAILS];
};

#pragma pack(pop)

#define IA_GOFO_MSG_ERR_GROUP_APP_EXT_START	(16U)
#define IA_GOFO_MSG_ERR_GROUP_MAX		(31U)
#define IA_GOFO_MSG_ERR_GROUP_INTERNAL_START	(IA_GOFO_MSG_ERR_GROUP_MAX + 1U)
#define IA_GOFO_MSG_ERR_GROUP_RESERVED	IA_GOFO_MSG_ERR_GROUP_UNSPECIFIED
#define IA_GOFO_MSG_ERR_GROUP_GENERAL		1

static inline void ia_gofo_msg_err_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_err);
}

enum ia_gofo_msg_err_general {
	IA_GOFO_MSG_ERR_GENERAL_OK = IA_GOFO_MSG_ERR_OK,
	IA_GOFO_MSG_ERR_GENERAL_MSG_TOO_SMALL = 1,
	IA_GOFO_MSG_ERR_GENERAL_MSG_TOO_LARGE = 2,
	IA_GOFO_MSG_ERR_GENERAL_DEVICE_STATE = 3,
	IA_GOFO_MSG_ERR_GENERAL_ALIGNMENT = 4,
	IA_GOFO_MSG_ERR_GENERAL_INDIRECT_REF_PTR_INVALID = 5,
	IA_GOFO_MSG_ERR_GENERAL_INVALID_MSG_TYPE = 6,
	IA_GOFO_MSG_ERR_GENERAL_SYSCOM_FAIL = 7,
	IA_GOFO_MSG_ERR_GENERAL_N
};

#pragma pack(push, 1)
#define IA_GOFO_MSG_TYPE_RESERVED	0
#define IA_GOFO_MSG_TYPE_INDIRECT	1
#define IA_GOFO_MSG_TYPE_LOG		2
#define IA_GOFO_MSG_TYPE_GENERAL_ERR	3

struct ia_gofo_msg_header {
	struct ia_gofo_tlv_header tlv_header;
	struct ia_gofo_tlv_list msg_options;
	u64 user_token;
};

struct ia_gofo_msg_header_ack {
	struct ia_gofo_msg_header header;
	struct ia_gofo_msg_err err;

};

struct ia_gofo_msg_general_err {
	struct ia_gofo_msg_header_ack header;
};

#pragma pack(pop)

static inline void ia_gofo_msg_header_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_header);
	CHECK_ALIGN64(struct ia_gofo_msg_header_ack);
	CHECK_ALIGN64(struct ia_gofo_msg_general_err);
}

#pragma pack(push, 1)
enum ia_gofo_msg_link_streaming_mode {
	IA_GOFO_MSG_LINK_STREAMING_MODE_SOFF = 0,
	IA_GOFO_MSG_LINK_STREAMING_MODE_DOFF = 1,
	IA_GOFO_MSG_LINK_STREAMING_MODE_BCLM = 2,
	IA_GOFO_MSG_LINK_STREAMING_MODE_BCSM_FIX = 3,
	IA_GOFO_MSG_LINK_STREAMING_MODE_N
};

enum ia_gofo_soc_pbk_instance_id {
	IA_GOFO_SOC_PBK_ID0 = 0,
	IA_GOFO_SOC_PBK_ID1 = 1,
	IA_GOFO_SOC_PBK_ID_N
};

#define IA_GOFO_MSG_LINK_PBK_MAX_SLOTS	(2U)

struct ia_gofo_msg_indirect {
	struct ia_gofo_msg_header header;
	struct ia_gofo_tlv_header ref_header;
	ia_gofo_addr_t ref_msg_ptr;
};

#pragma pack(pop)

static inline void ia_gofo_msg_indirect_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_indirect);
}

#pragma pack(push, 1)
#define IA_GOFO_MSG_LOG_MAX_PARAMS	(4U)
#define IA_GOFO_MSG_LOG_DOC_FMT_ID_MIN	(0U)

#define IA_GOFO_MSG_LOG_DOC_FMT_ID_MAX	(4095U)
#define IA_GOFO_MSG_LOG_FMT_ID_INVALID	(0xfffffffU)

struct ia_gofo_msg_log_info {
	u16 log_counter;
	u8 msg_parameter_types;
	u8 is_out_of_order;
	u32 fmt_id;
	u32 params[IA_GOFO_MSG_LOG_MAX_PARAMS];
};

struct ia_gofo_msg_log_info_ts {
	u64 msg_ts;
	struct ia_gofo_msg_log_info log_info;
};

struct ia_gofo_msg_log {
	struct ia_gofo_msg_header header;
	struct ia_gofo_msg_log_info_ts log_info_ts;
};

#pragma pack(pop)

static inline void ia_gofo_msg_log_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_log);
	CHECK_ALIGN64(struct ia_gofo_msg_log_info);
	CHECK_ALIGN64(struct ia_gofo_msg_log_info_ts);
}

#define IA_GOFO_MSG_ABI_OUT_ACK_QUEUE_ID	(0U)
#define IA_GOFO_MSG_ABI_OUT_LOG_QUEUE_ID	(1U)
#define IA_GOFO_MSG_ABI_IN_DEV_QUEUE_ID		(2U)

#endif
