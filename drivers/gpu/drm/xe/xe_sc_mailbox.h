/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2025 Intel Corporation
 */

#ifndef __XE_SC_MAILBOX_H__
#define __XE_SC_MAILBOX_H__

#include <linux/types.h>

struct xe_sc;

/**
 * struct xe_sc_mailbox_mkhi_msg_hdr - MKHI protocol message header
 */
struct xe_sc_mailbox_mkhi_msg_hdr {
	/** @group_id: Message group identifier */
	u32 group_id    : 8;
	/** @command: Command identifier within the group */
	u32 command     : 7;
	/** @is_response: Response flag - 0 for request, 1 for response */
	u32 is_response : 1;
	/** @reserved: Reserved field, must be zero */
	u32 reserved    : 8;
	/** @result: Result code from firmware */
	u32 result      : 8;
} __packed;

/**
 * struct xe_sc_mailbox_app_msg_hdr - Application message header
 */
struct xe_sc_mailbox_app_msg_hdr {
	/** @group_id: Application group identifier */
	u32 group_id  : 8;
	/** @command: Specific command within the application group */
	u32 command   : 8;
	/** @version: Protocol version */
	u32 version   : 8;
	/** @reserved: Reserved field, must be zero */
	u32 reserved  : 8;
} __packed;

/**
 * struct xe_sc_mailbox_command - System Controller mailbox command structure
 */
struct xe_sc_mailbox_command {
	/** @header: Application message header containing command information */
	struct xe_sc_mailbox_app_msg_hdr header;
	/** @data_in: Pointer to input payload data (can be NULL if no input data) */
	void *data_in;
	/** @data_in_len: Size of input payload in bytes (0 if no input data) */
	size_t data_in_len;
	/** @data_out: Pointer to output buffer for response data (can be NULL if no response) */
	void *data_out;
	/** @data_out_len: Size of output buffer in bytes (0 if no response expected) */
	size_t data_out_len;
};

int xe_sc_mailbox_send_command(void *handle, void *cmd_buffer, size_t *rdata_len);

#endif /* __XE_SC_MAILBOX_H__ */
