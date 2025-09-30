/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2016 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_PSYS_H
#define IPU7_FW_PSYS_H

#include "abi/ipu7_fw_common_abi.h"
#include "abi/ipu7_fw_msg_abi.h"

#include "ipu7-syscom.h"

#include <uapi/linux/ipu7-psys.h>

#define IPU_PSYS_MAX_GRAPH_NUMS	(8U)
#define IPU_PSYS_NUM_STREAMS	IPU_PSYS_MAX_GRAPH_NUMS

struct ipu7_msg_to_str {
	const enum ipu7_msg_type type;
	const char *msg;
};

struct ipu7_psys;
struct ipu7_psys_stream;
struct ipu_psys_task_queue;

int ipu7_fw_psys_init(struct ipu7_psys *psys);
void ipu7_fw_psys_release(struct ipu7_psys *psys);
int ipu7_fw_psys_open(struct ipu7_psys *psys);
void ipu7_fw_psys_close(struct ipu7_psys *psys);
int ipu7_fw_psys_graph_open(const struct ipu_psys_graph_info *graph,
			    struct ipu7_psys *psys,
			    struct ipu7_psys_stream *ip);
int ipu7_fw_psys_graph_close(u8 graph_id, struct ipu7_psys *psys);
int ipu7_fw_psys_task_request(const struct ipu_psys_task_request *task,
			      struct ipu7_psys_stream *ip,
			      struct ipu_psys_task_queue *tq,
			      struct ipu7_psys *psys);
int ipu7_fw_psys_event_handle(struct ipu7_psys *psys, u8 *buf_ptr);
int ipu7_fw_psys_get_log(struct ipu7_psys *psys);
#endif /* IPU7_FW_PSYS_H */
