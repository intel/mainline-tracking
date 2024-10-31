/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023-2025 Intel Corporation */
#ifndef _ISSEI_HAM_H_
#define _ISSEI_HAM_H_

#include <linux/types.h>

struct issei_device;

int issei_ham_send_start_req(struct issei_device *idev);
int issei_ham_send_clients_req(struct issei_device *idev);

static inline int issei_is_ham_rsp(u16 fw_id, u16 host_id)
{
	return fw_id == 0 && host_id == 0;
}

int issei_ham_process_ham_rsp(struct issei_device *idev, u32 status, const u8 *buf, size_t length);

#endif /* _ISSEI_HAM_H_ */
