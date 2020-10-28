/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xlink core header file.
 *
 * Copyright (C) 2018-2019 Intel Corporation
 *
 */

#ifndef XLINK_CORE_H_
#define XLINK_CORE_H_
#include "xlink-defs.h"

#define NUM_REG_EVENTS 4

enum xlink_error do_xlink_write_volatile(struct xlink_handle *handle,
					 u16 chan, u8 const *message,
					 u32 size, u32 user_flag);

enum xlink_error xlink_write_data_user(struct xlink_handle *handle,
				       u16 chan, u8 const *pmessage,
				       u32 size);
#endif /* XLINK_CORE_H_ */
