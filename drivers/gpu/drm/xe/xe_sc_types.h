/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2025 Intel Corporation
 */

#ifndef _XE_SC_TYPES_H_
#define _XE_SC_TYPES_H_

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "xe_device_types.h"
#include "xe_sc_mailbox.h"

struct xe_device;

/**
 * struct xe_sc - System Controller driver context
 */
struct xe_sc {
	/** @xe: Back pointer to xe_device */
	struct xe_device *xe;
	/** @mmio: MMIO region for SC registers */
	struct xe_mmio mmio;

	/** @cmd_lock: Mutex protecting mailbox command operations */
	struct mutex cmd_lock;

	/** @phase_bit: MKHI message boundary phase toggle bit */
	u32 phase_bit;
};

#endif /* _XE_SC_TYPES_H_ */
