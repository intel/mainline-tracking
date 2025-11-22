/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2025 Intel Corporation
 */

#ifndef _XE_SOC_REMAPPER_H_
#define _XE_SOC_REMAPPER_H_

#include <linux/types.h>

#include "xe_device_types.h"

int xe_soc_remapper_init(struct xe_device *xe);
void xe_soc_remapper_set_telem_region(struct xe_device *xe, u32 index);

#endif
