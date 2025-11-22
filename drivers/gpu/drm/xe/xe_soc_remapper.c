// SPDX-License-Identifier: MIT
/*
 * Copyright © 2025 Intel Corporation
 */

#include <linux/spinlock.h>

#include "xe_soc_remapper.h"

int xe_soc_remapper_init(struct xe_device *xe)
{
	spin_lock_init(&xe->soc_remapper.lock);

	return 0;
}
