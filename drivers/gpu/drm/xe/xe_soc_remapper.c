// SPDX-License-Identifier: MIT
/*
 * Copyright © 2025 Intel Corporation
 */

#include <linux/spinlock.h>

#include "regs/xe_soc_remapper_regs.h"
#include "xe_mmio.h"
#include "xe_soc_remapper.h"

static void xe_soc_remapper_set_region(struct xe_device *xe, struct xe_reg reg,
				       u32 mask, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&xe->soc_remapper.lock, flags);
	xe_mmio_rmw32(xe_root_tile_mmio(xe), reg, mask, val);
	spin_unlock_irqrestore(&xe->soc_remapper.lock, flags);
}

void xe_soc_remapper_set_telem_region(struct xe_device *xe, u32 index)
{
	xe_soc_remapper_set_region(xe, SG_REMAP_INDEX1, SG_REMAP_TELEM_MASK,
				   REG_FIELD_PREP(SG_REMAP_TELEM_MASK, index));
}

int xe_soc_remapper_init(struct xe_device *xe)
{
	spin_lock_init(&xe->soc_remapper.lock);

	return 0;
}
