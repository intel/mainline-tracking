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
	u32 old;

	spin_lock_irqsave(&xe->soc_remapper.lock, flags);
	old = xe_mmio_rmw32(xe_root_tile_mmio(xe), reg, mask, val);
	xe->soc_remapper.state = (old & ~mask) | val;
	xe->soc_remapper.state_initialized = true;
	spin_unlock_irqrestore(&xe->soc_remapper.lock, flags);
}

void xe_soc_remapper_set_telem_region(struct xe_device *xe, u32 index)
{
	xe_soc_remapper_set_region(xe, SG_REMAP_INDEX1, SG_REMAP_TELEM_MASK,
				   REG_FIELD_PREP(SG_REMAP_TELEM_MASK, index));
}

void xe_soc_remapper_set_sysctrl_region(struct xe_device *xe, u32 index)
{
	xe_soc_remapper_set_region(xe, SG_REMAP_INDEX1, SG_REMAP_SYSCTRL_MASK,
				   REG_FIELD_PREP(SG_REMAP_SYSCTRL_MASK, index));
}

void xe_soc_remapper_resume(struct xe_device *xe)
{
	unsigned long flags;

	if (!xe->soc_remapper.state_initialized)
		return;

	spin_lock_irqsave(&xe->soc_remapper.lock, flags);
	xe_mmio_write32(xe_root_tile_mmio(xe), SG_REMAP_INDEX1, xe->soc_remapper.state);
	spin_unlock_irqrestore(&xe->soc_remapper.lock, flags);
}

int xe_soc_remapper_init(struct xe_device *xe)
{
	spin_lock_init(&xe->soc_remapper.lock);

	return 0;
}
