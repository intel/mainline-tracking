// SPDX-License-Identifier: GPL-2.0
#include "perf_regs.h"
#include "../../../util/perf_regs.h"

static const struct sample_reg sample_reg_masks[] = {
	SMPL_REG_END
};

void arch__intr_reg_mask(unsigned long *mask)
{
	*(uint64_t *)mask = PERF_REGS_MASK;
}

void arch__user_reg_mask(unsigned long *mask)
{
	*(uint64_t *)mask = PERF_REGS_MASK;
}

const struct sample_reg *arch__sample_reg_masks(void)
{
	return sample_reg_masks;
}
