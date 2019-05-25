// SPDX-License-Identifier: GPL-2.0
//
// icl-sst.c  --  ICL platform specific DSP operations
//
// Copyright (C) 2019, Intel Corporation.

#include <linux/kobject.h>
#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl-sst-ipc.h"
#include "skl.h"

__maybe_unused static int
icl_enable_logs(struct sst_dsp *dsp, enum skl_log_enable enable,
		u32 aging_period, u32 fifo_full_period,
		unsigned long resource_mask, u32 *priorities)
{
	struct skl_dev *skl = dsp->thread_context;
	struct icl_log_state_info *info;
	u32 size, num_libs = skl->fw_cfg.max_libs_count;
	int i, ret;

	size = struct_size(info, logs_priorities_mask, num_libs);
	info = kzalloc(size, GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->aging_timer_period = aging_period;
	info->fifo_full_timer_period = fifo_full_period;
	info->enable = enable;
	if (enable)
		for_each_set_bit(i, &resource_mask, GENMASK(num_libs, 0))
			info->logs_priorities_mask[i] = *priorities++;

	ret = skl_enable_logs_set(&skl->ipc, (u32 *)info, size);
	kfree(info);
	return ret;
}
