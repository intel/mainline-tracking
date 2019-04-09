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

union icl_memwnd2_slot_type {
	u32 val;
	struct {
		u32 resource_id:8;
		u32 type:24;
	};
};

struct icl_memwnd2_desc {
	u32 resource_id;
	union icl_memwnd2_slot_type slot_id;
	u32 vma;
} __packed;

#define ICL_MEMWND2_SLOTS_COUNT	15

struct icl_memwnd2 {
	union {
		struct icl_memwnd2_desc slot_desc[ICL_MEMWND2_SLOTS_COUNT];
		u8 rsvd[PAGE_SIZE];
	};
	u8 slot_array[ICL_MEMWND2_SLOTS_COUNT][PAGE_SIZE];
} __packed;

#define ICL_SLOT_UNUSED \
	((union icl_memwnd2_slot_type) { 0x00000000U })
#define ICL_SLOT_CRITICAL_LOG \
	((union icl_memwnd2_slot_type) { 0x54524300U })
#define ICL_SLOT_DEBUG_LOG \
	((union icl_memwnd2_slot_type) { 0x474f4c00U })
#define ICL_SLOT_GDB_STUB \
	((union icl_memwnd2_slot_type) { 0x42444700U })
#define ICL_SLOT_BROKEN \
	((union icl_memwnd2_slot_type) { 0x44414544U })

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

static int icl_slot_offset(struct sst_dsp *dsp,
		union icl_memwnd2_slot_type slot_type)
{
	struct icl_memwnd2_desc desc[ICL_MEMWND2_SLOTS_COUNT];
	int i;

	memcpy_fromio(&desc, dsp->addr.sram2, sizeof(desc));

	for (i = 0; i < ICL_MEMWND2_SLOTS_COUNT; i++)
		if (desc[i].slot_id.val == slot_type.val)
			return offsetof(struct icl_memwnd2, slot_array) +
				skl_log_buffer_offset(dsp, i);
	return -ENXIO;
}

__maybe_unused static unsigned int
icl_log_buffer_offset(struct sst_dsp *dsp, u32 core)
{
	union icl_memwnd2_slot_type slot_type = ICL_SLOT_DEBUG_LOG;
	int ret;

	slot_type.resource_id = core;
	ret = icl_slot_offset(dsp, slot_type);
	if (ret) {
		dev_dbg(dsp->dev,
			"No slot offset found for: %d\n", slot_type.val);
		return 0;
	}

	return ret;
}
