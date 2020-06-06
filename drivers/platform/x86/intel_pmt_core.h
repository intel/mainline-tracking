// SPDX-License-Identifier: GPL-2.0
#ifndef _INTEL_PMT_CORE_H
#define _INTEL_PMT_CORE_H

#include <linux/types.h>

/* PMT device names */
#define TELEM_DEV_NAME		"pmt_telemetry"
#define WATCHER_DEV_NAME	"pmt_watcher"
#define CRASHLOG_DEV_NAME	"pmt_crashlog"

#define GUID_OFFSET		0x4
#define BASE_OFFSET		0x8
#define CRASHLOG_SIZE_OFFSET	0xC

enum pmt_cap {
	PMT_CAP_TELEM = 2,
	PMT_CAP_WATCHER,
	PMT_CAP_CRASHLOG
};

struct pmt_header {
	u8	access_type;
	u8	type;
	u16	size;
	bool	irq_support;
	u32	guid;
	u32	base_offset;
	u8	bir;
	u32	crashlog_size;
	u8	crashlog_flag;
	u8	crashlog_version;
};

bool pmt_is_early_client_hw(struct device *dev);
void pmt_populate_header(enum pmt_cap cap,void __iomem *disc_offset,
			 struct pmt_header *header);
int pmt_get_base_address(struct device *dev, struct pmt_header *header,
			 struct resource *header_res, unsigned long *address);

#endif
