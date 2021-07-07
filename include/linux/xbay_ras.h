/* SPDX-License-Identifier: GPL-2.0 only */
/*
 * Intel xBay RAS: User common Header file
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#define THB_DDR_SLICE_NUM	4
#define THB_DDR_MCS_PER_SLICE	4

#define THB_CSRAM_BANK_NUM	8

#define NUM_CACHES	2

/* EDAC Info size */
#define EDAC_STS_SZ		4
#define DDR_EDAC_INFO_SZ	1024
#define CSRAM_EDAC_INFO_SZ	128
#define CPU_EDAC_INFO_SZ	48
#define WDOG_INFO_SZ		4
#define TOTAL_INFO_SZ		1208

/* EDAC Info structure offsets for user space */
#define EDAC_STS_OFF	0
#define DDR_EDAC_OFF	4
#define CSRAM_EDAC_OFF	1028
#define CPU_EDAC_OFF	1156
#define WDOG_OFF	1204

#define RAS_EDAC_INTR_STAT_0_MASK	0xFFFFF
#define RAS_EDAC_INTR_STAT_0_DDR_MASK	0xFFFF
#define RAS_EDAC_INTR_STAT_0_CSRAM_MASK 0x90000

/* Reserved for Watchdog */
#define RAS_RESV_WDOG_INTR	(1 << 29)

/* Only for Test purposes */
#define RAS_TST_DDR_INTR	(1 << 0)
#define RAS_TST_CSRAM_INTR	(1 << 16)
#define RAS_TST_CPU_INTR	(1 << 28)

struct ddr_edac_err_info_t {
	uint32_t row;
	uint32_t col;
	uint32_t bank;
	uint32_t bit_pos;
	uint32_t data_pattern;
	uint32_t bank_grp_num;
	uint32_t blk_num;
};

struct ddr_edac_status_t {
	uint32_t ce_cnt;
	uint32_t ue_cnt;
	struct ddr_edac_err_info_t ce_info;
	struct ddr_edac_err_info_t ue_info;
};

struct csram_edac_status_t {
	uint16_t sbit_cnt;
	uint16_t dbit_cnt;
	uint32_t err_status;
	uint64_t err_acsramess;
};

struct cpu_edac_info_t {
	uint32_t cpu;
	uint32_t way;
	uint32_t repeat_err;
	uint32_t other_err;
	uint32_t index;
	uint32_t ram_id;
};

struct edac_info_t {
	uint32_t edac_event;
	struct ddr_edac_status_t
		ddr_edac_status[THB_DDR_SLICE_NUM][THB_DDR_MCS_PER_SLICE];
	struct csram_edac_status_t
		csram_edac_status[THB_CSRAM_BANK_NUM];
	struct cpu_edac_info_t
		cpu_edac_info[NUM_CACHES];
	uint32_t wdog_sts;
} __attribute__ ((packed));

enum xbay_ras_wdog_event {
	XBAY_RAS_NS_WDOG_TH_TO = 1,
	XBAY_RAS_NS_WDOG_TO,
};

#ifdef CONFIG_XBAY_RAS
void xbay_ras_notify_wdog_event(enum xbay_ras_wdog_event evnt);
#endif

