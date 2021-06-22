// SPDX-License-Identifier: GPL-2.0 only
/*
 * Intel xBay RAS: PCIe RAS DES SW
 *
 * Copyright (C) 2021 Intel Corporation
 */

#include "ras_core.h"

#ifdef CONFIG_RAS_PCIE_RASDES
#define THB_PCIE_EP_RASDES_CAP_OFF	0x1D0

#define RASDES_EVENT_COUNTER_REG	(THB_PCIE_EP_RASDES_CAP_OFF + 0x4)
#define RASDES_EVENT_DATA_REG		(THB_PCIE_EP_RASDES_CAP_OFF + 0x8)

#define PCIE_RASDES_NUM_GROUPS		8
#define PCIE_RASDES_NUM_ERR_GROUPS	4
#define PCIE_RASDES_NUM_STAT_GROUPS	4

#define PCIE_RASDES_MAX_GROUP_CNT	30
#endif

#define PCIE_PF0_CORR_ERR_MASK		0x114
#define PCIE_PF0_UNCORR_ERR_MASK	0x108

#define PCIE_PF0_CORR_INT_ERR		BIT(14)
#define PCIE_PF0_UNCORR_INT_ERR		BIT(22)

#define PCIE_AER_REPORT_ERROR		0x2F0

#define PCIE_AER_REPORT_UNCORR		BIT(1)
#define PCIE_AER_REPORT_ERR_GEN		BIT(0)

static void __iomem *pcie_ep_apb_base;
static void __iomem *pcie_ep_pf0_cfg_base;

#ifdef CONFIG_RAS_PCIE_RASDES
/* RAS DES counter group 0 */
struct pcie_rasdes_grp0_t {
	u8 ebuf_overflow;
	u8 ebuf_underrun;
	u8 decode_error;
	u8 running_disparity_error;
	u8 skp_os_parity_error;
	u8 sync_header_error;
	u8 rx_valid_deassertion;
	u8 dummy;
};

/* RAS DES counter group 1 */
struct pcie_rasdes_grp1_t {
	u8 detect_ei_infer;
	u8 receiver_error;
	u8 rx_recovery_request;
	u8 n_fts_timeout;
	u8 framing_error;
	u8 deskew_error;
	u8 dummy1;
	u8 dummy2;
};

/* RAS DES counter group 2 */
struct pcie_rasdes_grp2_t {
	u8 bad_tlp;
	u8 lcrc_error;
	u8 bad_dllp;
	u8 replay_number_rollover;
	u8 replay_timeout;
	u8 rx_nak_dllp;
	u8 tx_nak_dllp;
	u8 retry_tlp;
};

/* RAS DES counter group 3 */
struct pcie_rasdes_grp3_t {
	u8 fc_timeout;
	u8 poisoned_tlp;
	u8 ecrc_error;
	u8 unsupported_request;
	u8 completer_abort;
	u8 completion_timeout;
	u8 dummy1;
	u8 dummy2;
};

/* RAS DES counter group 4 */
struct pcie_rasdes_grp4_t {
	u8 ebuf_skp_add;
	u8 ebuf_skp_del;
	u8 dummy1;
	u8 dummy2;
};

/* RAS DES counter group 5 */
struct pcie_rasdes_grp5_t {
	u32 l0_to_recovery_entry;
	u32 l1_to_recovery_entry;
	u32 tx_l0s_entry;
	u32 rx_l0s_entry;
	u32 aspm_l1_reject;
	u32 l1_entry;
	u32 l1_cpm;
	u32 l11_entry;
	u32 l12_entry;
	u32 l1_short_duration;
	u32 l12_abort;
	u32 l2_entry;
	u32 speed_change;
	u32 link_width_change;
};

/* RAS DES counter group 6 */
struct pcie_rasdes_grp6_t {
	u32 tx_ack_dllp;
	u32 tx_update_fc_dllp;
	u32 rx_ack_dllp;
	u32 rx_update_fc_dllp;
	u32 rx_nullified_tlp;
	u32 tx_nullified_tlp;
	u32 rx_duplicate_tlp;
};

/* RAS DES counter group 7 */
struct pcie_rasdes_grp7_t {
	u32 tx_memory_write;
	u32 tx_memory_read;
	u32 tx_io_write;
	u32 tx_io_read;
	u32 tx_completion_without_data;
	u32 tx_completion_with_data;
	u32 tx_message_tlp;
	u32 tx_atomic;
	u32 tx_tlp_with_prefix;
	u32 rx_memory_write;
	u32 rx_memory_read;
	u32 rx_config_write;
	u32 rx_config_read;
	u32 rx_tx_io_write;
	u32 rx_io_read;
	u32 rx_completion_without_data;
	u32 rx_completion_with_data;
	u32 rx_message_tlp;
	u32 rx_atomic;
	u32 rx_tlp_with_prefix;
};

/* RAS DES counter values as written to register */
enum pcie_rasdes_cnt_t {
	/* group 0 */
	PCIE_RASDES_EBUF_OVERFLOW = 0x0000,
	PCIE_RASDES_EBUF_UNDERRUN = 0x0001,
	PCIE_RASDES_DECODE_ERROR = 0x0002,
	PCIE_RASDES_RUNNING_DISPARITY_ERROR = 0x0003,
	PCIE_RASDES_SKP_OS_PARITY_ERROR = 0x0004,
	PCIE_RASDES_SYNC_HEADER_ERROR = 0x0005,
	PCIE_RASDES_RX_VALID_DEASSERTION = 0x0006,

	/* group 1 - events 0 to 4 are reserved */
	PCIE_RASDES_DETECT_EI_INFER = 0x0105,
	PCIE_RASDES_RECEIVER_ERROR = 0x0106,
	PCIE_RASDES_RX_RECOVERY_REQUEST = 0x0107,
	PCIE_RASDES_N_FTS_TIMEOUT = 0x0108,
	PCIE_RASDES_FRAMING_ERROR = 0x0109,
	PCIE_RASDES_DESKEW_ERROR = 0x010A,

	/* group 2 */
	PCIE_RASDES_BAD_TLP = 0x0200,
	PCIE_RASDES_LCRC_ERROR = 0x0201,
	PCIE_RASDES_BAD_DLLP = 0x0202,
	PCIE_RASDES_REPLAY_NUMBER_ROLLOVER = 0x0203,
	PCIE_RASDES_REPLAY_TIMEOUT = 0x0204,
	PCIE_RASDES_RX_NAK_DLLP = 0x0205,
	PCIE_RASDES_TX_NAK_DLLP = 0x0206,
	PCIE_RASDES_RETRY_TLP = 0x0207,

	/* group 3 */
	PCIE_RASDES_FC_TIMEOUT = 0x0300,
	PCIE_RASDES_POISONED_TLP = 0x0301,
	PCIE_RASDES_ECRC_ERROR = 0x0302,
	PCIE_RASDES_UNSUPPORTED_REQUEST = 0x0303,
	PCIE_RASDES_COMPLETER_ABORT = 0x0304,
	PCIE_RASDES_COMPLETION_TIMEOUT = 0x0305,

	/* group 4 */
	PCIE_RASDES_EBUF_SKP_ADD = 0x0400,
	PCIE_RASDES_EBUF_SKP_DEL = 0x0401,

	/* group 5 */
	PCIE_RASDES_L0_TO_RECOVERY_ENTRY = 0x0500,
	PCIE_RASDES_L1_TO_RECOVERY_ENTRY = 0x0501,
	PCIE_RASDES_TX_L0S_ENTRY = 0x0502,
	PCIE_RASDES_RX_L0S_ENTRY = 0x0503,
	PCIE_RASDES_ASPM_L1_REJECT = 0x0504,
	PCIE_RASDES_L1_ENTRY = 0x0505,
	PCIE_RASDES_L1_CPM = 0x0506,
	PCIE_RASDES_L11_ENTRY = 0x0507,
	PCIE_RASDES_L12_ENTRY = 0x0508,
	PCIE_RASDES_L1_SHORT_DURATION = 0x0509,
	PCIE_RASDES_L12_ABORT = 0x050A,
	PCIE_RASDES_L2_ENTRY = 0x050B,
	PCIE_RASDES_SPEED_CHANGE = 0x050C,
	PCIE_RASDES_LINK_WIDTH_CHANGE = 0x050D,

	/* group 6 */
	PCIE_RASDES_TX_ACK_DLLP = 0x0600,
	PCIE_RASDES_TX_UPDATE_FC_DLLP = 0x0601,
	PCIE_RASDES_RX_ACK_DLLP = 0x0602,
	PCIE_RASDES_RX_UPDATE_FC_DLLP = 0x0603,
	PCIE_RASDES_RX_NULLIFIED_TLP = 0x0604,
	PCIE_RASDES_TX_NULLIFIED_TLP = 0x0605,
	PCIE_RASDES_RX_DUPLICATE_TLP = 0x0606,

	/* group 7 */
	PCIE_RASDES_TX_MEMORY_WRITE = 0x0700,
	PCIE_RASDES_TX_MEMORY_READ = 0x0701,
	PCIE_RASDES_TX_IO_WRITE = 0x0704,
	PCIE_RASDES_TX_IO_READ = 0x0705,
	PCIE_RASDES_TX_COMPLETION_WITHOUT_DATA = 0x0706,
	PCIE_RASDES_TX_COMPLETION_WITH_DATA = 0x0707,
	PCIE_RASDES_TX_MESSAGE_TLP = 0x0708,
	PCIE_RASDES_TX_ATOMIC = 0x0709,
	PCIE_RASDES_TX_TLP_WITH_PREFIX = 0x070A,
	PCIE_RASDES_RX_MEMORY_WRITE = 0x070B,
	PCIE_RASDES_RX_MEMORY_READ = 0x070C,
	PCIE_RASDES_RX_CONFIG_WRITE = 0x070D,
	PCIE_RASDES_RX_CONFIG_READ = 0x070E,
	PCIE_RASDES_RX_TX_IO_WRITE = 0x070F,
	PCIE_RASDES_RX_IO_READ = 0x0710,
	PCIE_RASDES_RX_COMPLETION_WITHOUT_DATA = 0x0711,
	PCIE_RASDES_RX_COMPLETION_WITH_DATA = 0x0712,
	PCIE_RASDES_RX_MESSAGE_TLP = 0x0713,
	PCIE_RASDES_RX_ATOMIC = 0x0714,
	PCIE_RASDES_RX_TLP_WITH_PREFIX = 0x0715,
};

struct pcie_rasdes_grp_desc_t {
	const u32 entry_size;
	u8 counter_offsets[PCIE_RASDES_MAX_GROUP_CNT];
};

static pcie_rasdes_grp_desc_t pcie_rasdes_grp_table[PCIE_RASDES_NUM_GROUPS] = {
	{
		.entry_size = sizeof(pcie_rasdes_grp0_t),
		.counter_offsets = {0, 1, 2, 3, 4, 5, 6, 0xff}
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp1_t),
		.counter_offsets = {5, 6, 7, 8, 9, 10, 0xff},
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp2_t),
		.counter_offsets = {0, 1, 2, 3, 4, 5, 6, 7, 0xff},
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp3_t),
		.counter_offsets = {0, 1, 2, 3, 4, 5, 0xff},
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp4_t),
		.counter_offsets = {0, 1, 0xff},
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp5_t),
		.counter_offsets = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
				    12, 13, 0xff},
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp6_t),
		.counter_offsets = {0, 1, 2, 3, 4, 5, 6, 0xff},
	},

	{
		.entry_size = sizeof(pcie_rasdes_grp7_t),
		.counter_offsets = {0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
				    14, 15, 16, 17, 18, 19, 20, 21, 0xff},
	},

};

static u32 pcie_rasdes_rd_counter(pcie_rasdes_cnt_t counter)
{
	u32 val;

	val = readl(pcie_ep_apb_base + RASDES_EVENT_COUNTER_REG);
	val |= (counter << 16);
	writel(val, pcie_ep_apb_base + RASDES_EVENT_COUNTER_REG);

	return readl(pcie_ep_apb_base + RASDES_EVENT_DATA_REG);
}

static void pcie_rasdes_enable(void)
{
	u32 val;

	val = readl(pcie_ep_apb_base + RASDES_EVENT_COUNTER_REG);
	val |= (0x7 << 2);
	writel(val, pcie_ep_apb_base + RASDES_EVENT_COUNTER_REG);
}

static void pcie_rasdes_get_group(u32 group)
{
	int i;
	u32 counter_val, counter_id;

	if (group >= PCIE_RASDES_NUM_GROUPS)
		return;

	for (i = 0;
	     pcie_rasdes_grp_table[group].counter_offsets[i] != 0xff; i++) {
		counter_id = (group << 8) |
				pcie_rasdes_grp_table[group].counter_offsets[i];
		counter_val =
			pcie_rasdes_rd_counter((pcie_rasdes_cnt_t)counter_id);
	}
}

static void pcie_rasdes_get_stat_groups(void)
{
	int i;

	for (i = PCIE_RASDES_NUM_ERR_GROUPS;
		i < PCIE_RASDES_NUM_GROUPS; i++)
		pcie_rasdes_get_group(i);
}

void xbay_pcie_rasdes_get_err_counters(void)
{
	int i;

	for (i = 0; i < PCIE_RASDES_NUM_ERR_GROUPS; i++)
		pcie_rasdes_get_group(i);
}
#endif

#ifdef DEBUG_RAS_PCIE
static ssize_t tst_pcie_ep_report_error_store(struct device *dev,
					      struct device_attribute *mattr,
					      const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);

	if (strncmp(data, "CE", 2) == 0) {
		dev_info(&pdev->dev, "Reporting Internal CE to Host\n");
		xbay_pcie_report_internal_error(0);
	} else {
		dev_info(&pdev->dev, "Reporting Internal UE to Host\n");
		xbay_pcie_report_internal_error(1);
	}

	return count;
}
static DEVICE_ATTR_WO(tst_pcie_ep_report_error);

static const struct attribute *pcie_rasdes_attrs[] = {
	&dev_attr_tst_pcie_ep_report_error.attr,
	NULL,
};

static const struct attribute_group pcie_rasdes_attributes = {
	.attrs = (struct attribute **)pcie_rasdes_attrs,
};
#endif

void xbay_pcie_report_internal_error(bool ce)
{
	if (!pcie_ep_apb_base)
		return;

	if (!ce)
		writel(PCIE_AER_REPORT_ERR_GEN,
		       pcie_ep_apb_base + PCIE_AER_REPORT_ERROR);
	else
		writel(PCIE_AER_REPORT_ERR_GEN | PCIE_AER_REPORT_UNCORR,
		       pcie_ep_apb_base + PCIE_AER_REPORT_ERROR);
}

int xbay_pcie_rasdes_probe(struct platform_device *pdev,
			   struct device_node *child)
{
	struct device_node *np;
	struct platform_device *pcie_ep_pdev;
	struct resource *res;
	u32 val;

	np = of_parse_phandle(child, "thb-pcie-ep", 0);
	if (!np) {
		dev_err(&pdev->dev, "Failed to find a THB PCIe EP node\n");
		return -ENODEV;
	}

	pcie_ep_pdev = of_find_device_by_node(np);
	if (!pcie_ep_pdev) {
		dev_info(&pdev->dev, "PCIe RASDES module probe deferred\n");
		of_node_put(np);
		return -EPROBE_DEFER;
	}

	/* Map PCIe EP Registers */
	res = platform_get_resource_byname(pcie_ep_pdev, IORESOURCE_MEM, "apb");
	pcie_ep_apb_base = devm_ioremap(&pdev->dev, res->start,
					(res->end - res->start + 1));
	if (IS_ERR(pcie_ep_apb_base)) {
		dev_err(&pdev->dev,
			"Failed to I/O Map PCIe EP APB memory");
		return PTR_ERR(pcie_ep_apb_base);
	}

	res = platform_get_resource_byname(pcie_ep_pdev, IORESOURCE_MEM, "dbi");
	pcie_ep_pf0_cfg_base = devm_ioremap(&pdev->dev, res->start,
					    (res->end - res->start + 1));
	if (IS_ERR(pcie_ep_pf0_cfg_base)) {
		dev_err(&pdev->dev,
			"Failed to I/O Map PCIe EP DBI memory");
		return PTR_ERR(pcie_ep_pf0_cfg_base);
	}

	/* Unmask PF0 AER Internal Errors */
	val = readl(pcie_ep_pf0_cfg_base + PCIE_PF0_CORR_ERR_MASK);
	val &= ~PCIE_PF0_CORR_INT_ERR;
	writel(val, pcie_ep_pf0_cfg_base + PCIE_PF0_CORR_ERR_MASK);

	val = readl(pcie_ep_pf0_cfg_base + PCIE_PF0_UNCORR_ERR_MASK);
	val &= ~PCIE_PF0_UNCORR_INT_ERR;
	writel(val, pcie_ep_pf0_cfg_base + PCIE_PF0_UNCORR_ERR_MASK);

#ifdef CONFIG_RAS_PCIE_RASDES
	pcie_rasdes_enable();
#endif

#ifdef DEBUG_RAS_PCIE
	err = sysfs_create_group(&pdev->dev.kobj, &pcie_rasdes_attributes);
	if (err < 0) {
		dev_err(&pdev->dev,
			"Failed to create PCIe RAS sysfs entries\n");
		return err;
	}
#endif

	dev_info(&pdev->dev, "PCIe RASDES module probed Successfully\n");

	return 0;
}
