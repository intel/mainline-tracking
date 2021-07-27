// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel XPCIe XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include "epf.h"

#define DMA_DBI_OFFSET			(0x380000)

/* PCIe DMA control 1 register definitions. */
#define DMA_CH_CONTROL1_CB_SHIFT	(0)
#define DMA_CH_CONTROL1_TCB_SHIFT	(1)
#define DMA_CH_CONTROL1_LLP_SHIFT	(2)
#define DMA_CH_CONTROL1_LIE_SHIFT	(3)
#define DMA_CH_CONTROL1_CS_SHIFT	(5)
#define DMA_CH_CONTROL1_CCS_SHIFT	(8)
#define DMA_CH_CONTROL1_LLE_SHIFT	(9)
#define DMA_CH_CONTROL1_FUNC_NUM_SHIFT	(12)
#define DMA_CH_CONTROL1_CB_MASK		(BIT(DMA_CH_CONTROL1_CB_SHIFT))
#define DMA_CH_CONTROL1_TCB_MASK	(BIT(DMA_CH_CONTROL1_TCB_SHIFT))
#define DMA_CH_CONTROL1_LLP_MASK	(BIT(DMA_CH_CONTROL1_LLP_SHIFT))
#define DMA_CH_CONTROL1_LIE_MASK	(BIT(DMA_CH_CONTROL1_LIE_SHIFT))
#define DMA_CH_CONTROL1_CS_MASK		(0x3 << DMA_CH_CONTROL1_CS_SHIFT)
#define DMA_CH_CONTROL1_CCS_MASK	(BIT(DMA_CH_CONTROL1_CCS_SHIFT))
#define DMA_CH_CONTROL1_LLE_MASK	(BIT(DMA_CH_CONTROL1_LLE_SHIFT))

/* DMA control 1 register Channel Status */
#define DMA_CH_CONTROL1_CS_RUNNING	(0x1 << DMA_CH_CONTROL1_CS_SHIFT)
#define DMA_CH_CONTROL1_CS_HALTED	(0x2 << DMA_CH_CONTROL1_CS_SHIFT)
#define DMA_CH_CONTROL1_CS_STOPPED	(0x3 << DMA_CH_CONTROL1_CS_SHIFT)

/* PCIe DMA Engine enable register definitions. */
#define DMA_ENGINE_EN_SHIFT		(0)
#define DMA_ENGINE_EN_MASK		(BIT(DMA_ENGINE_EN_SHIFT))

/* PCIe DMA interrupt registers definitions. */
#define DMA_ABORT_INTERRUPT_SHIFT	(16)
#define DMA_ABORT_INTERRUPT_MASK	(0xFF << DMA_ABORT_INTERRUPT_SHIFT)
#define DMA_ABORT_INTERRUPT_CH_MASK(_c) (BIT(_c) << DMA_ABORT_INTERRUPT_SHIFT)
#define DMA_DONE_INTERRUPT_MASK		(0xFF)
#define DMA_DONE_INTERRUPT_CH_MASK(_c)	(BIT(_c))
#define DMA_ALL_INTERRUPT_MASK \
	(DMA_ABORT_INTERRUPT_MASK | DMA_DONE_INTERRUPT_MASK)

#define DMA_LL_ERROR_SHIFT		(16)
#define DMA_CPL_ABORT_SHIFT		(8)
#define DMA_CPL_TIMEOUT_SHIFT		(16)
#define DMA_DATA_POI_SHIFT		(24)
#define DMA_AR_ERROR_CH_MASK(_c)	(BIT(_c))
#define DMA_LL_ERROR_CH_MASK(_c)	(BIT(_c) << DMA_LL_ERROR_SHIFT)
#define DMA_UNREQ_ERROR_CH_MASK(_c)	(BIT(_c))
#define DMA_CPL_ABORT_ERROR_CH_MASK(_c)	(BIT(_c) << DMA_CPL_ABORT_SHIFT)
#define DMA_CPL_TIMEOUT_ERROR_CH_MASK(_c) (BIT(_c) << DMA_CPL_TIMEOUT_SHIFT)
#define DMA_DATA_POI_ERROR_CH_MASK(_c)	(BIT(_c) << DMA_DATA_POI_SHIFT)

#define DMA_LLLAIE_SHIFT		(16)
#define DMA_LLLAIE_MASK			(0xF << DMA_LLLAIE_SHIFT)

#define DMA_CHAN_WRITE_MAX_WEIGHT	(0x1)
#define DMA_CHAN_READ_MAX_WEIGHT	(0x1)
#define DMA_CHAN0_WEIGHT_OFFSET		(0)
#define DMA_CHAN1_WEIGHT_OFFSET		(5)
#define DMA_CHAN2_WEIGHT_OFFSET		(10)
#define DMA_CHAN3_WEIGHT_OFFSET		(15)
#define DMA_CHAN_WRITE_ALL_MAX_WEIGHT					\
	((DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN0_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN1_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN2_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_WRITE_MAX_WEIGHT << DMA_CHAN3_WEIGHT_OFFSET))
#define DMA_CHAN_READ_ALL_MAX_WEIGHT					\
	((DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN0_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN1_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN2_WEIGHT_OFFSET) |	\
	 (DMA_CHAN_READ_MAX_WEIGHT << DMA_CHAN3_WEIGHT_OFFSET))

#define PCIE_REGS_PCIE_APP_CNTRL	0x8
#define APP_XFER_PENDING		BIT(6)
#define PCIE_REGS_PCIE_SII_PM_STATE_1	0xb4
#define PM_LINKST_IN_L1			BIT(10)

#define DMA_POLLING_TIMEOUT		1000000
#define DMA_PCIE_PM_L1_TIMEOUT		20
#define DMA_HRTIMER_PERIOD		(5000 * 1000)

struct __packed pcie_dma_reg {
	u32 dma_ctrl_data_arb_prior;
	u32 reserved1;
	u32 dma_ctrl;
	u32 dma_write_engine_en;
	u32 dma_write_doorbell;
	u32 reserved2;
	u32 dma_write_channel_arb_weight_low;
	u32 dma_write_channel_arb_weight_high;
	u32 reserved3[3];
	u32 dma_read_engine_en;
	u32 dma_read_doorbell;
	u32 reserved4;
	u32 dma_read_channel_arb_weight_low;
	u32 dma_read_channel_arb_weight_high;
	u32 reserved5[3];
	u32 dma_write_int_status;
	u32 reserved6;
	u32 dma_write_int_mask;
	u32 dma_write_int_clear;
	u32 dma_write_err_status;
	u32 dma_write_done_imwr_low;
	u32 dma_write_done_imwr_high;
	u32 dma_write_abort_imwr_low;
	u32 dma_write_abort_imwr_high;
	u16 dma_write_ch_imwr_data[8];
	u32 reserved7[4];
	u32 dma_write_linked_list_err_en;
	u32 reserved8[3];
	u32 dma_read_int_status;
	u32 reserved9;
	u32 dma_read_int_mask;
	u32 dma_read_int_clear;
	u32 reserved10;
	u32 dma_read_err_status_low;
	u32 dma_read_err_status_high;
	u32 dma_rd_err_sts_h;
	u32 reserved11[2];
	u32 dma_read_linked_list_err_en;
	u32 reserved12;
	u32 dma_read_done_imwr_low;
	u32 dma_read_done_imwr_high;
	u32 dma_read_abort_imwr_low;
	u32 dma_read_abort_imwr_high;
	u16 dma_read_ch_imwr_data[8];
};

struct __packed pcie_dma_chan {
	u32 dma_ch_control1;
	u32 reserved1;
	u32 dma_transfer_size;
	u32 dma_sar_low;
	u32 dma_sar_high;
	u32 dma_dar_low;
	u32 dma_dar_high;
	u32 dma_llp_low;
	u32 dma_llp_high;
};

enum xpcie_ep_engine_type {
	WRITE_ENGINE,
	READ_ENGINE
};

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
#define DMA_CHAN_NUM (8)
#else
#define DMA_CHAN_NUM (4)
#endif

static u32 dma_chan_offset[2][DMA_CHAN_NUM] = {
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	{ 0x200, 0x400, 0x600, 0x800, 0xA00, 0xC00, 0xE00, 0x1000 },
	{ 0x300, 0x500, 0x700, 0x900, 0xB00, 0xD00, 0xF00, 0x1100 }
#else
	{ 0x200, 0x400, 0x600, 0x800 },
	{ 0x300, 0x500, 0x700, 0x900 }
#endif
};

static void __iomem *intel_xpcie_ep_get_dma_base(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct xpcie_epf *xpcie_epf = (struct xpcie_epf *)dev->driver_data;

	return xpcie_epf->dbi_base + DMA_DBI_OFFSET;
}

static int intel_xpcie_ep_dma_disable(void __iomem *dma_base,
				      enum xpcie_ep_engine_type rw)
{
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)dma_base;
	void __iomem *int_mask, *int_clear;
	void __iomem *engine_en, *ll_err;
	int i;

	if (rw == WRITE_ENGINE) {
		engine_en = (void __iomem *)&dma_reg->dma_write_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_write_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_write_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_write_linked_list_err_en;
	} else {
		engine_en = (void __iomem *)&dma_reg->dma_read_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_read_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_read_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_read_linked_list_err_en;
	}

	iowrite32(0x0, engine_en);

	/* Mask all interrupts. */
	iowrite32(DMA_ALL_INTERRUPT_MASK, int_mask);

	/* Clear all interrupts. */
	iowrite32(DMA_ALL_INTERRUPT_MASK, int_clear);

	/* Disable LL abort interrupt (LLLAIE). */
	iowrite32(0, ll_err);

	/* Wait until the engine is disabled. */
	for (i = 0; i < DMA_POLLING_TIMEOUT; i++)
		if (!(ioread32(engine_en) & DMA_ENGINE_EN_MASK))
			return 0;


	return -EBUSY;
}

static void intel_xpcie_ep_dma_enable(void __iomem *dma_base,
				      enum xpcie_ep_engine_type rw)
{
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)(dma_base);
	void __iomem *engine_en, *ll_err, *arb_weight, *arb_weight_hi;
	struct __iomem pcie_dma_chan * dma_chan;
	void __iomem *int_mask, *int_clear;
	u32 offset, weight;
	int i;

	if (rw == WRITE_ENGINE) {
		engine_en = (void __iomem *)&dma_reg->dma_write_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_write_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_write_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_write_linked_list_err_en;
		arb_weight = (void __iomem *)
			     &dma_reg->dma_write_channel_arb_weight_low;
		arb_weight_hi = (void __iomem *)
				&dma_reg->dma_write_channel_arb_weight_high;
		weight = DMA_CHAN_WRITE_ALL_MAX_WEIGHT;
	} else {
		engine_en = (void __iomem *)&dma_reg->dma_read_engine_en;
		int_mask = (void __iomem *)&dma_reg->dma_read_int_mask;
		int_clear = (void __iomem *)&dma_reg->dma_read_int_clear;
		ll_err = (void __iomem *)&dma_reg->dma_read_linked_list_err_en;
		arb_weight = (void __iomem *)
			     &dma_reg->dma_read_channel_arb_weight_low;
		arb_weight_hi = (void __iomem *)
				&dma_reg->dma_read_channel_arb_weight_high;
		weight = DMA_CHAN_READ_ALL_MAX_WEIGHT;
	}

	iowrite32(DMA_ENGINE_EN_MASK, engine_en);

	/* Unmask all interrupts, so that the interrupt line gets asserted. */
	iowrite32(~(u32)DMA_ALL_INTERRUPT_MASK, int_mask);

	/* Clear all interrupts. */
	iowrite32(DMA_ALL_INTERRUPT_MASK, int_clear);

	/* Set channel round robin weight. */
	iowrite32(weight, arb_weight);
	iowrite32(weight, arb_weight_hi);

	/* Enable LL abort interrupt (LLLAIE). */
	iowrite32(DMA_LLLAIE_MASK, ll_err);

	/* Enable linked list mode. */
	for (i = 0; i < DMA_CHAN_NUM; i++) {
		offset = dma_chan_offset[rw][i];
		dma_chan = (struct __iomem pcie_dma_chan *)(dma_base + offset);
		iowrite32(DMA_CH_CONTROL1_LLE_MASK,
			  (void __iomem *)&dma_chan->dma_ch_control1);
	}
}

/*
 * Make sure EP is not in L1 state when DMA doorbell.
 * The DMA controller may start the wrong channel if doorbell occurs at the
 * same time as controller is transitioning to L1.
 */
static int intel_xpcie_ep_dma_doorbell(struct xpcie_epf *xpcie_epf, int chan,
				       void __iomem *doorbell)
{
	int rc = 0;
#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	int i = DMA_PCIE_PM_L1_TIMEOUT;
	u32 val, pm_val;

	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
	iowrite32(val | APP_XFER_PENDING,
		  xpcie_epf->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
	pm_val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_SII_PM_STATE_1);
	while (pm_val & PM_LINKST_IN_L1) {
		if (i-- < 0) {
			rc = -ETIME;
			break;
		}
		udelay(5);
		pm_val = ioread32(xpcie_epf->apb_base +
				  PCIE_REGS_PCIE_SII_PM_STATE_1);
	}

	iowrite32((u32)chan, doorbell);

	iowrite32(val & ~APP_XFER_PENDING,
		  xpcie_epf->apb_base + PCIE_REGS_PCIE_APP_CNTRL);
#else
	iowrite32((u32)chan, doorbell);
#endif

	return rc;
}

static int intel_xpcie_ep_dma_err_status(void __iomem *err_status, int chan)
{
	if (ioread32(err_status) &
	    (DMA_AR_ERROR_CH_MASK(chan) | DMA_LL_ERROR_CH_MASK(chan)))
		return -EIO;

	return 0;
}

static int intel_xpcie_ep_dma_rd_err_sts_h(void __iomem *err_status,
					   int chan)
{
	if (ioread32(err_status) &
	    (DMA_UNREQ_ERROR_CH_MASK(chan) |
	     DMA_CPL_ABORT_ERROR_CH_MASK(chan) |
	     DMA_CPL_TIMEOUT_ERROR_CH_MASK(chan) |
	     DMA_DATA_POI_ERROR_CH_MASK(chan)))
		return -EIO;

	return 0;
}

static void
intel_xpcie_ep_dma_setup_ll_descs(struct __iomem pcie_dma_chan * dma_chan,
				  struct xpcie_dma_ll_desc_buf *desc_buf,
				  int descs_num,
				  int func_no)
{
	struct xpcie_dma_ll_desc *descs = desc_buf->virt;
	int i;

	/* Setup linked list descriptors */
	for (i = 0; i < descs_num - 1; i++)
		descs[i].dma_ch_control1 = DMA_CH_CONTROL1_CB_MASK;
	descs[descs_num - 1].dma_ch_control1 = DMA_CH_CONTROL1_LIE_MASK |
						DMA_CH_CONTROL1_CB_MASK;
	descs[descs_num].dma_ch_control1 = DMA_CH_CONTROL1_LLP_MASK |
					   DMA_CH_CONTROL1_TCB_MASK;
	descs[descs_num].src_addr = (phys_addr_t)desc_buf->phys;

	/* Setup linked list settings */
	iowrite32(DMA_CH_CONTROL1_LLE_MASK | DMA_CH_CONTROL1_CCS_MASK |
		  (func_no << DMA_CH_CONTROL1_FUNC_NUM_SHIFT),
		  (void __iomem *)&dma_chan->dma_ch_control1);
	iowrite32((u32)desc_buf->phys, (void __iomem *)&dma_chan->dma_llp_low);
	iowrite32((u64)desc_buf->phys >> 32,
		  (void __iomem *)&dma_chan->dma_llp_high);
}

int intel_xpcie_ep_dma_write_ll(struct pci_epf *epf, int chan, int descs_num)
{
	unsigned long dma_time_period_us = DMA_HRTIMER_PERIOD;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct __iomem pcie_dma_chan * dma_chan;
	struct xpcie_dma_ll_desc_buf *desc_buf;
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)(dma_base);
	ktime_t free_tx_dma_tp;
	int rc;
#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	int i;
#endif

	if (descs_num <= 0 || descs_num > XPCIE_NUM_TX_DESCS)
		return -EINVAL;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	chan = xpcie_epf->epf->func_no;
#endif

	if (chan < 0 || chan >= DMA_CHAN_NUM)
		return -EINVAL;

	dma_chan = (struct __iomem pcie_dma_chan *)
		(dma_base + dma_chan_offset[WRITE_ENGINE][chan]);

	desc_buf = &xpcie_epf->tx_desc_buf;

	intel_xpcie_ep_dma_setup_ll_descs(dma_chan, desc_buf, descs_num, chan);

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	xpcie_epf->dma_wr_done = false;
#endif

	if (!hrtimer_active(&xpcie_epf->free_tx_dma_timer)) {
		free_tx_dma_tp = ktime_set(0, dma_time_period_us * 1000ULL);
		hrtimer_start(&xpcie_epf->free_tx_dma_timer, free_tx_dma_tp, HRTIMER_MODE_REL);
	}

	/* Start DMA transfer. */
	rc = intel_xpcie_ep_dma_doorbell(xpcie_epf, chan,
					 (void __iomem *)
					 &dma_reg->dma_write_doorbell);
	if (rc)
		return rc;

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	/* Wait for DMA transfer to complete. */
	for (i = 0; i < DMA_POLLING_TIMEOUT; i++) {
		usleep_range(5, 10);
		if (ioread32((void __iomem *)&dma_reg->dma_write_int_status) &
		    (DMA_DONE_INTERRUPT_CH_MASK(chan) |
		     DMA_ABORT_INTERRUPT_CH_MASK(chan)))
			break;
	}
	if (i == DMA_POLLING_TIMEOUT) {
		dev_err(&xpcie_epf->epf->dev, "DMA Wr timeout\n");
		rc = -ETIME;
		goto cleanup;
	}
#else
	/* Wait for DMA transfer to complete. */
	rc = wait_event_interruptible(xpcie_epf->dma_wr_wq,
				      xpcie_epf->dma_wr_done);
	if (atomic_read(&xpcie_epf->dma_wr_eng_reset_cnt)) {
		atomic_set(&xpcie_epf->dma_wr_eng_reset_cnt, 0);
		dev_info(&xpcie_epf->epf->dev,
			 "[TX_DMA_TIMER_CB] dma write engine reset\n");
		if (xpcie_epf->dma_wr_rc & 0xffff)
			dev_info(&xpcie_epf->epf->dev,
				 "[TX_DMA_TIMER_CB]: dma err -EIO\n");
		if (xpcie_epf->dma_wr_rc & 0xffff0000)
			dev_info(&xpcie_epf->epf->dev,
				 "[TX_DMA_TIMER_CB]: dma reset -EBUSY\n");

		dev_info(&xpcie_epf->epf->dev,
			 "[TX_DMA_TIMER_CB] dma_write_int_status: 0x%x\n",
			 xpcie_epf->dma_wr_int_status);
		dev_info(&xpcie_epf->epf->dev, "dma_write_err_status: 0x%x\n",
			 xpcie_epf->dma_wr_err_status);
	}
#endif

	rc = intel_xpcie_ep_dma_err_status((void __iomem *)
					   &dma_reg->dma_write_err_status,
					   chan);

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
cleanup:
	/* Clear the done/abort interrupt. */
	iowrite32((DMA_DONE_INTERRUPT_CH_MASK(chan) |
		   DMA_ABORT_INTERRUPT_CH_MASK(chan)),
		  (void __iomem *)&dma_reg->dma_write_int_clear);
#endif

	if (rc) {
		if (intel_xpcie_ep_dma_disable(dma_base, WRITE_ENGINE)) {
			dev_err(&xpcie_epf->epf->dev,
				"failed to disable WR DMA\n");
			return rc;
		}
		intel_xpcie_ep_dma_enable(dma_base, WRITE_ENGINE);
	}

	return rc;
}

int intel_xpcie_ep_dma_read_ll(struct pci_epf *epf, int chan, int descs_num)
{
	unsigned long dma_time_period_us = DMA_HRTIMER_PERIOD;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct xpcie_dma_ll_desc_buf *desc_buf;
	struct __iomem pcie_dma_reg * dma_reg =
				(struct __iomem pcie_dma_reg *)(dma_base);
	struct __iomem pcie_dma_chan * dma_chan;
	ktime_t free_rx_dma_tp;
	int rc;
#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	int i;
#endif

	if (descs_num <= 0 || descs_num > XPCIE_NUM_RX_DESCS)
		return -EINVAL;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	chan = xpcie_epf->epf->func_no;
#endif

	if (chan < 0 || chan >= DMA_CHAN_NUM)
		return -EINVAL;

	dma_chan = (struct __iomem pcie_dma_chan *)
		(dma_base + dma_chan_offset[READ_ENGINE][chan]);

	desc_buf = &xpcie_epf->rx_desc_buf;

	intel_xpcie_ep_dma_setup_ll_descs(dma_chan, desc_buf, descs_num, chan);

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	xpcie_epf->dma_rd_done = false;
#endif

	if (!hrtimer_active(&xpcie_epf->free_rx_dma_timer)) {
		free_rx_dma_tp = ktime_set(0, dma_time_period_us * 1000ULL);
		hrtimer_start(&xpcie_epf->free_rx_dma_timer, free_rx_dma_tp, HRTIMER_MODE_REL);
	}

	/* Start DMA transfer. */
	rc = intel_xpcie_ep_dma_doorbell(xpcie_epf, chan,
					 (void __iomem *)
					 &dma_reg->dma_read_doorbell);
	if (rc)
		return rc;

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	/* Wait for DMA transfer to complete. */
	for (i = 0; i < DMA_POLLING_TIMEOUT; i++) {
		usleep_range(5, 10);
		if (ioread32((void __iomem *)&dma_reg->dma_read_int_status) &
		    (DMA_DONE_INTERRUPT_CH_MASK(chan) |
		     DMA_ABORT_INTERRUPT_CH_MASK(chan)))
			break;
	}
	if (i == DMA_POLLING_TIMEOUT) {
		dev_err(&xpcie_epf->epf->dev, "DMA Rd timeout\n");
		rc = -ETIME;
		goto cleanup;
	}
#else
	/* Wait for DMA transfer to complete. */
	rc = wait_event_interruptible(xpcie_epf->dma_rd_wq,
				      xpcie_epf->dma_rd_done);
	if (atomic_read(&xpcie_epf->dma_rd_eng_reset_cnt)) {
		atomic_set(&xpcie_epf->dma_rd_eng_reset_cnt, 0);
		dev_info(&xpcie_epf->epf->dev,
			 "[RX_DMA_TIMER_CB] dma read engine reset\n");
		if (xpcie_epf->dma_rd_rc)
			dev_info(&xpcie_epf->epf->dev,
				 "[RX_DMA_TIMER_CB]: dma reset -EBUSY\n");

		dev_info(&xpcie_epf->epf->dev,
			 "[RX_DMA_TIMER_CB] dma_read_int_status: 0x%x\n",
			 xpcie_epf->dma_rd_int_status);
		dev_info(&xpcie_epf->epf->dev,
			 "[RX_DMA_TIMER_CB] dma_read_err_status_low: 0x%x\n",
			 xpcie_epf->dma_rd_err_status_low);
		dev_info(&xpcie_epf->epf->dev,
			 "[RX_DMA_TIMER_CB] dma_read_err_status_high: 0x%x\n",
			 xpcie_epf->dma_rd_err_status_high);
	}
#endif

	rc = intel_xpcie_ep_dma_err_status((void __iomem *)
					   &dma_reg->dma_read_err_status_low,
					   chan);
	if (!rc) {
		rc =
		intel_xpcie_ep_dma_rd_err_sts_h((void __iomem *)
						&dma_reg->dma_rd_err_sts_h,
						chan);
	}

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
cleanup:
	/* Clear the done/abort interrupt. */
	iowrite32((DMA_DONE_INTERRUPT_CH_MASK(chan) |
		   DMA_ABORT_INTERRUPT_CH_MASK(chan)),
		  (void __iomem *)&dma_reg->dma_read_int_clear);
#endif

	if (rc) {
		if (intel_xpcie_ep_dma_disable(dma_base, READ_ENGINE)) {
			dev_err(&xpcie_epf->epf->dev,
				"failed to disable RD DMA\n");
			return rc;
		}
		intel_xpcie_ep_dma_enable(dma_base, READ_ENGINE);
	}

	return rc;
}

static void intel_xpcie_ep_dma_free_ll_descs_mem(struct xpcie_epf *xpcie_epf)
{
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	struct device *dma_dev = &xpcie_epf->epf->dev;
#else
	struct device *dma_dev = xpcie_epf->epf->epc->dev.parent;
#endif

	if (xpcie_epf->tx_desc_buf.virt) {
		dma_free_coherent(dma_dev,
				  xpcie_epf->tx_desc_buf.size,
				  xpcie_epf->tx_desc_buf.virt,
				  xpcie_epf->tx_desc_buf.phys);
	}
	if (xpcie_epf->rx_desc_buf.virt) {
		dma_free_coherent(dma_dev,
				  xpcie_epf->rx_desc_buf.size,
				  xpcie_epf->rx_desc_buf.virt,
				  xpcie_epf->rx_desc_buf.phys);
	}

	memset(&xpcie_epf->tx_desc_buf, 0,
	       sizeof(struct xpcie_dma_ll_desc_buf));
	memset(&xpcie_epf->rx_desc_buf, 0,
	       sizeof(struct xpcie_dma_ll_desc_buf));
}

static int intel_xpcie_ep_dma_alloc_ll_descs_mem(struct xpcie_epf *xpcie_epf)
{
	int tx_num = XPCIE_NUM_TX_DESCS + 1;
	int rx_num = XPCIE_NUM_RX_DESCS + 1;
	size_t tx_size, rx_size;
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	struct device *dma_dev = &xpcie_epf->epf->dev;
#else
	struct device *dma_dev = xpcie_epf->epf->epc->dev.parent;
#endif
	tx_size = tx_num * sizeof(struct xpcie_dma_ll_desc);
	rx_size = rx_num * sizeof(struct xpcie_dma_ll_desc);

	xpcie_epf->tx_desc_buf.virt =
			dma_alloc_coherent(dma_dev, tx_size,
					   &xpcie_epf->tx_desc_buf.phys,
					   GFP_KERNEL);
	xpcie_epf->rx_desc_buf.virt =
			dma_alloc_coherent(dma_dev, rx_size,
					   &xpcie_epf->rx_desc_buf.phys,
					   GFP_KERNEL);
	if (!xpcie_epf->tx_desc_buf.virt ||
	    !xpcie_epf->rx_desc_buf.virt) {
		intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);
		return -ENOMEM;
	}

	xpcie_epf->tx_desc_buf.size = tx_size;
	xpcie_epf->rx_desc_buf.size = rx_size;

	return 0;
}

static int intel_xpcie_ep_dma_reset_read_engine(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Disable the DMA read engine. */
	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, READ_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, READ_ENGINE);

	return 0;
}

static int intel_xpcie_ep_dma_reset_write_engine(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Disable the DMA write engine. */
	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, WRITE_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, WRITE_ENGINE);

	return 0;
}

void intel_xpcie_ep_stop_dma(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
	int chan;

	chan = xpcie_epf->epf->func_no;
	chan = chan | (1 << 31);
	iowrite32((u32)chan, &dma_reg->dma_write_doorbell);
	iowrite32((u32)chan, &dma_reg->dma_read_doorbell);

	intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);
}

void intel_xpcie_ep_start_dma(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	intel_xpcie_ep_dma_alloc_ll_descs_mem(xpcie_epf);
}

int intel_xpcie_ep_dma_reset(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Disable the DMA read/write engine. */
	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, WRITE_ENGINE) ||
	    intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, READ_ENGINE))
		return -EBUSY;

	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, WRITE_ENGINE);
	intel_xpcie_ep_dma_enable(xpcie_epf->dma_base, READ_ENGINE);

	return 0;
}

static irqreturn_t intel_xpcie_ep_dma_wr_interrupt(int irq, void *args)
{
	struct xpcie_epf *xpcie_epf = args;
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	int chan = xpcie_epf->epf->func_no;

	if (ioread32(&dma_reg->dma_write_int_status) &
		(DMA_ABORT_INTERRUPT_CH_MASK(chan) |
		 DMA_DONE_INTERRUPT_CH_MASK(chan))) {
		iowrite32(DMA_DONE_INTERRUPT_CH_MASK(chan),
			  &dma_reg->dma_write_int_clear);

		if (hrtimer_active(&xpcie_epf->free_tx_dma_timer))
			hrtimer_cancel(&xpcie_epf->free_tx_dma_timer);
		xpcie_epf->dma_wr_done = true;
		wake_up_interruptible(&xpcie_epf->dma_wr_wq);
	}
#endif
	return IRQ_HANDLED;
}

static irqreturn_t intel_xpcie_ep_dma_rd_interrupt(int irq, void *args)
{
	struct xpcie_epf *xpcie_epf = args;
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	int chan = xpcie_epf->epf->func_no;

	if (ioread32(&dma_reg->dma_read_int_status) &
		(DMA_ABORT_INTERRUPT_CH_MASK(chan) |
		 DMA_DONE_INTERRUPT_CH_MASK(chan))) {
		iowrite32(DMA_DONE_INTERRUPT_CH_MASK(chan),
			  &dma_reg->dma_read_int_clear);

		if (hrtimer_active(&xpcie_epf->free_rx_dma_timer))
			hrtimer_cancel(&xpcie_epf->free_rx_dma_timer);
		xpcie_epf->dma_rd_done = true;
		wake_up_interruptible(&xpcie_epf->dma_rd_wq);
	}
#endif
	return IRQ_HANDLED;
}

static enum hrtimer_restart
free_rx_dma_timer_cb(struct hrtimer *free_rx_dma_timer)
{
	struct xpcie_epf *xpcie_epf =
		container_of(free_rx_dma_timer,
			     struct xpcie_epf, free_rx_dma_timer);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
	void __iomem *err_status;

	atomic_inc(&xpcie_epf->dma_rd_eng_reset_cnt);

	err_status = &dma_reg->dma_read_int_status;
	xpcie_epf->dma_rd_int_status = ioread32(err_status);

	err_status = &dma_reg->dma_read_err_status_low;
	xpcie_epf->dma_rd_err_status_low = ioread32(err_status);
	err_status = &dma_reg->dma_read_err_status_high;
	xpcie_epf->dma_rd_err_status_high = ioread32(err_status);

	/* RESET DMA READ_ENGINE */
	xpcie_epf->dma_rd_rc =
			intel_xpcie_ep_dma_reset_read_engine(xpcie_epf->epf);
	xpcie_epf->dma_rd_done = true;
	wake_up_interruptible(&xpcie_epf->dma_rd_wq);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart
free_tx_dma_timer_cb(struct hrtimer *free_tx_dma_timer)
{
	struct xpcie_epf *xpcie_epf =
		container_of(free_tx_dma_timer,
			     struct xpcie_epf, free_tx_dma_timer);
	void __iomem *dma_base = xpcie_epf->dma_base;
	struct pcie_dma_reg *dma_reg = (struct pcie_dma_reg *)dma_base;
	int func_no = xpcie_epf->epf->func_no;
	void __iomem *err_status;
	int ret1 = 0, ret2 = 0;

	atomic_inc(&xpcie_epf->dma_wr_eng_reset_cnt);

	err_status = &dma_reg->dma_write_int_status;
	xpcie_epf->dma_wr_int_status = ioread32(err_status);

	ret1 = intel_xpcie_ep_dma_err_status(&dma_reg->dma_write_err_status, func_no);

	err_status = &dma_reg->dma_write_err_status;
	xpcie_epf->dma_wr_err_status = ioread32(err_status);

	/* RESET DMA WRITE_ENGINE */
	ret2 = intel_xpcie_ep_dma_reset_write_engine(xpcie_epf->epf);

	xpcie_epf->dma_wr_rc = ret1 | (ret2 << 16);

	xpcie_epf->dma_wr_done = true;
	wake_up_interruptible(&xpcie_epf->dma_wr_wq);

	return HRTIMER_NORESTART;
}

int intel_xpcie_ep_dma_uninit(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	hrtimer_cancel(&xpcie_epf->free_tx_dma_timer);
	hrtimer_cancel(&xpcie_epf->free_rx_dma_timer);

	if (intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, WRITE_ENGINE) ||
	    intel_xpcie_ep_dma_disable(xpcie_epf->dma_base, READ_ENGINE))
		return -EBUSY;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	if (xpcie_epf->irq_rdma)
		free_irq(xpcie_epf->irq_rdma, xpcie_epf);
	if (xpcie_epf->irq_wdma)
		free_irq(xpcie_epf->irq_wdma, xpcie_epf);
#endif

	intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);

	return 0;
}

int intel_xpcie_ep_dma_init(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	int rc;

	xpcie_epf->dma_base = intel_xpcie_ep_get_dma_base(epf);

	rc = intel_xpcie_ep_dma_alloc_ll_descs_mem(xpcie_epf);
	if (rc)
		return rc;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	init_waitqueue_head(&xpcie_epf->dma_rd_wq);
	init_waitqueue_head(&xpcie_epf->dma_wr_wq);

	rc = request_irq(xpcie_epf->irq_rdma, &intel_xpcie_ep_dma_rd_interrupt,
			 0, "xpcie_epf_rd_dma", xpcie_epf);
	if (rc) {
		pr_err("intel_xpcie_ep_dma: Failed to request read intr\n");
		intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);
		return rc;
	}

	rc = request_irq(xpcie_epf->irq_wdma, &intel_xpcie_ep_dma_wr_interrupt,
			 0, "xpcie_epf_wr_dma", xpcie_epf);
	if (rc) {
		pr_err("intel_xpcie_ep_dma: Failed to request write intr\n");
		free_irq(xpcie_epf->irq_rdma, xpcie_epf);
		intel_xpcie_ep_dma_free_ll_descs_mem(xpcie_epf);
		return rc;
	}
#endif

	hrtimer_init(&xpcie_epf->free_tx_dma_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	xpcie_epf->free_tx_dma_timer.function = free_tx_dma_timer_cb;

	hrtimer_init(&xpcie_epf->free_rx_dma_timer,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	xpcie_epf->free_rx_dma_timer.function = free_rx_dma_timer_cb;

	return intel_xpcie_ep_dma_reset(epf);
}
