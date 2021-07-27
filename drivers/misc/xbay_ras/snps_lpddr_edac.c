// SPDX-License-Identifier: GPL-2.0 only
/*
 * Intel xBay RAS: Synopsys DDR ECC SW
 *
 * Copyright (C) 2021 Intel Corporation
 *
 * Code referred from:
 * drivers/edac/synopsys_edac.c
 */

#include "ras_core.h"

/* Register Offsets */
#define ECC_CFG0_OFF		0x00
#define ECC_CFG1_OFF		0x04
#define ECC_STAT_OFF		0x08
#define ECC_CTRL_OFF		0x0C
#define ECC_ERRCNT_OFF		0x10
#define ECC_CE_ADDR0_OFF	0x14
#define ECC_CE_ADDR1_OFF	0x18
#define ECC_BITMASK0_OFF	0x28
#define ECC_BITMASK1_OFF	0x2C
#define ECC_BITMASK2_OFF	0x30
#define ECC_POISON0_OFF		0x48
#define ECC_POISON1_OFF		0x4C

/* Status Register Bit definitions */
#define ECC_STAT_UE_CNT_MASK	GENMASK(19, 16)
#define ECC_STAT_CE_CNT_MASK	GENMASK(11, 8)
#define ECC_STAT_BIT_NUM_MASK	GENMASK(6, 0)
#define ECC_STAT_UE_CNT_SHIFT	16
#define ECC_STAT_CE_CNT_SHIFT	8

/* Control Register Bit definitions */
#define ECC_CTRL_CLR_CE		BIT(0)
#define ECC_CTRL_CLR_UE		BIT(1)
#define ECC_CTRL_CLR_CE_CNT	BIT(2)
#define ECC_CTRL_CLR_UE_CNT	BIT(3)
#define ECC_CTRL_CLR_INTR	BIT(4)
#define ECC_CTRL_CLR_MASK	GENMASK(4, 0)

/* CE/UE Info register definitions */
#define ECC_CE_INFO_OFF		ECC_CE_ADDR0_OFF
#define ECC_UE_INFO_OFF		(ECC_CE_INFO_OFF + 0x20)

#define ADDR0_OFF		0x00
#define ADDR1_OFF		0x04
#define SYN0_OFF		0x08
#define SYN1_OFF		0x0C
#define SYN2_OFF		0x10

#define ADDR0_ROW_MASK		GENMASK(17, 0)
#define ADDR0_RANK_MASK		BIT(24)
#define ADDR1_BANK_GRP_MASK	GENMASK(25, 24)
#define ADDR1_BANK_NUM_MASK	GENMASK(18, 16)
#define ADDR1_BLK_NUM_MASK	GENMASK(11, 0)
#define ADDR1_BANK_GRP_SHIFT	24
#define ADDR1_BANK_NUM_SHIFT	16

static struct ddr_edac_status_t
	ddr_edac_status[THB_DDR_SLICE_NUM][THB_DDR_MCS_PER_SLICE];
static void __iomem
	*ddr_edac_base[THB_DDR_SLICE_NUM][THB_DDR_MCS_PER_SLICE];
static int ddr_edac_irq[THB_DDR_SLICE_NUM][THB_DDR_MCS_PER_SLICE];
static u32 num_slices, num_mcs_per_slice;

static int ddr_edac_clear_err(struct platform_device *pdev,
			      int slice_id, int mc_id)
{
	void __iomem *base;

	if (slice_id >= num_slices || mc_id >= num_mcs_per_slice) {
		dev_err(&pdev->dev, "Invalid DDR ECC Slice %d MC %d\n",
			slice_id, mc_id);
		return -EINVAL;
	}

	base = ddr_edac_base[slice_id][mc_id];
	writel(ECC_CTRL_CLR_MASK, base + ECC_CTRL_OFF);

	return 0;
}

#if defined(DEBUG_RAS_DDR) && defined(DEBUG_DDR_HW_INJ)
static void ddr_edac_print_info(struct platform_device *pdev,
				int slice_id, int mc_id)
{
	struct ddr_edac_status_t *e_sts;
	struct ddr_edac_err_info_t *e_info = NULL;

	if (slice_id >= num_slices || mc_id >= num_mcs_per_slice) {
		dev_err(&pdev->dev, "Invalid DDR ECC Slice %d MC %d\n",
			slice_id, mc_id);
		return;
	}

	e_sts = &ddr_edac_status[slice_id][mc_id];

	dev_info(&pdev->dev, "DDR ECC Slice %d MC %d Statistics :\n",
		 slice_id, mc_id);
	dev_info(&pdev->dev, "CE: %d UE: %d\n", e_sts->ce_cnt, e_sts->ue_cnt);
	dev_info(&pdev->dev, "CE Statistics :\n");
	e_info = &e_sts->ce_info;
	dev_info(&pdev->dev, "BitPos %d\n", e_info->bit_pos);
	dev_info(&pdev->dev, "Row %d Bank %d Bank Group %d Block %d\n",
		 e_info->row, e_info->bank,
		 e_info->bank_grp_num, e_info->blk_num);
	dev_info(&pdev->dev, "Data Pattern: %08x\n", e_info->data_pattern);
	dev_info(&pdev->dev, "UE Statistics :\n");
	e_info = &e_sts->ue_info;
	dev_info(&pdev->dev, "Row %d Bank %d Bank Group %d Block %d\n",
		 e_info->row, e_info->bank,
		 e_info->bank_grp_num, e_info->blk_num);
	dev_info(&pdev->dev, "Data Pattern: %08x\n", e_info->data_pattern);
}
#endif

static int ddr_edac_get_err_info(struct platform_device *pdev,
				 int slice_id, int mc_id)
{
	void __iomem *base, *e_base = NULL;
	struct ddr_edac_status_t *e_sts;
	struct ddr_edac_err_info_t *e_info = NULL;
	u32 val;
	enum error_type err;

	if (slice_id >= num_slices || mc_id >= num_mcs_per_slice) {
		dev_err(&pdev->dev, "Invalid DDR ECC Slice %d MC %d\n",
			slice_id, mc_id);
		return -EINVAL;
	}

	base = ddr_edac_base[slice_id][mc_id];
	e_sts = &ddr_edac_status[slice_id][mc_id];

	/* Read ECC STAT */
	val = readl(base + ECC_STAT_OFF);
	e_sts->ce_cnt = (val & ECC_STAT_CE_CNT_MASK) >> ECC_STAT_CE_CNT_SHIFT;
	e_sts->ue_cnt = (val & ECC_STAT_UE_CNT_MASK) >> ECC_STAT_UE_CNT_SHIFT;
	if (!e_sts->ce_cnt && !e_sts->ue_cnt) {
		dev_err(&pdev->dev,
			"No DDR ECC CE/UE detected on Slice %d MC %d\n",
			slice_id, mc_id);
		return NO_ERR;
	}

	if (e_sts->ce_cnt) {
		err = CORR_ERR;
		e_base = base + ECC_CE_INFO_OFF;
		e_info = &e_sts->ce_info;
		e_info->bit_pos = val & ECC_STAT_BIT_NUM_MASK;
	} else {
		err = UNCORR_ERR;
		e_base = base + ECC_UE_INFO_OFF;
		e_info = &e_sts->ue_info;
	}

	/* Read ADDR0 */
	val = readl(e_base + ADDR0_OFF);
	e_info->row = (val & ADDR0_ROW_MASK);

	/* Read ADDR1 */
	val = readl(e_base + ADDR1_OFF);
	e_info->bank = (val & ADDR1_BANK_NUM_MASK) >> ADDR1_BANK_NUM_SHIFT;
	e_info->bank_grp_num = (val & ADDR1_BANK_GRP_MASK) >>
				ADDR1_BANK_GRP_SHIFT;
	e_info->blk_num = (val & ADDR1_BLK_NUM_MASK);

	/* Read SYN0 */
	e_info->data_pattern = readl(e_base + SYN0_OFF);

	return err;
}

static u8 get_slice_mc_id(int irq)
{
	int i, j = 0;

	for (i = 0; i < num_slices; i++) {
		for (j = 0; j < num_mcs_per_slice; j++) {
			if (ddr_edac_irq[i][j] == irq)
				return  (i << 4) | j;
		}
	}
	return  (i << 4) | j;
}

static irqreturn_t ddr_edac_handle_irq(int irq, void *data)
{
	u8 id;
	enum error_type err;
	struct platform_device *pdev = (struct platform_device *)data;

	id = get_slice_mc_id(irq);

	err = ddr_edac_get_err_info(pdev, id >> 4, id & 0xF);
	if (err < 0)
		return IRQ_HANDLED;

	ddr_edac_clear_err(pdev, id >> 4, id & 0xF);
	xbay_ras_clear_interrupt(irq);
	xbay_pcie_report_internal_error(err);

	return IRQ_HANDLED;
}

#if defined(DEBUG_RAS_DDR) && defined(DEBUG_DDR_HW_INJ)
static ssize_t tst_ddr_edac_poison_addr_show(struct device *dev,
					     struct device_attribute *mattr,
					     char *data)
{
	struct platform_device *pdev = to_pdev(dev);

	dev_info(&pdev->dev, "%s %d\n", __func__, __LINE__);

	return 0;
}

static ssize_t tst_ddr_edac_poison_addr_store(struct device *dev,
					      struct device_attribute *mattr,
					      const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);
	ulong poison_addr;

	if (kstrtoul(data, 0, &poison_addr))
		return -EINVAL;

	dev_info(&pdev->dev, "Poisoning DDR Address %lu\n", poison_addr);

	return count;
}

static ssize_t tst_ddr_edac_inj_error_store(struct device *dev,
					    struct device_attribute *mattr,
					    const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);

	if (strncmp(data, "CE", 2) == 0)
		dev_info(&pdev->dev, "Injecting CE on DDR\n");
	else
		dev_info(&pdev->dev, "Injecting UE on DDR\n");

	return count;
}

static DEVICE_ATTR_RW(tst_ddr_edac_poison_addr);
static DEVICE_ATTR_WO(tst_ddr_edac_inj_error);
#endif

#if defined(DEBUG_RAS_DDR) && defined(DEBUG_DDR_SW_INJ)
static ssize_t tst_ddr_edac_sw_inj_store(struct device *dev,
					 struct device_attribute *mattr,
					 const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);
	struct ddr_edac_status_t *e_sts = NULL;
	struct ddr_edac_err_info_t *e_info = NULL;
	int slice_id, mc_id;
	bool ce = false;

	if (strncmp(data, "CE_0_0", 6) == 0) {
		ce = true; slice_id = 0; mc_id = 0;
	} else if (strncmp(data, "UE_2_0", 6) == 0) {
		ce = false; slice_id = 2; mc_id = 0;
	} else if (strncmp(data, "CE_1_1", 6) == 0) {
		ce = true; slice_id = 1; mc_id = 1;
	} else if (strncmp(data, "UE_3_1", 6) == 0) {
		ce = false; slice_id = 3; mc_id = 1;
	} else {
		dev_err(&pdev->dev, "Invalid options chosen\n");
		dev_err(&pdev->dev,
			"CE_0_0, UE_2_0, CE_1_1, UE_3_1 are available\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "Injecting SW Error on DDR Slice%d MC%d\n",
		 slice_id, mc_id);
	e_sts = &ddr_edac_status[slice_id][mc_id];
	e_info = &e_sts->ce_info;

	e_info->row = 0x04 + mc_id;
	e_info->col = 0x05 + mc_id;
	e_info->bank = 0x06 + mc_id;
	e_info->bit_pos = 0x07 + mc_id;
	e_info->data_pattern = 0xa5a5a5 + mc_id;
	e_info->bank_grp_num = 0x08 + mc_id;
	e_info->blk_num = 0x09 + mc_id;
	if (ce)
		e_sts->ce_cnt = 0x02 + mc_id;
	else
		e_sts->ue_cnt = 0x03 + mc_id;

	return count;
}
static DEVICE_ATTR_WO(tst_ddr_edac_sw_inj);
#endif

#ifdef DEBUG_RAS_DDR
static const struct attribute *ddr_edac_inject_attrs[] = {
#ifdef DEBUG_DDR_HW_INJ
	&dev_attr_tst_ddr_edac_poison_addr.attr,
	&dev_attr_tst_ddr_edac_inj_error.attr,
#endif
#ifdef DEBUG_DDR_SW_INJ
	&dev_attr_tst_ddr_edac_sw_inj.attr,
#endif
	NULL,
};
#endif

#ifdef DEBUG_RAS_DDR
static const struct attribute_group ddr_edac_attributes = {
	.attrs = (struct attribute **)ddr_edac_inject_attrs,
};
#endif

void xbay_ddr_edac_get_all_info(struct platform_device *pdev,
				char *buf, int count)
{
	memcpy(buf, (char *)ddr_edac_status, count);
}

void xbay_ddr_edac_clear_all_info(struct platform_device *pdev, int count)
{
	memset((char *)ddr_edac_status, 0, count);
}

int xbay_ddr_edac_probe(struct platform_device *pdev, struct device_node *np)
{
	int i, j, err;
	char slice_name[20];

	err = of_property_read_u32(np, "num-slices", &num_slices);
	if (err) {
		dev_err(&pdev->dev, "missing num-slices property\n");
		return err;
	}

	if (num_slices > THB_DDR_SLICE_NUM) {
		dev_err(&pdev->dev, "Expected DDR Slices %d, but got %d\n",
			THB_DDR_SLICE_NUM,
			num_slices);
		return -EINVAL;
	}

	err = of_property_read_u32(np, "num-mcs-per-slice",
				   &num_mcs_per_slice);
	if (err) {
		dev_err(&pdev->dev, "missing num-mcs-per-slice property\n");
		return err;
	}

	if (num_mcs_per_slice > THB_DDR_MCS_PER_SLICE) {
		dev_err(&pdev->dev, "Expected MCs per Slice %d, but got %d\n",
			THB_DDR_MCS_PER_SLICE,
			num_mcs_per_slice);
		return -EINVAL;
	}

	for (i = 0; i < num_slices; i++) {
		for (j = 0; j < num_mcs_per_slice; j++) {
			sprintf(slice_name, "slice%d-mc%d-edac", i, j);

			/* Map DDR ECC Registers */
			ddr_edac_base[i][j] =
			    devm_platform_ioremap_resource_byname(pdev,
								  slice_name);
			if (IS_ERR(ddr_edac_base[i][j])) {
				dev_err(&pdev->dev,
					"Failed to Map DDR ECC slice%d mc%d\n",
					i, j);
				return PTR_ERR(ddr_edac_base[i][j]);
			}

			/* Get IRQ */
			ddr_edac_irq[i][j] = of_irq_get_byname(np, slice_name);
			if (ddr_edac_irq[i][j] < 0) {
				dev_err(&pdev->dev,
					"Failed to get IRQ slice%d mc%d\n",
					i, j);
				return ddr_edac_irq[i][j];
			}

			/* Request IRQ */
			err = devm_request_threaded_irq(&pdev->dev,
							ddr_edac_irq[i][j],
							ddr_edac_handle_irq,
							NULL,
							IRQF_SHARED,
							slice_name,
							pdev);
			if (err) {
				dev_err(&pdev->dev,
					"Failed to request IRQ %d\n",
					ddr_edac_irq[i][j]);
				return err;
			}
		}
	}

#ifdef DEBUG_RAS_DDR
	err = sysfs_create_group(&pdev->dev.kobj, &ddr_edac_attributes);
	if (err < 0) {
		dev_err(&pdev->dev,
			"Failed to create DDR EDAC sysfs entries\n");
		return err;
	}
#endif

	dev_info(&pdev->dev, "DDR EDAC module probed Successfully\n");

	return 0;
}
