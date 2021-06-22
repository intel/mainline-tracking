// SPDX-License-Identifier: GPL-2.0 only
/*
 * Intel xBay RAS: Synopsys CSRAM ECC SW
 *
 * Copyright (C) 2021 Intel Corporation
 */

#include "ras_core.h"

#define THB_CSRAM_IRQ_NUM	2

/* Register Offsets */
#define ERR_STATUS	0x00
#define ERR_MASK	0x08
#define ERR_SVRTY	0x10
#define ERR_ACSRAMS_LOG	0x18
#define ECC_INFO	0x20
#define ERR_INJ		0x28

#define ERR_STATUS_ERR_MASK	GENMASK(12, 0)

#define ECC_INFO_SBIT_MASK	GENMASK(15, 0)
#define ECC_INFO_DBIT_MASK	GENMASK(31, 16)
#define ECC_INFO_DBIT_SHIFT	16

#define ERR_INJ_PARITY	BIT(11)
#define ERR_INJ_DEC_BG	BIT(7)
#define ERR_INJ_DEC_FG	BIT(6)
#define ERR_INJ_BG	BIT(5)
#define ERR_INJ_FG	BIT(4)

enum {
	NON_FATAL,
	FATAL,
};

static struct csram_edac_status_t
	csram_edac_status[THB_CSRAM_BANK_NUM];
static void __iomem
	*csram_edac_base[THB_CSRAM_BANK_NUM];
static int csram_edac_irq[THB_CSRAM_IRQ_NUM];
static u32 num_banks;

#if defined(DEBUG_RAS_CSRAM) && defined(DEBUG_CSRAM_HW_INJ)
static void csram_edac_print_info(struct platform_device *pdev)
{
	struct csram_edac_status_t *e_sts;
	int bank_num;

	for (bank_num = 0; bank_num < num_banks; bank_num++) {
		e_sts = &csram_edac_status[bank_num];

		dev_dbg(&pdev->dev, "CSRAM Bank %d Statistics :\n", bank_num);
		dev_dbg(&pdev->dev, "SBIT: %d DBIT: %d\n",
			e_sts->sbit_cnt, e_sts->dbit_cnt);
		dev_dbg(&pdev->dev, "Error @Acsramess: %llx\n",
			e_sts->err_acsramess);
		dev_dbg(&pdev->dev, "Error status: %08x\n",
			e_sts->err_status);
	}
}
#endif

static int csram_edac_get_err_info(struct platform_device *pdev, int irq)
{
	void __iomem *base = NULL;
	struct csram_edac_status_t *e_sts;
	u64 val;
	int bank_num;
	enum error_type err = NO_ERR;

	if (csram_edac_irq[FATAL] == irq)
		err = UNCORR_ERR;
	else
		err = CORR_ERR;

	for (bank_num = 0; bank_num < num_banks; bank_num++) {
		base = csram_edac_base[bank_num];
		e_sts = &csram_edac_status[bank_num];

		/* Read ECC Info */
		val = readl(base + ECC_INFO);
		e_sts->sbit_cnt = val & ECC_INFO_SBIT_MASK;
		e_sts->dbit_cnt = val &
				  (ECC_INFO_DBIT_MASK >> ECC_INFO_DBIT_SHIFT);

		/* Read ACSRAM log */
		e_sts->err_acsramess = readl(base + ERR_ACSRAMS_LOG);

		/* Read error status */
		val = readl(base + ERR_STATUS);
		e_sts->err_status = val & ERR_STATUS_ERR_MASK;

		/* Clear Error */
		if (val)
			writel(val, base + ERR_STATUS);
	}

	return err;
}

static irqreturn_t csram_edac_handle_irq(int irq, void *data)
{
	int err = 0;
	struct platform_device *pdev = (struct platform_device *)data;

	dev_dbg(&pdev->dev, "CSRAM Interrupt %d raised\n", irq);

	err = csram_edac_get_err_info(pdev, irq);
	dev_dbg(&pdev->dev, "CSRAM Error type %d", err);
	xbay_ras_clear_interrupt(irq);
	xbay_pcie_report_internal_error(err);

	return IRQ_HANDLED;
}

#if defined(DEBUG_RAS_CSRAM) && defined(DEBUG_CSRAM_HW_INJ)
static ssize_t tst_csram_edac_inj_error_store(struct device *dev,
					      struct device_attribute *mattr,
					      const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);
	u32 inj_error_val = 0;

	if (strncmp(data, "Parity", 6) == 0) {
		inj_error_val = ERR_INJ_PARITY;
	} else if (strncmp(data, "DecBG", 5) == 0) {
		inj_error_val = ERR_INJ_DEC_BG;
	} else if (strncmp(data, "DecFG", 5) == 0) {
		inj_error_val = ERR_INJ_DEC_FG;
	} else if (strncmp(data, "BG", 2) == 0) {
		inj_error_val = ERR_INJ_BG;
	} else if (strncmp(data, "FG", 2) == 0) {
		inj_error_val = ERR_INJ_FG;
	} else {
		dev_err(&pdev->dev, "Invalid option chosen\n");
		return 0;
	}

	dev_dbg(&pdev->dev, "Injecting %08x error on CSRAM ...\n",
		inj_error_val);
	writel(inj_error_val, csram_edac_base[0] + ERR_INJ);

	return count;
}
static DEVICE_ATTR_WO(tst_csram_edac_inj_error);
#endif

#if defined(DEBUG_RAS_CSRAM) && defined(DEBUG_CSRAM_SW_INJ)
static ssize_t tst_csram_edac_sw_inj_store(struct device *dev,
					   struct device_attribute *mattr,
					   const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);
	struct csram_edac_status_t *e_sts = NULL;
	int bank_num;
	bool se = false;

	if (strncmp(data, "SE_0", 4) == 0) {
		bank_num = 0; se = true;
	} else if (strncmp(data, "DE_0", 4) == 0) {
		bank_num = 0; se = false;
	} else if (strncmp(data, "SE_3", 4) == 0) {
		bank_num = 3; se = true;
	} else if (strncmp(data, "DE_3", 4) == 0) {
		bank_num = 3; se = false;
	} else {
		dev_err(&pdev->dev, "Invalid options chosen\n");
		dev_err(&pdev->dev, "SE_0, DE_0, SE_3, DE_3 are available\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "Injecting SW Error on CSRAM Bank%d\n",
		 bank_num);

	e_sts = &csram_edac_status[bank_num];
	if (se)
		e_sts->sbit_cnt = 0x02 + bank_num;
	else
		e_sts->dbit_cnt = 0x03 + bank_num;
	e_sts->err_acsramess = 0x5a5a5a + bank_num;
	e_sts->err_status = 0xa5a5 + bank_num;

	return count;
}
static DEVICE_ATTR_WO(tst_csram_edac_sw_inj);
#endif

#ifdef DEBUG_RAS_CSRAM
static const struct attribute *csram_edac_inject_attrs[] = {
#ifdef DEBUG_CSRAM_HW_INJ
	&dev_attr_tst_csram_edac_inj_error.attr,
#endif
#ifdef DEBUG_CSRAM_SW_INJ
	&dev_attr_tst_csram_edac_sw_inj.attr,
#endif
	NULL,
};
#endif

#ifdef DEBUG_RAS_CSRAM
static const struct attribute_group csram_edac_attributes = {
	.attrs = (struct attribute **)csram_edac_inject_attrs,
};
#endif

void xbay_csram_edac_get_all_info(struct platform_device *pdev,
				  char *buf, int count)
{
	memcpy(buf, (char *)csram_edac_status, count);
}

void xbay_csram_edac_clear_all_info(struct platform_device *pdev, int count)
{
	memset((char *)csram_edac_status, 0, count);
}

int xbay_csram_edac_probe(struct platform_device *pdev, struct device_node *np)
{
	int i, err;
	char of_name[20];

	err = of_property_read_u32(np, "num-banks", &num_banks);
	if (err) {
		dev_err(&pdev->dev, "missing num-banks property\n");
		return err;
	}

	if (num_banks > THB_CSRAM_BANK_NUM) {
		dev_err(&pdev->dev, "Expected CSRAM Banks %d, but got %d\n",
			THB_CSRAM_BANK_NUM,
			num_banks);
		return -EINVAL;
	}

	for (i = 0; i < num_banks; i++) {
		sprintf(of_name, "csram-bank%d-edac", i);

		/* Map CSRAM ECC Registers */
		csram_edac_base[i] =
			devm_platform_ioremap_resource_byname(pdev,
							      of_name);
		if (IS_ERR(csram_edac_base[i])) {
			dev_err(&pdev->dev,
				"Failed to I/O Map CSRAM ECC bank%d\n", i);
			return PTR_ERR(csram_edac_base[i]);
		}
	}

	for (i = 0; i < THB_CSRAM_IRQ_NUM; i++) {
		/* Get IRQ */
		sprintf(of_name, "csram-edac-irq%d", i);

		csram_edac_irq[i] = of_irq_get_byname(np, of_name);
		if (csram_edac_irq[i] < 0) {
			dev_err(&pdev->dev,
				"Failed to get IRQ csram-edac-irq%d\n", i);
			return csram_edac_irq[i];
		}

		/* Request IRQ */
		err = devm_request_threaded_irq(&pdev->dev,
						csram_edac_irq[i],
						csram_edac_handle_irq,
						NULL,
						IRQF_SHARED,
						of_name,
						pdev);

		if (err) {
			dev_err(&pdev->dev,
				"Failed to request IRQ %d\n",
				csram_edac_irq[i]);
			return err;
		}
	}

	err = sysfs_create_group(&pdev->dev.kobj, &csram_edac_attributes);
	if (err < 0) {
		dev_err(&pdev->dev,
			"Failed to create CSRAM EDAC sysfs entries\n");
		return err;
	}

	dev_info(&pdev->dev, "CSRAM EDAC module probed Successfully\n");

	return 0;
}
