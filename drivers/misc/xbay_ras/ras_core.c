// SPDX-License-Identifier: GPL-2.0 only
/*
 * Intel xBay RAS Collector Driver
 *
 * Copyright (C) 2021 Intel Corporation
 */

#include <linux/wait.h>

#include "ras_core.h"

/* Interrupt Status Register */
#define RAS_EDAC_INTR_STAT_0	0x0
#define RAS_EDAC_INTR_STAT_1	0x4
#define RAS_EDAC_INTR_BITS	64

#define RAS_DDR_EDAC_BMAP	BIT(0)
#define RAS_CSRAM_EDAC_BMAP	BIT(1)
#define RAS_CPU_EDAC_BMAP	BIT(2)
#define RAS_WDOG_EDAC_BMAP	BIT(3)
#define RAS_ALL_EDAC_BMAP	(RAS_DDR_EDAC_BMAP |  \
				RAS_CSRAM_EDAC_BMAP | \
				RAS_CPU_EDAC_BMAP | \
				RAS_WDOG_EDAC_BMAP)

static void __iomem *ras_base;
static u32 ras_intr_bit_map[RAS_EDAC_INTR_BITS], ras_num_intr;
static u32 record_last_edac_err, ns_wdog_sts;
static spinlock_t intr_stat_lock;
static DECLARE_WAIT_QUEUE_HEAD(err_sts_waitq);

static void xbay_wdog_get_all_info(struct platform_device *pdev,
				   char *buf, int count)
{
	unsigned long flags;

	memcpy(buf, &ns_wdog_sts, count);
	spin_lock_irqsave(&intr_stat_lock, flags);
	ns_wdog_sts = 0;
	spin_unlock_irqrestore(&intr_stat_lock, flags);
}

static ssize_t ras_get_edac_info(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t off, size_t count)
{
	struct platform_device *pdev =
			to_pdev(container_of(kobj, struct device, kobj));
	unsigned long flags;
	int read_map = 0x0;

	if (count == EDAC_STS_SZ) {
		wait_event_interruptible(err_sts_waitq,
					 (record_last_edac_err != 0));
		memcpy(buf + EDAC_STS_OFF, &record_last_edac_err, EDAC_STS_SZ);
		return count;
	} else if (count == DDR_EDAC_INFO_SZ) {
		read_map |= RAS_DDR_EDAC_BMAP;
	} else if (count == CSRAM_EDAC_INFO_SZ) {
		read_map |= RAS_CSRAM_EDAC_BMAP;
	} else if (count == CPU_EDAC_INFO_SZ) {
		read_map |= RAS_CPU_EDAC_BMAP;
	} else if (count == WDOG_INFO_SZ) {
		read_map |= RAS_WDOG_EDAC_BMAP;
	} else if (count == TOTAL_INFO_SZ) {
		read_map |= RAS_ALL_EDAC_BMAP;
	} else {
		dev_warn(&pdev->dev, "Invalid length to read\n");
		return -EINVAL;
	}

	memcpy(buf + EDAC_STS_OFF, &record_last_edac_err, EDAC_STS_SZ);
	if (read_map & RAS_DDR_EDAC_BMAP)
		xbay_ddr_edac_get_all_info(pdev, buf + DDR_EDAC_OFF,
					   DDR_EDAC_INFO_SZ);
	if (read_map & RAS_CSRAM_EDAC_BMAP)
		xbay_csram_edac_get_all_info(pdev, buf + CSRAM_EDAC_OFF,
					     CSRAM_EDAC_INFO_SZ);
	if (read_map & RAS_CPU_EDAC_BMAP)
		xbay_cpu_edac_get_all_info(pdev, buf + CPU_EDAC_OFF,
					   CPU_EDAC_INFO_SZ);
	if (read_map & RAS_WDOG_EDAC_BMAP)
		xbay_wdog_get_all_info(pdev, buf + WDOG_OFF,
				       WDOG_INFO_SZ);

	spin_lock_irqsave(&intr_stat_lock, flags);
	if (read_map & RAS_DDR_EDAC_BMAP)
		record_last_edac_err &= ~RAS_EDAC_INTR_STAT_0_DDR_MASK;
	if (read_map & RAS_CSRAM_EDAC_BMAP)
		record_last_edac_err &= ~RAS_EDAC_INTR_STAT_0_CSRAM_MASK;
	if (read_map & RAS_CPU_EDAC_BMAP)
		record_last_edac_err &= ~RAS_TST_CPU_INTR;
	if (read_map & RAS_WDOG_EDAC_BMAP)
		record_last_edac_err &= ~RAS_RESV_WDOG_INTR;
	spin_unlock_irqrestore(&intr_stat_lock, flags);

	return count;
}

static struct bin_attribute ras_info_attr = {
	.attr =	{
		.name = "get_edac_info",
		.mode = 0644,
	},
	.size = 4096,
	.read = ras_get_edac_info,
};

static struct bin_attribute *ras_edac_info_bin_attrs[] = {
	&ras_info_attr,
	NULL,
};

#ifdef DEBUG_RAS_SW_INJ
static ssize_t tst_set_ras_interrupt_store(struct device *dev,
					   struct device_attribute *mattr,
					   const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);
	u32 val = 0;
	unsigned long flags;

	if (strncmp(data, "DDR", 3) == 0) {
		val = RAS_TST_DDR_INTR;
	} else if (strncmp(data, "CSRAM", 5) == 0) {
		val = RAS_TST_CSRAM_INTR;
	} else if (strncmp(data, "CPU", 3) == 0) {
		val = RAS_TST_CPU_INTR;
	} else if (strncmp(data, "WDOG", 4) == 0) {
		val = RAS_RESV_WDOG_INTR;
	} else {
		dev_err(&pdev->dev, "Invalid options chosen\n");
		dev_err(&pdev->dev,
			"DDR, CSRAM, CPU, WDOG available\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&intr_stat_lock, flags);
	record_last_edac_err |= val;
	if (val == RAS_RESV_WDOG_INTR)
		ns_wdog_sts = XBAY_RAS_NS_WDOG_TO;
	spin_unlock_irqrestore(&intr_stat_lock, flags);
	wake_up_interruptible(&err_sts_waitq);

	return count;
}
static DEVICE_ATTR_WO(tst_set_ras_interrupt);

static ssize_t tst_clear_edac_info_store(struct device *dev,
					 struct device_attribute *mattr,
					 const char *data, size_t count)
{
	struct platform_device *pdev = to_pdev(dev);

	record_last_edac_err = 0;
	ns_wdog_sts = 0;
	xbay_ddr_edac_clear_all_info(pdev, DDR_EDAC_INFO_SZ);
	xbay_csram_edac_clear_all_info(pdev, CSRAM_EDAC_INFO_SZ);
	xbay_cpu_edac_clear_all_info(pdev, CPU_EDAC_INFO_SZ);

	return count;
}
static DEVICE_ATTR_WO(tst_clear_edac_info);

static const struct attribute *tst_ras_sw_inject_attrs[] = {
	&dev_attr_tst_set_ras_interrupt.attr,
	&dev_attr_tst_clear_edac_info.attr,
	NULL,
};
#endif

static const struct attribute_group ras_edac_info_attrs = {
	.bin_attrs = ras_edac_info_bin_attrs,
#ifdef DEBUG_RAS_SW_INJ
	.attrs = (struct attribute **)tst_ras_sw_inject_attrs,
#endif
};

void xbay_ras_clear_interrupt(int irq)
{
	struct irq_data *irqd = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(irqd);
	int bit;

	for (bit = 0; bit < ras_num_intr; bit++) {
		if (hwirq == ras_intr_bit_map[bit]) {
			record_last_edac_err |=
				   readl(ras_base + RAS_EDAC_INTR_STAT_0);
			writel(1 << bit, ras_base + RAS_EDAC_INTR_STAT_0);
			wake_up_interruptible(&err_sts_waitq);
			break;
		}
	}
}

void xbay_ras_notify_wdog_event(enum xbay_ras_wdog_event evnt)
{
	/* Called from IRQ context */
	ns_wdog_sts |= evnt;
	record_last_edac_err |= RAS_RESV_WDOG_INTR;
	xbay_pcie_report_internal_error(UNCORR_ERR);
	wake_up_interruptible(&err_sts_waitq);
}
EXPORT_SYMBOL(xbay_ras_notify_wdog_event);

static int xbay_ras_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node, *child;
	int err;

	ras_base = devm_platform_ioremap_resource_byname(pdev,
							 "ras-regs");
	if (IS_ERR(ras_base)) {
		dev_err(&pdev->dev, "Failed to I/O map of RAS regs\n");
		return PTR_ERR(ras_base);
	}

	/* Clear any errors, if already set */
	writel(RAS_EDAC_INTR_STAT_0_MASK, ras_base + RAS_EDAC_INTR_STAT_0);

	err = of_property_read_u32(node, "ras-num-intr",
				   &ras_num_intr);
	if (err) {
		dev_err(&pdev->dev, "missing ras-num-intr property\n");
		return err;
	}

	err = of_property_read_u32_array(node, "ras-intr-bit-map",
					 ras_intr_bit_map,
					 ras_num_intr);
	if (err) {
		dev_err(&pdev->dev, "missing ras-intr-bit-map property\n");
		return err;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &ras_edac_info_attrs);
	if (err < 0) {
		dev_err(&pdev->dev,
			"Failed to create RAS EDAC sysfs entries\n");
		return err;
	}

	for_each_child_of_node(node, child) {
		if (of_device_is_compatible(child, "snps,ddr-edac"))
			xbay_ddr_edac_probe(pdev, child);

		if (of_device_is_compatible(child, "netspeed,noc-csram-edac"))
			xbay_csram_edac_probe(pdev, child);

		if (of_device_is_compatible(child, "arm,cortex-a53-edac"))
			xbay_cortex_a53_edac_probe(pdev, child);

		if (of_device_is_compatible(child, "thb,pcie-ep-edac"))
			xbay_pcie_rasdes_probe(pdev, child);
	}

	spin_lock_init(&intr_stat_lock);
	dev_info(&pdev->dev, "RAS Driver probed Successfully\n");

	return 0;
}

static int xbay_ras_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id xbay_ras_of_match[] = {
	{ .compatible = "intel,xbay-ras", },
	{},
};
MODULE_DEVICE_TABLE(of, xbay_ras_of_match);

static struct platform_driver xbay_ras_driver = {
	.probe = xbay_ras_probe,
	.remove = xbay_ras_remove,
	.driver = {
		.name = "xbay_ras",
		.of_match_table = xbay_ras_of_match,
	},
};

module_platform_driver(xbay_ras_driver);

MODULE_AUTHOR("Intel");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("xBay RAS Driver");
