/* SPDX-License-Identifier: GPL-2.0 only */
/*
 * Intel xBay RAS: Core Header file
 *
 * Copyright (C) 2021 Intel Corporation
 */

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/xbay_ras.h>

#define DEBUG_RAS_SW_INJ

#define DEBUG_RAS_DDR
#define DEBUG_DDR_SW_INJ
#undef DEBUG_DDR_HW_INJ

#define DEBUG_RAS_CSRAM
#define DEBUG_CSRAM_SW_INJ
#undef DEBUG_CSRAM_HW_INJ

#define DEBUG_RAS_CPU
#define DEBUG_CPU_SW_INJ
#undef DEBUG_CPU_HW_INJ

#define to_pdev(devp) container_of(devp, struct platform_device, dev)

enum error_type {
	NO_ERR = -1,
	CORR_ERR,
	UNCORR_ERR,
};

/* Function declarations */
int xbay_pcie_rasdes_probe(struct platform_device *pdev,
			   struct device_node *np);
int xbay_ddr_edac_probe(struct platform_device *pdev,
			struct device_node *np);
int xbay_cortex_a53_edac_probe(struct platform_device *pdev,
			       struct device_node *np);
int xbay_csram_edac_probe(struct platform_device *pdev,
			  struct device_node *np);
void xbay_ddr_edac_get_all_info(struct platform_device *pdev,
				char *buf, int count);
void xbay_csram_edac_get_all_info(struct platform_device *pdev,
				  char *buf, int count);
void xbay_cpu_edac_get_all_info(struct platform_device *pdev,
				char *buf, int count);
void xbay_ddr_edac_clear_all_info(struct platform_device *pdev,
				  int count);
void xbay_csram_edac_clear_all_info(struct platform_device *pdev,
				    int count);
void xbay_cpu_edac_clear_all_info(struct platform_device *pdev,
				  int count);
void xbay_pcie_report_internal_error(bool ce);
void xbay_ras_clear_interrupt(int value);
void xbay_ras_notify_wdog_event(enum xbay_ras_wdog_event evnt);
