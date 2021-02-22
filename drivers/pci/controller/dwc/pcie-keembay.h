/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * pcie-keembay - PCIe controller driver for Intel Keem Bay
 *
 * Copyright (C) 2019 Intel Corporation
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

#ifndef _PCIE_KEEMBAY_H
#define _PCIE_KEEMBAY_H

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "pcie-designware.h"

#define KEEMBAY_PCIE_STEPPING_MAXLEN 8

struct keembay_pcie {
	struct dw_pcie		*pci;
	void __iomem		*base;
	enum dw_pcie_device_mode mode;
	char			stepping[KEEMBAY_PCIE_STEPPING_MAXLEN];
	int			irq;
	int			ev_irq;
	int			err_irq;
	int			mem_access_irq;

	struct clk		*clk_master;
	struct clk		*clk_aux;
	struct gpio_desc	*reset;
};

#define to_keembay_pcie(x)	dev_get_drvdata((x)->dev)

struct thunderbay_pcie {
	struct dw_pcie          *pci;
	void __iomem            *base;
	enum dw_pcie_device_mode mode;
	struct resource         *doorbell_base;
	struct resource         *doorbell_clear;
	struct resource         *mmr2[8];
	struct resource         *mmr4[8];
	bool                    setup_bar[8];
	int                     irq;
	int                     irq_doorbell[8];
	int                     irq_rdma[8];
	int                     irq_wdma[8];

	bool                    tbh_half;

	void __iomem            *cpr_base;
	struct dma_pool         *rc_dma_pool;
	dma_addr_t              rc_dma_mem_pa;
	void                    *rc_dma_mem_va;
	struct dma_chan         *rd_dma_chan;
	bool                    rc_dma;
};

#define to_thunderbay_pcie(x)				dev_get_drvdata((x)->dev)

#endif /* _PCIE_KEEMBAY_H */
