/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_STRUCT_HEADER_
#define XPCIE_STRUCT_HEADER_

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <pcie-keembay.h>
#include "../common/xpcie.h"

extern bool dma_ll_mode;

struct xpcie_dma_ll_desc {
	u32 dma_ch_control1;
	u32 dma_transfer_size;
	union {
		struct {
			u32 dma_sar_low;
			u32 dma_sar_high;
		};
		phys_addr_t src_addr;
	};
	union {
		struct {
			u32 dma_dar_low;
			u32 dma_dar_high;
		};
		phys_addr_t dst_addr;
	};
} __packed;

struct xpcie_dma_ll_desc_buf {
	struct xpcie_dma_ll_desc *virt;
	dma_addr_t phys;
	size_t size;
};

struct xpcie_epf {
	struct pci_epf			*epf;
	void				*vaddr[BAR_5 + 1];
	enum pci_barno			comm_bar;
	enum pci_barno			bar4;
	const struct pci_epc_features	*epc_features;
	struct xpcie			xpcie;
	int				irq;
	int				irq_dma;
	int				irq_err;
	void __iomem			*apb_base;
	void __iomem			*dma_base;

	irq_handler_t			core_irq_callback;
	dma_addr_t			tx_phys;
	void				*tx_virt;
	size_t				tx_size;
	dma_addr_t			rx_phys;
	void				*rx_virt;
	size_t				rx_size;

	struct xpcie_dma_ll_desc_buf	tx_desc_buf[4];
	struct xpcie_dma_ll_desc_buf	rx_desc_buf[4];
};

static inline struct device *xpcie_to_dev(struct xpcie *xpcie)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);

	return &xpcie_epf->epf->dev;
}

#endif // XPCIE_STRUCT_HEADER_
