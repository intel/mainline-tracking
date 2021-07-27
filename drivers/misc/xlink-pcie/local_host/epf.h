/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel XPCIe XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_EPF_HEADER_
#define XPCIE_EPF_HEADER_

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

#include "../common/xpcie.h"
#include "../common/util.h"

#define XPCIE_DRIVER_NAME "intel_xpcie_pcie_epf"
#define XPCIE_DRIVER_DESC "Intel(R) xLink PCIe endpoint function driver"

#define KEEMBAY_XPCIE_STEPPING_MAXLEN 8

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
#define DMA_CHAN_NUM		(8)
#else
#define DMA_CHAN_NUM		(4)
#endif

#define XPCIE_NUM_TX_DESCS	(64)
#define XPCIE_NUM_RX_DESCS	(64)

extern bool dma_ll_mode;
extern u32 xlink_sw_id;

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
	struct pci_epf *epf;
	void *vaddr[BAR_5 + 1];
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	enum pci_barno                  doorbell_bar;
#endif
	enum pci_barno comm_bar;
	enum pci_barno bar4;
	const struct pci_epc_features *epc_features;
	struct xpcie xpcie;
	int irq;
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	int                             irq_doorbell;
	int                             irq_rdma;
	int                             irq_wdma;
	int				irq_flr;
	wait_queue_head_t		dma_rd_wq;
	bool				dma_rd_done;
	wait_queue_head_t		dma_wr_wq;
	bool				dma_wr_done;
#else
	int irq_dma;
	int irq_err;
#endif
	void __iomem *apb_base;
	void __iomem *dma_base;
	void __iomem *dbi_base;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	struct resource *doorbell_base;
	struct resource *doorbell_clear;
	struct resource mmr2;
	struct resource mmr4;
	bool tbh_half;
#endif

	char stepping[KEEMBAY_XPCIE_STEPPING_MAXLEN];

	irq_handler_t			core_irq_callback;
	dma_addr_t			tx_phys;
	void				*tx_virt;
	size_t				tx_size;

	struct xpcie_dma_ll_desc_buf     tx_desc_buf;
	struct xpcie_dma_ll_desc_buf     rx_desc_buf;

	struct hrtimer			free_tx_dma_timer;
	struct hrtimer			free_rx_dma_timer;
	atomic_t			dma_wr_eng_reset_cnt;
	u32				dma_wr_int_status;
	u32				dma_wr_err_status;
	int				dma_wr_rc;
	atomic_t			dma_rd_eng_reset_cnt;
	u32				dma_rd_int_status;
	u32				dma_rd_err_status_low;
	u32				dma_rd_err_status_high;
	int				dma_rd_rc;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
#define MXLK_MAX_NAME_LEN (32)
	char				name[MXLK_MAX_NAME_LEN];
	u32				sw_devid;
	bool				sw_dev_id_updated;
	struct list_head		list;
	xlink_device_event		event_fn;

	struct work_struct		flr_irq_event;
#endif
};

static inline struct device *xpcie_to_dev(struct xpcie *xpcie)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);

	return &xpcie_epf->epf->dev;
}

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
struct xpcie_epf *intel_xpcie_get_device_by_name(const char *name);
#endif
int intel_xpcie_ep_dma_init(struct pci_epf *epf);
int intel_xpcie_ep_dma_uninit(struct pci_epf *epf);
int intel_xpcie_ep_dma_reset(struct pci_epf *epf);
int intel_xpcie_ep_dma_read_ll(struct pci_epf *epf, int chan, int descs_num);
int intel_xpcie_ep_dma_write_ll(struct pci_epf *epf, int chan, int descs_num);
void intel_xpcie_ep_stop_dma(struct pci_epf *epf);
void intel_xpcie_ep_start_dma(struct pci_epf *epf);

void intel_xpcie_register_host_irq(struct xpcie *xpcie,
				   irq_handler_t func);
int intel_xpcie_raise_irq(struct xpcie *xpcie,
			  enum xpcie_doorbell_type type,
			  u8 value);
int intel_xpcie_copy_from_host_ll(struct xpcie *xpcie,
				  int chan, int descs_num);
int intel_xpcie_copy_to_host_ll(struct xpcie *xpcie,
				int chan, int descs_num);

/*
 * Sysfs entry for sw device id.
 */
void intel_xpcie_init_sysfs_swdev_id(struct xpcie *xpcie, struct device *dev);
void intel_xpcie_uninit_sysfs_swdev_id(struct xpcie *xpcie, struct device *dev);

void intel_xpcie_pci_notify_event(struct xpcie_epf *xdev,
				  enum xlink_device_event_type event_type);

#endif /* XPCIE_EPF_HEADER_ */
