/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel XPCIe XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_HEADER_
#define XPCIE_HEADER_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci_ids.h>
#include <linux/xlink_drv_inf.h>

#include "core.h"

#ifndef PCI_DEVICE_ID_INTEL_KEEMBAY
#define PCI_DEVICE_ID_INTEL_KEEMBAY 0x6240
#endif

#ifndef PCI_DEVICE_ID_INTEL_TBH_FULL
#define PCI_DEVICE_ID_INTEL_TBH_FULL 0x4FC0
#endif

#ifndef PCI_DEVICE_ID_INTEL_TBH_PRIME
#define PCI_DEVICE_ID_INTEL_TBH_PRIME 0x4FC1
#endif

#define XPCIE_IO_COMM_SIZE SZ_16K
#define XPCIE_MMIO_OFFSET SZ_4K

/* Status encoding of both device and host */
#define XPCIE_STATUS_ERROR	(0xFFFFFFFF)
#define XPCIE_STATUS_UNINIT	(0)
#define XPCIE_STATUS_READY	(1)
#define XPCIE_STATUS_RECOVERY	(2)
#define XPCIE_STATUS_OFF	(3)
#define XPCIE_STATUS_RUN	(4)

#define XPCIE_MAGIC_STRLEN	(16)
#define XPCIE_MAGIC_YOCTO	"VPUYOCTO"

/* MMIO layout and offsets shared between device and host */
struct xpcie_mmio {
	u32 device_status;
	u32 host_status;
#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
	u16 phy_dev_id;
	u8 max_functions;
#endif
	u8 legacy_a0;
	u8 htod_tx_doorbell;
	u8 htod_rx_doorbell;
	u8 htod_event_doorbell;
	u8 dtoh_tx_doorbell;
	u8 dtoh_rx_doorbell;
	u8 dtoh_event_doorbell;
#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
	u8 htod_phy_id_doorbell_status;
#endif
	u8 reserved;
	u32 cap_offset;
	u8 magic[XPCIE_MAGIC_STRLEN];
} __packed __aligned(8);

#define XPCIE_MMIO_LEGACY_A0	(offsetof(struct xpcie_mmio, legacy_a0))
#define XPCIE_MMIO_DEV_STATUS	(offsetof(struct xpcie_mmio, device_status))
#define XPCIE_MMIO_LEGACY_A0	(offsetof(struct xpcie_mmio, legacy_a0))
#define XPCIE_MMIO_HOST_STATUS	(offsetof(struct xpcie_mmio, host_status))
#define XPCIE_MMIO_LEGACY_A0	(offsetof(struct xpcie_mmio, legacy_a0))
#define XPCIE_MMIO_HTOD_TX_DOORBELL \
	(offsetof(struct xpcie_mmio, htod_tx_doorbell))
#define XPCIE_MMIO_HTOD_RX_DOORBELL \
	(offsetof(struct xpcie_mmio, htod_rx_doorbell))
#define XPCIE_MMIO_HTOD_EVENT_DOORBELL \
	(offsetof(struct xpcie_mmio, htod_event_doorbell))
#define XPCIE_MMIO_DTOH_TX_DOORBELL \
	(offsetof(struct xpcie_mmio, dtoh_tx_doorbell))
#define XPCIE_MMIO_DTOH_RX_DOORBELL \
	(offsetof(struct xpcie_mmio, dtoh_rx_doorbell))
#define XPCIE_MMIO_DTOH_EVENT_DOORBELL \
	(offsetof(struct xpcie_mmio, dtoh_event_doorbell))
#define XPCIE_MMIO_CAP_OFF	(offsetof(struct xpcie_mmio, cap_offset))
#define XPCIE_MMIO_MAGIC_OFF	(offsetof(struct xpcie_mmio, magic))

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))

#define XPCIE_MMIO_PHY_DEV_ID	(offsetof(struct xpcie_mmio, phy_dev_id))
#define XPCIE_MMIO_MAX_FUNCTIONS \
	(offsetof(struct xpcie_mmio, max_functions))
#define XPCIE_MMIO_HTOD_PHY_ID_DOORBELL_STATUS \
	(offsetof(struct xpcie_mmio, htod_phy_id_doorbell_status))
#endif

struct xpcie {
	u32 status;
	bool legacy_a0;
	void *bar0;
	void *mmio;
	void *bar4;
#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
	void __iomem *doorbell_base; /*IPC DoorBell address space*/
	void __iomem *doorbell_clear; /*IPC DoorBell clear address space*/
#endif
	struct workqueue_struct *rx_wq;
	struct workqueue_struct *tx_wq;

	struct xpcie_interface interfaces[XPCIE_NUM_INTERFACES];

	size_t fragment_size;
	struct xpcie_cap_txrx *txrx;
	struct xpcie_stream tx;
	struct xpcie_stream rx;

	struct mutex wlock; /* write lock */
	struct xpcie_list write;
	bool no_tx_buffer;
	wait_queue_head_t tx_waitq;
	bool tx_pending;
	bool stop_flag;

	struct xpcie_list rx_pool;
	struct xpcie_list tx_pool;

	struct delayed_work rx_event;
	struct delayed_work tx_event;
};

#endif /* XPCIE_HEADER_ */
