/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_HEADER_
#define XPCIE_HEADER_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/mempool.h>
#include <linux/dma-mapping.h>
#include <linux/cache.h>
#include <linux/wait.h>

#include "common.h"

#ifdef XLINK_PCIE_REMOTE
#define XPCIE_DRIVER_NAME "mxlk"
#define XPCIE_DRIVER_DESC "Intel(R) Keem Bay XLink PCIe driver"
#else
#define XPCIE_DRIVER_NAME "mxlk_pcie_epf"
#define XPCIE_DRIVER_DESC "Intel(R) xLink PCIe endpoint function driver"
#endif

struct xpcie_pipe {
	u32 old;
	u32 ndesc;
	u32 *head;
	u32 *tail;
	struct xpcie_transfer_desc *tdr;
};

struct xpcie_buf_desc {
	struct xpcie_buf_desc *next;
	void *head;
	dma_addr_t phys;
	size_t true_len;
	void *data;
	size_t length;
	int interface;
	bool own_mem;
};

struct xpcie_stream {
	size_t frag;
	struct xpcie_pipe pipe;
#ifdef XLINK_PCIE_REMOTE
	struct xpcie_buf_desc **ddr;
#endif
};

struct xpcie_list {
	/* list lock */
	spinlock_t lock;
	size_t bytes;
	size_t buffers;
	struct xpcie_buf_desc *head;
	struct xpcie_buf_desc *tail;
};

struct xpcie_interface {
	int id;
	struct xpcie *xpcie;
	struct mutex rlock; /* Read lock */
	struct xpcie_list read;
	struct xpcie_buf_desc *partial_read;
	bool data_avail;
	wait_queue_head_t rx_waitq;
};

struct xpcie_debug_stats {
	struct {
		size_t cnts;
		size_t bytes;
	} tx_krn, rx_krn, tx_usr, rx_usr;
	size_t send_ints;
	size_t interrupts;
	size_t rx_event_runs;
	size_t tx_event_runs;
};

struct xpcie {
	u32 status;
	bool legacy_a0;

#ifdef XLINK_PCIE_REMOTE
	void __iomem *bar0;
	struct xpcie_mmio __iomem *mmio; /* XLink memory space */
	void __iomem *bar4;
#else
	struct xpcie_mmio *mmio; /* XLink memory space */
	void *bar4;
#endif

	struct workqueue_struct *rx_wq;
	struct workqueue_struct *tx_wq;

	struct xpcie_interface interfaces[XPCIE_NUM_INTERFACES];

	size_t fragment_size;
	struct xpcie_cap_txrx *txrx;
	struct xpcie_stream tx;
	struct xpcie_stream rx;

	/* Write Lock */
	struct mutex wlock;
	struct xpcie_list write;
	bool no_tx_buffer;
	wait_queue_head_t tx_waitq;
	bool tx_pending;
	bool stop_flag;

	struct xpcie_list rx_pool;
	struct xpcie_list tx_pool;

	struct delayed_work rx_event;
	struct delayed_work tx_event;

	struct device_attribute debug;
	bool debug_enable;
	struct xpcie_debug_stats stats;
};

#endif /* XPCIE_HEADER_ */
