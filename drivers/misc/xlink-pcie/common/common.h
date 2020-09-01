/* SPDX-License-Identifier: GPL-2.0-only */
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#ifndef XPCIE_COMMON_HEADER_
#define XPCIE_COMMON_HEADER_

#include <linux/io.h>
#include <linux/types.h>
#include <linux/pci_ids.h>
#include <linux/sizes.h>

#ifndef PCI_DEVICE_ID_INTEL_KEEMBAY
#define PCI_DEVICE_ID_INTEL_KEEMBAY 0x6240
#endif

/*
 * Number of interfaces to statically allocate resources for
 */
#define XPCIE_NUM_INTERFACES (1)

#define XPCIE_FRAGMENT_SIZE SZ_128K
#define XPCIE_NUM_TX_DESCS (64)
#define XPCIE_NUM_RX_DESCS (64)

/*
 * Status encoding of the transfer descriptors
 */
#define XPCIE_DESC_STATUS_SUCCESS (0)
#define XPCIE_DESC_STATUS_ERROR (0xFFFF)

/*
 * Layout transfer descriptors used by device and host
 */
struct xpcie_transfer_desc {
	uint64_t address;
	uint32_t length;
	uint16_t status;
	uint16_t interface;
} __packed;

#define XPCIE_IO_COMM_SIZE SZ_16K
#define XPCIE_MMIO_OFFSET SZ_4K

#define XPCIE_VERSION_MAJOR 0
#define XPCIE_VERSION_MINOR 5
#define XPCIE_VERSION_BUILD 0
#define _TOSTR(X) #X
#define _VERSION(A, B, C) _TOSTR(A) "." _TOSTR(B) "." _TOSTR(C)
#define XPCIE_DRIVER_VERSION \
	_VERSION(XPCIE_VERSION_MAJOR, XPCIE_VERSION_MINOR, XPCIE_VERSION_BUILD)

struct xpcie_version {
	uint8_t major;
	uint8_t minor;
	uint16_t build;
} __packed;

/*
 * Status encoding of both device and host
 */
#define XPCIE_STATUS_ERROR (0xFFFFFFFF)
#define XPCIE_STATUS_UNINIT (0)
#define XPCIE_STATUS_BOOT_FW (1)
#define XPCIE_STATUS_BOOT_OS (2)
#define XPCIE_STATUS_READY (3)
#define XPCIE_STATUS_RECOVERY (4)
#define XPCIE_STATUS_RUN (5)
#define XPCIE_STATUS_OFF (6)
#define XPCIE_STATUS_BOOT_PRE_OS (7)

/*
 * MMIO layout and offsets shared between device and host
 */
struct xpcie_mmio {
	struct xpcie_version version;
	uint32_t device_status;
	uint32_t host_status;
	uint8_t legacy_a0;
	uint8_t htod_tx_doorbell;
	uint8_t htod_rx_doorbell;
	uint8_t htod_event_doorbell;
	uint8_t dtoh_tx_doorbell;
	uint8_t dtoh_rx_doorbell;
	uint8_t dtoh_event_doorbell;
	uint8_t reserved;
	uint32_t cap_offset;
} __packed;

/*
 * Defined capabilities located in mmio space
 */
#define XPCIE_CAP_NULL (0)
#define XPCIE_CAP_TXRX (1)

/*
 * Header at the beginning of each capability to define and link to next
 */
struct xpcie_cap_hdr {
	uint16_t id;
	uint16_t next;
} __packed;

#define XPCIE_CAP_HDR_ID (offsetof(struct xpcie_cap_hdr, id))
#define XPCIE_CAP_HDR_NEXT (offsetof(struct xpcie_cap_hdr, next))

struct xpcie_cap_pipe {
	uint32_t ring;
	uint32_t ndesc;
	uint32_t head;
	uint32_t tail;
} __packed;

/*
 * Transmit and Receive capability
 */
struct xpcie_cap_txrx {
	struct xpcie_cap_hdr hdr;
	u32 fragment_size;
	struct xpcie_cap_pipe tx;
	struct xpcie_cap_pipe rx;
} __packed;

static inline u64 _ioread64(void __iomem *addr)
{
	u64 low, high;

	low = ioread32(addr);
	high = ioread32(addr + sizeof(u32));

	return low | (high << 32);
}

static inline void _iowrite64(u64 value, void __iomem *addr)
{
	iowrite32(value, addr);
	iowrite32(value >> 32, addr + sizeof(u32));
}

#ifdef XLINK_PCIE_REMOTE

#define intel_xpcie_iowrite64 _iowrite64
#define intel_xpcie_iowrite32 iowrite32
#define intel_xpcie_iowrite16 iowrite16
#define intel_xpcie_iowrite8 iowrite8
#define intel_xpcie_ioread64 _ioread64
#define intel_xpcie_ioread32 ioread32
#define intel_xpcie_ioread16 ioread16
#define intel_xpcie_ioread8 ioread8

#else

#define intel_xpcie_iowrite64(value, addr)	{ *(addr) = value; }
#define intel_xpcie_iowrite32(value, addr)	{ *(addr) = value; }
#define intel_xpcie_iowrite16(value, addr)	{ *(addr) = value; }
#define intel_xpcie_iowrite8(value, addr)	{ *(addr) = value; }
#define intel_xpcie_ioread64(addr) (*(addr))
#define intel_xpcie_ioread32(addr) (*(addr))
#define intel_xpcie_ioread16(addr) (*(addr))
#define intel_xpcie_ioread8(addr)  (*(addr))

#endif // XLINK_PCIE_REMOTE

#endif
