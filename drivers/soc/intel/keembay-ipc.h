/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Keem Bay IPC header.
 *
 * Copyright (C) 2018-2021 Intel Corporation
 */

struct keembay_ipc;

struct keembay_ipc *keembay_ipc_init(struct device *dev);

void keembay_ipc_deinit(struct keembay_ipc *kmb_ipc);

int keembay_ipc_get_vpu_mem_info(struct keembay_ipc *kmb_ipc,
				 dma_addr_t *vpu_addr, size_t *size);

int keembay_ipc_open_channel(struct keembay_ipc *kmb_ipc, u16 chan_id);

int keembay_ipc_close_channel(struct keembay_ipc *kmb_ipc, u16 chan_id);

int keembay_ipc_send(struct keembay_ipc *kmb_ipc, u16 chan_id, u32 vpu_addr,
		     size_t size);

int keembay_ipc_recv(struct keembay_ipc *kmb_ipc, u16 chan_id, u32 *vpu_addr,
		     size_t *size, u32 timeout);
