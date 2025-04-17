/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023-2025 Intel Corporation */
#ifndef _ISSEI_DEV_H_
#define _ISSEI_DEV_H_

#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "dma.h"

struct device;

struct issei_device;
struct issei_host_client;

extern struct class *issei_class;

#define ISSEI_HOST_CLIENTS_MAX		255

#define ISSEI_SUPPORTED_PROTOCOL_VER	1

#define ISSEI_MAX_CONSEC_RESET		3

#define ISSEI_RST_HW_READY_TIMEOUT_MSEC	2000
#define ISSEI_RST_STEP_TIMEOUT_MSEC	2000
#define ISSEI_STOP_TIMEOUT_MSEC		500
#define ISSEI_WRITE_TIMEOUT_MSEC	1000

/**
 * struct issei_write_buf - write buffer object
 * @list: linked list pointer
 * @cl: host client that requested this write
 * @data: data to write
 * @data_size: data size
 */
struct issei_write_buf {
	struct list_head list;
	struct issei_host_client *cl;
	const u8 *data;
	size_t data_size;
};

/**
 * struct issei_hw_ops - callbacks for hardware operations
 * @irq_clear: clear irq
 * @irq_enable: enable irq
 * @irq_disable: disable irq
 * @irq_sync: sync irq
 * @hw_reset: initiate hardware reset
 * @hw_config: initial hardware config
 * @hw_is_ready: check if hardware is ready
 * @hw_reset_release: release hardware from reset
 * @host_set_ready: set host ready indicator
 * @setup_message_send: send setup message
 * @setup_message_recv: receive setup message
 * @irq_write_generate: generate interrupt on write complete
 */
struct issei_hw_ops {
	void (*irq_clear)(struct issei_device *idev);
	void (*irq_enable)(struct issei_device *idev);
	void (*irq_disable)(struct issei_device *idev);
	void (*irq_sync)(struct issei_device *idev);
	int (*hw_reset)(struct issei_device *idev, bool enable);
	int (*hw_config)(struct issei_device *idev);
	bool (*hw_is_ready)(struct issei_device *idev);
	void (*hw_reset_release)(struct issei_device *idev);
	void (*host_set_ready)(struct issei_device *idev);
	int (*setup_message_send)(struct issei_device *idev);
	int (*setup_message_recv)(struct issei_device *idev);
	int (*irq_write_generate)(struct issei_device *idev);
};

/**
 * enum issei_rst_state: driver reset flow states
 * @ISSEI_RST_STATE_INIT: initial state
 * @ISSEI_RST_STATE_HW_READY: waiting for HW to be ready
 * @ISSEI_RST_STATE_SETUP: waiting for channel setup completion
 * @ISSEI_RST_STATE_START: waiting for start handshake completion
 * @ISSEI_RST_STATE_CLIENT_ENUM: waiting for client enumeration
 * @ISSEI_RST_STATE_DONE: reset flow is done
 * @ISSEI_RST_STATE_DISABLED: flow is disabled
 */
enum issei_rst_state {
	ISSEI_RST_STATE_INIT,
	ISSEI_RST_STATE_HW_READY,
	ISSEI_RST_STATE_SETUP,
	ISSEI_RST_STATE_START,
	ISSEI_RST_STATE_CLIENT_ENUM,
	ISSEI_RST_STATE_DONE,
	ISSEI_RST_STATE_DISABLED,
};

/**
 * struct issei_device - issei device
 * @dev: associated device object
 * @cdev: character device
 * @minor: allocated minor number
 * @power_down: device is powering down
 * @wait_rst_irq: wait queue for irq
 * @rst_irq: irq processing flag
 * @wait_rst_state: waitqueue for reset state processing
 * @rst_state: reset state
 * @fw_protocol_ver: protocol version
 * @fw_version: firmware version
 * @reset_thread: worker thread
 * @reset_count: number of consecutive link reset attempts
 * @all_reset_count: cumilative number of link reset attempts
 * @host_client_lock: mutex to protect host clients list
 * @host_client_list: host clients list
 * @host_client_last_id: last allocated host client id
 * @host_client_count: number of active host clients
 * @fw_client_lock: mutex to protect firmware clients list
 * @fw_client_list: firmware clients list
 * @dma: DMA memory configuration
 * @write_queue: write queue
 * @last_write_ts: last write timestamp
 * @ops: hardware opertions
 * @hw: hw-specific data
 */
struct issei_device {
	struct device *dev;
	struct cdev cdev;
	int minor;
	bool power_down;
	wait_queue_head_t wait_rst_irq;
	atomic_t rst_irq;
	wait_queue_head_t wait_rst_state;
	enum issei_rst_state rst_state;
	u16 fw_protocol_ver;
	u16 fw_version[4];
	/* reset flow */
	struct task_struct *reset_thread;
	u8 reset_count;
	u8 all_reset_count;
	/* host clients */
	struct mutex host_client_lock;
	struct list_head host_client_list;
	u16 host_client_last_id;
	u8 host_client_count;
	/* fw clients */
	struct mutex fw_client_lock;
	struct list_head fw_client_list;
	struct issei_dma dma;
	struct list_head write_queue;
	ktime_t last_write_ts;
	const struct issei_hw_ops *ops;
	char hw[] __aligned(sizeof(void *));
};

#endif /* _ISSEI_DEV_H_ */
