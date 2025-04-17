/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2023-2025 Intel Corporation
 * Intel Silicon Security Engine Interface (ISSEI) Linux driver:
 * ISSEI Interface Header
 */
#ifndef _LINUX_ISSEI_H
#define _LINUX_ISSEI_H

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * This IOCTL is used to associate the current file descriptor with a
 * FW Client (given by UUID). This opens a communication channel
 * between a host client and a FW client. From this point every read and write
 * will communicate with the associated FW client.
 * The communication between the clients can be terminated by
 * IOCTL_ISSEI_DISCONNECT_CLIENT IOCTL or by
 * closing the file descriptor (file_operation release()).
 *
 * The IOCTL argument is a struct with a union that contains
 * the input parameter and the output parameter for this IOCTL.
 *
 * The input parameter is UUID of the FW Client.
 * The output parameter is the properties of the FW client
 * (FW protocol version, max message size and client flags).
 */
#define IOCTL_ISSEI_CONNECT_CLIENT \
	_IOWR('H', 0x01, struct issei_connect_client_data)

/**
 * struct issei_client - ISSEI client information structure
 * @max_msg_length: maximum message length supported by the firmware client (in bytes)
 * @protocol_version: protocol version reported by the firmware client
 * @reserved: reserved
 * @flags: flag bitmask reported by the firmware client
 */
struct issei_client {
	__u32 max_msg_length;
	__u8 protocol_version;
	__u8 reserved[3];
	__u32 flags;
};

#define ISSEI_IOCTL_UUID_LEN 16

/**
 * struct issei_connect_client_data - IOCTL Connect Client Data structure
 * @in_client_uuid: unique id of the firmware client to connect to
 * @out_client_properties: connected firmware client properties
 */
struct issei_connect_client_data {
	union {
		__u8 in_client_uuid[ISSEI_IOCTL_UUID_LEN];
		struct issei_client out_client_properties;
	};
};

/*
 * This IOCTL is used to terminate association between
 * the host client and the FW client.
 */
#define IOCTL_ISSEI_DISCONNECT_CLIENT \
	_IO('H', 0x02)

/*
 * This IOCTL is used to obtain driver status
 *
 * The 32bit output parameter:
 * Bit[0] - driver reset status (1 - in reset, 0 - out of reset)
 * Bit[1-7] - Reserved
 * Bit[8-15] - Number of resets performed after driver load
 */
#define IOCTL_ISSEI_DRIVER_STATUS \
	_IOR('H', 0x03, __u32)

#endif /* _LINUX_ISSEI_H */
