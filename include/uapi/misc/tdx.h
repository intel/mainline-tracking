/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_MISC_TDX_H
#define _UAPI_MISC_TDX_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Input report data length for TDX_CMD_GET_TDREPORT IOCTL request */
#define TDX_REPORT_DATA_LEN		64
/* Output report data length after TDX_CMD_GET_TDREPORT IOCTL execution */
#define TDX_TDREPORT_LEN		1024

/* IOCTL to request TDREPORT data from TDX Module */
#define TDX_CMD_GET_TDREPORT		_IOWR('T', 0x01, __u64)
/* IOCTL to request Quote from VMM using report data */
#define TDX_CMD_GEN_QUOTE		_IOR('T', 0x02, __u64)
/* Get current size of quote */
#define TDX_CMD_GET_QUOTE_SIZE		_IOR('T', 0x03, __u64)

#endif /* _UAPI_MISC_TDX_H */
