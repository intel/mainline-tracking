/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note
 * VPU manager Linux Kernel API
 * Copyright (C) 2020-2021 Intel Corporation
 *
 */
#ifndef __VPUMGR_UAPI_H
#define __VPUMGR_UAPI_H

#include <linux/types.h>

/* ioctl numbers */
#define VPUMGR_MAGIC 'V'
/* VPU manager IOCTLs */
#define VPUMGR_IOCTL_DMABUF_ALLOC	_IOWR(VPUMGR_MAGIC, 2, struct vpumgr_args_alloc)
#define VPUMGR_IOCTL_DMABUF_IMPORT	_IOWR(VPUMGR_MAGIC, 3, struct vpumgr_args_import)
#define VPUMGR_IOCTL_DMABUF_UNIMPORT	_IOWR(VPUMGR_MAGIC, 4, __s32)
#define VPUMGR_IOCTL_DMABUF_PTR2VPU	_IOWR(VPUMGR_MAGIC, 5, __u64)
#define VPUMGR_IOCTL_VCM_SUBMIT		_IOWR(VPUMGR_MAGIC, 6, struct vpumgr_vcm_submit)
#define VPUMGR_IOCTL_VCM_WAIT		_IOWR(VPUMGR_MAGIC, 7, struct vpumgr_vcm_wait)
#define VPUMGR_IOCTL_FETCH_META	_IOWR(VPUMGR_MAGIC, 8, struct vpumgr_args_fetch_meta)
#define VPUMGR_IOCTL_END		_IO(VPUMGR_MAGIC, 9)

struct vpumgr_args_alloc {
	__s32 fd;           /* out: DMABuf fd */
	__s32 reserved;     /*  in: reserved */
	__s32 noncoherent; /*  in: if noncoherent buffer is required */
	__u64 size;	    /*  in: required buffer size */
};

/* vpu_access flags */
enum vpu_access_type {
	VPU_ACCESS_DEFAULT = 0,
	VPU_ACCESS_READ    = 1,
	VPU_ACCESS_WRITE   = 2,
	VPU_ACCESS_RW      = 3
};

struct vpumgr_args_import {
	__s32 fd;           /*  in: input DMABuf fd */
	__s32 vpu_access;   /*  in: how vpu is going to access the buffer */
	__u64 vpu_addr;	    /* out: vpu dma address of the DMABuf */
	__u64 size;         /* out: the size of the DMABuf */
};

/* Command code reserved for kernel mode driver,
 * user-space should not use commmand code smaller
 * than or equal to this micro
 */
#define VCTX_KMD_RESERVED_CMD_LAST           31

struct vpumgr_vcm_submit {
	__u32 cmd;          /*  in: command code */
	__u64 in;           /*  in: input paramer buffer address */
	__u32 in_len;       /*  in: input paramer buffer length */
	__s32 submit_id;    /* out: submit id */
};

struct vpumgr_vcm_wait {
	__s32 submit_id;    /*  in: submit id */
	__s32 vpu_rc;       /* out: vpu return code */
	__u64 out;          /*  in: address of the buffer for receiving result */
	__u32 out_len;      /*  in: length of the result buffer */
	__u32 timeout_ms;   /*  in: timeout in milliseconds */
};

struct _VIV_VIDMEM_METADATA {
	__u32 magic;                /* __FOURCC('v', 'i', 'v', 'm') */
	__u32 dmabuf_size;          /* DMABUF buffer size in byte (Maximum 4GB) */
	__u32 time_stamp;           /* time stamp for the DMABUF buffer */
	__u32 image_format;         /* ImageFormat */
	__u32 compressed;           /* if DMABUF buffer is compressed by DEC400 */

	struct {
		__u32 offset;            /* plane buffer address offset from DMABUF address */
		__u32 stride;            /* pitch in byte */
		__u32 width;             /* width in pixels */
		__u32 height;            /* height in pixels */
		__u32 tile_format;       /* uncompressed tile format */
		__u32 compress_format;   /* tile mode for DEC400 */
		__u32 ts_offset;         /* tile status buffer offset within this plane buffer */
		__s32 ts_fd;             /* fd of separate tile status buffer of the plane buffer */
		__s32 ts_fd2;            /* valid fd of the ts buffer in consumer side */
		__s32 ts_vaddr;          /* the vpu virtual address for this ts data buffer */
		__u32 fc_enabled;        /* gpu fastclear enabled for the plane buffer */
		__u32 fc_value_lower;    /* gpu fastclear color value (lower 32 bits)
								for the plane buffer */
		__u32 fc_value_upper;    /* gpu fastclear color value (upper 32 bits)
								for the plane buffer */
	} plane[3];
};

struct vpumgr_args_fetch_meta {
	__s32 fd;           /*  in: input DMABuf fd */
	__u64 size;         /*  in: the size of struct _VIV_VIDMEM_METADATA */
	void *meta_buffer; /*out: buffer pointer to get VSI Meta Data */
};
#endif /* __VPUMGR_UAPI_H */
