/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <syscall.h>
#include <errno.h>
#include <x86gprintrin.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include "../kselftest.h"

#ifndef __NR_uintr_register_handler
#define __NR_uintr_register_handler	450
#define __NR_uintr_unregister_handler	451
#define __NR_uintr_create_fd		452
#define __NR_uintr_register_sender	453
#define __NR_uintr_unregister_sender	454
#define __NR_uintr_wait			455
#endif

#define uintr_register_handler(handler, flags)	syscall(__NR_uintr_register_handler, handler, flags)
#define uintr_unregister_handler(flags)		syscall(__NR_uintr_unregister_handler, flags)
#define uintr_create_fd(vector, flags)		syscall(__NR_uintr_create_fd, vector, flags)
#define uintr_register_sender(fd, flags)	syscall(__NR_uintr_register_sender, fd, flags)
#define uintr_unregister_sender(fd, flags)	syscall(__NR_uintr_unregister_sender, fd, flags)
#define uintr_wait(flags)			syscall(__NR_uintr_wait, flags)

void __attribute__((interrupt)) uintr_empty_handler(struct __uintr_frame *ui_frame,
						    unsigned long long vector)
{
}

static inline int uintr_supported(void)
{
	if (!uintr_register_handler(uintr_empty_handler, 0)) {
		uintr_unregister_handler(0);
		return 1;
	}

	if (errno == EBUSY)
		return 1;

	return 0;
}

static inline void cpu_relax(void)
{
	asm volatile("rep; nop" ::: "memory");
}

void cpu_sets(int cpu, cpu_set_t *set)
{
	CPU_ZERO(set);
	CPU_SET(cpu, set);
	sched_setaffinity(cpu, sizeof(cpu_set_t), set);
}

ssize_t sock_fd_write(int sock, void *buf, ssize_t buflen, int fd)
{
	ssize_t size;
	struct msghdr msg;
	struct iovec iov;
	union {
		struct cmsghdr cmsghdr;
		char control[CMSG_SPACE(sizeof(int))];
	} cmsgu;
	struct cmsghdr *cmsg;

	iov.iov_base = buf;
	iov.iov_len = buflen;
	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	if (fd != -1) {
		msg.msg_control = cmsgu.control;
		msg.msg_controllen = sizeof(cmsgu.control);

		cmsg = CMSG_FIRSTHDR(&msg);
		cmsg->cmsg_len = CMSG_LEN(sizeof(int));
		cmsg->cmsg_level = SOL_SOCKET;
		cmsg->cmsg_type = SCM_RIGHTS;

		*((int *)CMSG_DATA(cmsg)) = fd;
	} else {
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
	}
	size = sendmsg(sock, &msg, 0);
	if (size < 0)
		exit(0);

	return size;
}

ssize_t sock_fd_read(int sock, void *buf, ssize_t bufsize, int *fd)
{
	ssize_t size;

	if (fd) {
		struct msghdr msg;
		struct iovec iov;
		union {
			struct cmsghdr cmsghdr;
			char control[CMSG_SPACE(sizeof(int))];
		} cmsgu;
		struct cmsghdr *cmsg;

		iov.iov_base = buf;
		iov.iov_len = bufsize;
		msg.msg_name = NULL;
		msg.msg_namelen = 0;
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;
		msg.msg_control = cmsgu.control;
		msg.msg_controllen = sizeof(cmsgu.control);
		size = recvmsg(sock, &msg, 0);
		if (size < 0)
			exit(0);

		cmsg = CMSG_FIRSTHDR(&msg);
		if (cmsg && cmsg->cmsg_len == CMSG_LEN(sizeof(int))) {
			if (cmsg->cmsg_level != SOL_SOCKET)
				exit(0);
			if (cmsg->cmsg_type != SCM_RIGHTS)
				exit(0);

			*fd = *((int *)CMSG_DATA(cmsg));
		} else {
			*fd = -1;
		}
	} else {
		size = read(sock, buf, bufsize);
		if (size < 0)
			exit(0);
	}

	return size;
}
