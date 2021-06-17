// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <x86gprintrin.h>

#include "uintr_common.h"

unsigned int total_nerrs;

static void test_register_handler_flags(void)
{
	unsigned int invalid_flag_mask, nerrs, flag, bit;

	/* Test valid flags first and remove them from the mask */
	/* None at this point */
	invalid_flag_mask = 0xFFFFFFFF;
	nerrs = 0;

	printf("[RUN]\tTest uintr_register_handler invalid flags\n");
	for (bit = 0; bit < sizeof(unsigned int); bit++) {
		flag = invalid_flag_mask & (1 << bit);
		if (flag && !uintr_register_handler(uintr_empty_handler, flag)) {
			nerrs++;
			uintr_unregister_handler(0);
		}
	}

	if (nerrs) {
		printf("[FAIL]\tuintr_register_handler accepted an invalid flag\n");
		total_nerrs += nerrs;
	} else {
		printf("[OK]\tuintr_register_handler returned errors on all invalid flags\n");
	}
}

static void test_unregister_handler_flags(void)
{
	unsigned int invalid_flag_mask, nerrs, flag, bit;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Test valid flags first and remove them from the mask */
	/* None at this point */
	invalid_flag_mask = 0xFFFFFFFF;
	nerrs = 0;

	printf("[RUN]\tTest uintr_unregister_handler invalid flags\n");
	for (bit = 0; bit < sizeof(unsigned int); bit++) {
		flag = invalid_flag_mask & (1 << bit);
		if (flag && !uintr_unregister_handler(flag)) {
			nerrs++;
			uintr_register_handler(uintr_empty_handler, flag);
		}
	}

	if (nerrs) {
		printf("[FAIL]\tuintr_unregister_handler accepted an invalid flag\n");
		total_nerrs += nerrs;
	} else {
		printf("[OK]\tuintr_unregister_handler returned errors on all invalid flags\n");
	}

	uintr_unregister_handler(0);
}

static void test_create_fd_flags(void)
{
	unsigned int invalid_flag_mask, nerrs, flag, bit;
	int fd;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Test valid flags first and remove them from the mask */
	/* None at this point */
	invalid_flag_mask = 0xFFFFFFFF;
	nerrs = 0;

	printf("[RUN]\tTest uintr_create_fd invalid flags\n");
	for (bit = 0; bit < sizeof(unsigned int); bit++) {
		flag = invalid_flag_mask & (1 << bit);
		fd = uintr_create_fd(0, flag);
		if (fd >= 0) {
			nerrs++;
			close(fd);
		}
	}

	if (nerrs) {
		printf("[FAIL]\tuintr_create_fd accepted an invalid flag\n");
		total_nerrs += nerrs;
	} else {
		printf("[OK]\tuintr_create_fd returned errors on all invalid flags\n");
	}

	uintr_unregister_handler(0);
}

static void test_register_sender_flags(void)
{
	unsigned int invalid_flag_mask, nerrs, flag, bit;
	int fd;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	fd = uintr_create_fd(0, 0);

	if (fd < 0) {
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Test valid flags first and remove them from the mask */
	/* None at this point */
	invalid_flag_mask = 0xFFFFFFFF;
	nerrs = 0;

	printf("[RUN]\tTest uintr_register_sender invalid flags\n");
	for (bit = 0; bit < sizeof(unsigned int); bit++) {
		flag = invalid_flag_mask & (1 << bit);
		if (uintr_register_sender(fd, flag) >= 0) {
			nerrs++;
			uintr_unregister_sender(fd, 0);
		}
	}

	if (nerrs) {
		printf("[FAIL]\tuintr_register_sender accepted an invalid flag\n");
		total_nerrs += nerrs;
	} else {
		printf("[OK]\tuintr_register_sender returned errors on all invalid flags\n");
	}

	close(fd);
	uintr_unregister_handler(0);
}

static void test_unregister_sender_flags(void)
{
	unsigned int invalid_flag_mask, nerrs, flag, bit;
	int fd;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	fd = uintr_create_fd(0, 0);

	if (fd < 0) {
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	if (uintr_register_sender(fd, 0) < 0) {
		close(fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Test valid flags first and remove them from the mask */
	/* None at this point */
	invalid_flag_mask = 0xFFFFFFFF;
	nerrs = 0;

	printf("[RUN]\tTest uintr_unregister_sender invalid flags\n");
	for (bit = 0; bit < sizeof(unsigned int); bit++) {
		flag = invalid_flag_mask & (1 << bit);
		if (!uintr_unregister_sender(fd, flag)) {
			nerrs++;
			uintr_register_sender(fd, 0);
		}
	}

	if (nerrs) {
		printf("[FAIL]\tuintr_register_sender accepted an invalid flag\n");
		total_nerrs += nerrs;
	} else {
		printf("[OK]\tuintr_register_sender returned errors on all invalid flags\n");
	}

	uintr_unregister_sender(fd, 0);
	close(fd);
	uintr_unregister_handler(0);
}

int main(void)
{
	if (!uintr_supported())
		return EXIT_SUCCESS;

	test_register_handler_flags();

	test_unregister_handler_flags();

	test_create_fd_flags();

	test_register_sender_flags();

	test_unregister_sender_flags();

	return total_nerrs ? EXIT_FAILURE : EXIT_SUCCESS;
}
