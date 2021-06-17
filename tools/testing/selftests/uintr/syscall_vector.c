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

unsigned int nerrs;

static void test_no_handler(void)
{
	int fd;

	/* Make sure no handler is registered */
	uintr_unregister_handler(0);

	printf("[RUN]\tTest uintr_create_fd without any handler registered\n");

	fd = uintr_create_fd(0, 0);
	if (fd < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_create_fd didn't throw any error\n");
		close(fd);
		nerrs++;
	}
}

static void test_register_same_vector(void)
{
	int fd, fd2;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Register vector 0 */
	fd = uintr_create_fd(0, 0);
	if (fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tTest uintr_create_fd with same vector\n");
	/* Register vector 0 again */
	fd2 = uintr_create_fd(0, 0);
	if (fd2 < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_create_fd didn't throw any error\n");
		close(fd2);
		nerrs++;
	}

	close(fd);
	uintr_unregister_handler(0);
}

static void test_vector(int vector)
{
	int fd;

	fd = uintr_create_fd(vector, 0);
	if (fd < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_create_fd didn't throw any error for vector %d\n", vector);
		close(fd);
		nerrs++;
	}
}

static void test_vectors_beyond_range(void)
{
	int vector, fd;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tTest uintr_create_fd with invalid vector -1\n");
	test_vector(-1);

	printf("[RUN]\tTest uintr_create_fd with invalid vector 64\n");
	test_vector(64);

	printf("[RUN]\tTest uintr_create_fd with invalid vector 100\n");
	test_vector(100);

	uintr_unregister_handler(0);
}

static void test_fd_dup(void)
{
	int orig_fd, dup_fd, fd, vector;

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	vector = 0;
	orig_fd = uintr_create_fd(vector, 0);
	if (orig_fd < 0) {
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	dup_fd = dup(orig_fd);
	if (dup_fd < 0) {
		close(orig_fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tDuplicate and close one of the FDs and try re-registering\n");

	close(orig_fd);
	fd = uintr_create_fd(vector, 0);
	if (fd < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_create_fd didn't throw any error during re-registeration\n");
		close(fd);
		nerrs++;
	}

	close(dup_fd);
	uintr_unregister_handler(0);
}

int main(void)
{
	if (!uintr_supported())
		return EXIT_SUCCESS;

	test_no_handler();

	test_register_same_vector();

	test_vectors_beyond_range();

	test_fd_dup();

	return nerrs ? EXIT_FAILURE : EXIT_SUCCESS;
}
