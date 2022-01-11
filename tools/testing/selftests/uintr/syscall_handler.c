// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <sys/random.h>
#include <sys/wait.h>
#include <x86gprintrin.h>

#include "uintr_common.h"

int uintr_received;
int nerrs;

static void test_handler_unregister(void)
{
	printf("[RUN]\tUnregister Handler: Unregister without any registration\n");
	if (uintr_unregister_handler(0))
		printf("[OK]\tUnregister Handler: Error no handler present\n");
	else
		printf("[FAIL]\tUnregister Handler: Didn't throw the expected error\n");
}

static void test_handler_already_registered(void)
{
	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tRegister Handler: Try registering twice\n");

	if (uintr_register_handler(uintr_empty_handler, 0)) {
		printf("[OK]\tRegister Handler: Error on second registration\n");
	} else {
		printf("[FAIL]\tRegister Handler: Didn't fail on second registration\n");
		nerrs++;
	}

	uintr_unregister_handler(0);
}

static void test_incorrect_handler(void)
{
	unsigned long long addr;

	printf("[RUN]\tRegister Handler: Invalid address\n");
	if (uintr_register_handler(0xabcd, 0)) {
		printf("[OK]\tRegister Handler: error\n");
	} else {
		printf("[FAIL]\tRegister Handler: Didn't throw the expected error\n");
		nerrs++;
	}
	uintr_unregister_handler(0);

	printf("[RUN]\tRegister Handler: non-canonical address\n");
	if (uintr_register_handler(0x0f0000000000abcd, 0)) {
		printf("[OK]\tRegister Handler: error\n");
	} else {
		printf("[FAIL]\tRegister Handler: Didn't throw the expected error\n");
		nerrs++;
	}
	uintr_unregister_handler(0);

	printf("[RUN]\tRegister Handler: kernel address\n");
	if (uintr_register_handler(0xffffffff00000000, 0)) {
		printf("[OK]\tHandler register error\n");
	} else {
		printf("[FAIL]\tRegister Handler: Didn't throw the expected error\n");
		nerrs++;
	}
	uintr_unregister_handler(0);

	getrandom(&addr, sizeof(addr), 0);
	printf("[RUN]\tRegister Handler: Random address\n");
	if (uintr_register_handler(addr, 0)) {
		printf("[OK]\tHandler register error\n");
	} else {
		printf("[FAIL]\tRegister Handler: Didn't throw the expected error\n");
		nerrs++;
	}
	uintr_unregister_handler(0);
}

static void usual_function(struct __uintr_frame *ui_frame, unsigned long long vector)
{
	uintr_received++;
}

static void sigsegv(int sig)
{
	printf("[OK]\tRegister Handler: SIGSEGV received\n");
	exit(EXIT_SUCCESS);
}

static void test_regular_function_as_handler(void)
{
	int uintr_fd, uipi_index;

	if (uintr_register_handler(usual_function, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	uintr_fd = uintr_create_fd(0, 0);
	if (uintr_fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	_stui();

	uipi_index = uintr_register_sender(uintr_fd, 0);
	if (uipi_index < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tRegister Handler: Regular function as handler\n");

	signal(SIGSEGV, sigsegv);

	_senduipi(uipi_index);

	while (!uintr_received)
		usleep(1);

	printf("[FAIL]\tRegister Handler: Error signal not generated\n");
	exit(EXIT_FAILURE);
}

int main(void)
{
	if (!uintr_supported())
		return EXIT_SUCCESS;

	test_handler_unregister();

	test_handler_already_registered();

	/* Expected to fail as kernel checks are not implemented yet */
	//test_incorrect_handler();

	/* Run this test in a separate process to avoid state corruption */
	if (fork() == 0)
		test_regular_function_as_handler();

	wait(NULL);

	return nerrs ? EXIT_FAILURE : EXIT_SUCCESS;
}
