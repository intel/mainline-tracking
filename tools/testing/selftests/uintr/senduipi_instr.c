// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <x86gprintrin.h>
#include <signal.h>
#include <sys/ucontext.h>
#include <string.h>
#include <err.h>
#include <stdbool.h>
#include <pthread.h>

#include "uintr_common.h"

static volatile sig_atomic_t expect_sigill;
static volatile sig_atomic_t expect_sigsegv;

static void set_handler(int sig, void (*handler)(int, siginfo_t *, void *), int flags)
{
	struct sigaction sa;

	memset(&sa, 0, sizeof(sa));
	sa.sa_sigaction = handler;
	sa.sa_flags = SA_SIGINFO | flags;
	sigemptyset(&sa.sa_mask);
	if (sigaction(sig, &sa, 0))
		err(1, "sigaction");
}

static void clear_handler(int sig)
{
	struct sigaction sa;

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = SIG_DFL;
	sigemptyset(&sa.sa_mask);
	if (sigaction(sig, &sa, 0))
		err(1, "sigaction");
}

static void sigsegv(int sig, siginfo_t *si, void *ctx_void)
{
	ucontext_t *ctx = (ucontext_t *)ctx_void;

	if (!expect_sigsegv) {
		clear_handler(SIGSEGV);
		return;
	}

	expect_sigsegv = false;

	/* skip senduipi */
	ctx->uc_mcontext.gregs[REG_RIP] += 4;
}

static void sigill(int sig, siginfo_t *si, void *ctx_void)
{
	ucontext_t *ctx = (ucontext_t *)ctx_void;

	if (!expect_sigill) {
		clear_handler(SIGILL);
		return;
	}

	expect_sigill = false;

	/* skip senduipi */
	ctx->uc_mcontext.gregs[REG_RIP] += 4;
}

static void test_no_registration(void)
{
	set_handler(SIGILL, sigill, 0);
	expect_sigill = true;

	printf("[RUN]\tSENDUIPI without registration\n");
	_senduipi(0);
	clear_handler(SIGILL);

	if (expect_sigill)
		printf("[FAIL]\tDidn't receive SIGILL as expected\n");
	else
		printf("[OK]\tSIGILL generated\n");
}

static void test_invalid_handle(void)
{
	int fd, uipi_handle;

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

	uipi_handle = uintr_register_sender(fd, 0);
	if (uipi_handle < 0) {
		close(fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	set_handler(SIGSEGV, sigsegv, 0);
	expect_sigsegv = true;

	printf("[RUN]\tSENDUIPI with invalid handle\n");
	_senduipi(uipi_handle + 1);
	clear_handler(SIGSEGV);

	if (expect_sigsegv)
		printf("[FAIL]\tDidn't receive SIGSEGV as expected\n");
	else
		printf("[OK]\tSIGSEGV generated\n");

	close(fd);
	uintr_unregister_handler(0);
}

static void test_after_unregister(void)
{
	int fd, uipi_handle;

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

	uipi_handle = uintr_register_sender(fd, 0);
	if (uipi_handle < 0) {
		close(fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	set_handler(SIGILL, sigill, 0);
	expect_sigill = true;

	printf("[RUN]\tSENDUIPI after unregister sender\n");

	uintr_unregister_sender(fd, 0);
	_senduipi(uipi_handle);
	clear_handler(SIGILL);

	if (expect_sigill)
		printf("[FAIL]\tDidn't receive SIGILL as expected\n");
	else
		printf("[OK]\tSIGILL generated\n");

	close(fd);
	uintr_unregister_handler(0);
}

static void test_after_closefd(void)
{
	int fd, uipi_handle;

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

	uipi_handle = uintr_register_sender(fd, 0);
	if (uipi_handle < 0) {
		close(fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	set_handler(SIGILL, sigill, 0);
	expect_sigill = true;

	printf("[RUN]\tSENDUIPI after close fd\n");

	close(fd);
	_senduipi(uipi_handle);
	clear_handler(SIGILL);

	if (expect_sigill)
		printf("[FAIL]\tDidn't receive SIGILL as expected\n");
	else
		printf("[OK]\tSIGILL generated\n");

	uintr_unregister_handler(0);
}

static void *sender_thread(void *arg)
{
	int uipi_handle = *(int *)arg;

	set_handler(SIGILL, sigill, 0);
	expect_sigill = true;

	printf("[RUN]\tSENDUIPI on another thread\n");

	_senduipi(uipi_handle);
	clear_handler(SIGILL);

	if (expect_sigill)
		printf("[FAIL]\tDidn't receive SIGILL as expected\n");
	else
		printf("[OK]\tSIGILL generated\n");

	return NULL;
}

static void test_separate_thread(void)
{
	int fd, uipi_handle;
	pthread_t pt;

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

	uipi_handle = uintr_register_sender(fd, 0);
	if (uipi_handle < 0) {
		close(fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	if (pthread_create(&pt, NULL, &sender_thread, (void *)&uipi_handle)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	pthread_join(pt, NULL);

	close(fd);
	uintr_unregister_handler(0);
}

int main(int argc, char *argv[])
{
	if (!uintr_supported())
		return EXIT_SUCCESS;

	test_no_registration();

	test_invalid_handle();

	test_after_unregister();

	test_after_closefd();

	test_separate_thread();

	exit(EXIT_SUCCESS);
}
