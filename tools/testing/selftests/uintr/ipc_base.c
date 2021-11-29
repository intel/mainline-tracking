// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <x86gprintrin.h>
#include <pthread.h>
#include <signal.h>

#include "uintr_common.h"

unsigned int uintr_received;
unsigned int client_received;
unsigned int server_received;
unsigned int nerrs;

static void __attribute__((interrupt)) uintr_handler(struct __uintr_frame *ui_frame,
						     unsigned long long vector)
{
	uintr_received = 1;
}

static void __attribute__((interrupt)) client_handler(struct __uintr_frame *ui_frame,
						      unsigned long long vector)
{
	client_received = 1;
}

static void __attribute__((interrupt)) server_handler(struct __uintr_frame *ui_frame,
						      unsigned long long vector)
{
	server_received = 1;
}

/* This check doesn't fail. */
static void print_uintr_support(void)
{
	printf("[RUN]\tCheck if User Interrupts (UINTR) is supported\n");
	if (uintr_supported())
		printf("[OK]\tUser Interrupts (UINTR) is supported\n");
	else
		printf("[OK]\tUser Interrupts (UINTR) is not supported. Skipping rest of the tests silently\n");
}

static void *sender_thread(void *arg)
{
	int  uintr_fd = *(int *)arg;
	int uipi_index;

	uipi_index = uintr_register_sender(uintr_fd, 0);
	if (uipi_index < 0) {
		printf("[FAIL]\tSender register error\n");
		return NULL;
	}

	/* Sleep before sending IPI to allow the receiver to start waiting */
	usleep(100);

	printf("\tother thread: sending IPI\n");
	_senduipi(uipi_index);

	uintr_unregister_sender(uintr_fd, 0);

	return NULL;
}

static inline void cpu_delay(void)
{
	long long dl = 1000;
	volatile long long cnt = dl << 10;

	while (cnt--)
		dl++;
}

static void test_thread_ipi(void)
{
	int wait_for_delay = 1000;
	int vector = 0;
	int uintr_fd;
	pthread_t pt;

	/* Register interrupt handler */
	if (uintr_register_handler(uintr_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Create uintr_fd */
	uintr_fd = uintr_create_fd(vector, 0);
	if (uintr_fd < 0) {
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Enable interrupts */
	_stui();

	uintr_received = 0;
	if (pthread_create(&pt, NULL, &sender_thread, (void *)&uintr_fd)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tBase UIPI test: pthreads\n");
	printf("\tSpin in userspace (waiting for interrupts)\n");
	// Keep spinning until interrupt received
	while (wait_for_delay-- && !uintr_received)
		cpu_delay();

	if (uintr_received) {
		printf("[OK]\tUser interrupt received\n");
	} else {
		printf("[FAIL]\tUser interrupt not received\n");
		nerrs++;
	}

	pthread_join(pt, NULL);
	close(uintr_fd);
	uintr_unregister_handler(0);
}

static void test_blocking_ipi(void)
{
	int ret, uintr_fd;
	int vector = 0;
	pthread_t pt;

	/* Register interrupt handler */
	if (uintr_register_handler(uintr_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Create uintr_fd */
	uintr_fd = uintr_create_fd(vector, 0);
	if (uintr_fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Enable interrupts */
	_stui();

	uintr_received = 0;
	if (pthread_create(&pt, NULL, &sender_thread, (void *)&uintr_fd)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tBase UIPI test: pthreads\n");
	printf("\tBlock in the kernel (waiting for interrupts)\n");
	uintr_wait(0);
	if (uintr_received) {
		printf("[OK]\tUser interrupt received\n");
	} else {
		printf("[FAIL]\tUser interrupt not received\n");
		nerrs++;
	}

	pthread_join(pt, NULL);
	close(uintr_fd);
	uintr_unregister_handler(0);
}

static void sender_process_uni(int uintr_fd)
{
	int uipi_index;

	uipi_index = uintr_register_sender(uintr_fd, 0);
	if (uipi_index < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		exit(EXIT_FAILURE);
	}

	_senduipi(uipi_index);

	uintr_unregister_sender(uintr_fd, 0);
	/* Close sender copy of uintr_fd */
	close(uintr_fd);

	pause();
}

static void test_process_ipi_unidirectional(void)
{
	int wait_for_usec = 100;
	int uintr_fd, pid;

	if (uintr_register_handler(uintr_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	uintr_fd = uintr_create_fd(0, 0);
	if (uintr_fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	uintr_received = 0;
	_stui();

	printf("[RUN]\tBase User IPI test: Process uni-directional\n");

	pid = fork();
	if (pid == 0) {
		/* Child sender process */
		sender_process_uni(uintr_fd);
		exit(EXIT_SUCCESS);
	}

	while (wait_for_usec-- && !uintr_received)
		usleep(1);

	close(uintr_fd);
	uintr_unregister_handler(0);
	kill(pid, SIGKILL);

	if (!uintr_received) {
		printf("[FAIL]\tUser interrupt not received\n");
		nerrs++;
	} else {
		printf("[OK]\tUser interrupt received\n");
	}
}

static void client_process_bi(int server_fd, int sock)
{
	int uipi_index;
	int client_fd;
	ssize_t size;

	uipi_index = uintr_register_sender(server_fd, 0);
	if (uipi_index < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		exit(EXIT_FAILURE);
	}

	if (uintr_register_handler(client_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Create uintr_fd with vector 1 */
	client_fd = uintr_create_fd(1, 0);
	if (client_fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}
	_stui();

	// Share client_fd
	if (sock_fd_write(sock, "1", 1, client_fd) != 1) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	while (!client_received)
		usleep(1);

	_senduipi(uipi_index);

	pause();
}

static void test_process_ipi_bidirectional(void)
{
	int server_fd, client_fd, pid;
	int sv[2];
	ssize_t size;
	char buf[16];
	int uipi_index;

	if (socketpair(AF_LOCAL, SOCK_STREAM, 0, sv) < 0)  {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	if (uintr_register_handler(server_handler, 0)) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	/* Create uintr_fd with vector 0 */
	server_fd = uintr_create_fd(0, 0);
	if (server_fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	_stui();

	printf("[RUN]\tBase User IPI test: Process bi-directional\n");

	pid = fork();
	if (pid == 0) {
		/* Child client process */
		close(sv[0]);
		client_process_bi(server_fd, sv[1]);
		exit(EXIT_SUCCESS);
	}

	close(sv[1]);
	if (sock_fd_read(sv[0], buf, sizeof(buf), &client_fd) != 1) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	//Register server as sender
	uipi_index = uintr_register_sender(client_fd, 0);
	if (uipi_index < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	_senduipi(uipi_index);

	while (!server_received)
		usleep(1);

	//close(uintr_fd);
	uintr_unregister_handler(0);
	kill(pid, SIGKILL);

	if (!server_received) {
		printf("[FAIL]\tUser interrupt not received\n");
		nerrs++;
	} else {
		printf("[OK]\tUser interrupt received\n");
	}
}

/* TODO: Use some better method for failure rather than the 45sec KSFT timeout */
int main(int argc, char *argv[])
{
	print_uintr_support();

	if (!uintr_supported())
		return EXIT_SUCCESS;

	test_thread_ipi();

	test_blocking_ipi();

	test_process_ipi_unidirectional();

	test_process_ipi_bidirectional();

	return nerrs ? EXIT_FAILURE : EXIT_SUCCESS;
}
