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
#include <sys/sysinfo.h>
#include <sys/wait.h>

#include "uintr_common.h"

unsigned int client_received;
unsigned int server_received;
unsigned int count = 100000;
unsigned int nprocs;
unsigned int start_cpu;

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

static void wait_for_interrupt(unsigned int *interrupt_received_flag)
{
	/* TODO: Add alternate to test with usleep() */

	while (!(*interrupt_received_flag))
		cpu_relax();
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

	/* Share client_fd */
	if (sock_fd_write(sock, "1", 1, client_fd) != 1) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	do {
		wait_for_interrupt(&client_received);
		client_received = 0;
		_senduipi(uipi_index);
	} while (1);
}

static void test_process_ipi_bidirectional(int i)
{
	int server_fd, client_fd, uipi_index, pid;
	char buf[16];
	ssize_t size;
	int sv[2];

	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET((start_cpu + (i % nprocs)) % get_nprocs(), &cpuset);
	if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

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

	printf("\tStarting Server-Client pair: %d\n", i);

	pid = fork();
	if (pid == 0) {
		/* Child client process */
		close(sv[0]);

		CPU_ZERO(&cpuset);
		CPU_SET((start_cpu + ((i + 1) % nprocs)) % get_nprocs(), &cpuset);
		if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
			printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
			return;
		}

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

	do {
		server_received = 0;
		_senduipi(uipi_index);
		wait_for_interrupt(&server_received);
	} while (count--);

	uintr_unregister_handler(0);

	kill(pid, SIGKILL);

	if (server_received)
		printf("[OK]\tAll interrupts received: pair: %d\n", i);
	else
		printf("[FAIL]\tInterrupt not received: pair: %d\n", i);

	exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
	int num_forks, i, c, pairs;
	int *pid = NULL;

	if (!uintr_supported())
		return EXIT_SUCCESS;

	nprocs = get_nprocs();
	pairs = nprocs * 2;

	srand(time(NULL));
	start_cpu = rand() % get_nprocs();

	while ((c = getopt(argc, argv, "c:i:p:")) != EOF) {
		switch (c) {
		case 'c':
			nprocs = atoi(optarg);
			if (nprocs > get_nprocs())
				nprocs = get_nprocs();
			break;

		case 'i':
			count = atoi(optarg);
			break;

		case 'p':
			pairs = atoi(optarg);
			break;

		default:
			break;
		}
	}

	printf("[RUN]\tContext switch IPC test\n");

	pid = malloc(sizeof(int) * pairs);
	if (!pid) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return EXIT_SUCCESS;
	}

	printf("\tServer-Client pairs:%d Number of cpus:%d Iterations per pair:%d\n",
	       pairs, nprocs, count);

	for (i = 0; i < pairs; i++) {
		pid[i] = fork();
		if (pid[i] == 0) {
			test_process_ipi_bidirectional(i);
			break;
		}
	}

	for (i = 0; i < pairs; i++)
		waitpid(pid[i], NULL, 0);

	return EXIT_SUCCESS;
}
