// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <x86gprintrin.h>
#include <sys/eventfd.h>
#include <pthread.h>
#include <sys/wait.h>

#include "uintr_common.h"

unsigned int nerrs;
int uintr_fd[256];

static void test_no_uintr_fd(void)
{
	int fd = -1;

	printf("[RUN]\tRegister without uintr_fd\n");
	if (uintr_register_sender(fd, 0) < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_register_sender didn't throw any error\n");
		nerrs++;
	}
}

static void test_invalid_uintr_fd(void)
{
	int fd = eventfd(0, 0);

	if (fd < 0) {
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	printf("[RUN]\tRegister with invalid uintr_fd\n");
	if (uintr_register_sender(fd, 0) < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_register_sender didn't throw any error\n");
		nerrs++;
	}
}

/*
 * This test case would need to change if we decide to return the same
 * uipi_index back when the same uintr_fd is registered for multi-threaded
 * usages.
 */
static void test_registering_uintrfd_again(void)
{
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

	printf("[RUN]\tRegister with the same uintr_fd again\n");
	if (uintr_register_sender(fd, 0) < 0) {
		close(fd);
		uintr_unregister_handler(0);
		printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
		return;
	}

	if (uintr_register_sender(fd, 0) < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_register_sender didn't throw any error\n");
		nerrs++;
	}

	close(fd);
	uintr_unregister_handler(0);
}

/* Threads needed to max out UITT size with 64 vectors per thread */
#define NUM_THREADS 4

pthread_cond_t test_done = PTHREAD_COND_INITIALIZER;
pthread_mutex_t test_done_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_barrier_t init_barrier;

static void *sender_thread(void *arg)
{
	int voffset = (*(int *)arg) * 64;
	int i, fd;

	if (uintr_register_handler(uintr_empty_handler, 0))
		pthread_exit(NULL);

	for (i = 0; i < 64; i++) {
		fd = uintr_create_fd(i, 0);
		if (fd < 0) {
			uintr_unregister_handler(0);
			pthread_exit(NULL);
		}

		uintr_fd[voffset + i] = fd;
	}

	pthread_barrier_wait(&init_barrier);

	pthread_mutex_lock(&test_done_mtx);
	pthread_cond_wait(&test_done, &test_done_mtx);
	pthread_mutex_unlock(&test_done_mtx);
}

/*
 * Currently, the UITT size is hardcoded as 256. This test would need to change
 * when the UITT is made dynamic in size.
 */
static void test_out_of_uitte(void)
{
	pthread_t pt[NUM_THREADS];
	int thread[NUM_THREADS];
	int fd, i;

	pthread_barrier_init(&init_barrier, NULL, NUM_THREADS + 1);

	for (i = 0; i < 4; i++) {
		thread[i] = i;
		if (pthread_create(&pt[i], NULL, &sender_thread, (void *)&thread[i]))
			goto skip;
	}

	/* Wait for all threads to finish their initialization */
	/* TODO: Avoid infinite waiting when one of the thread exits before reaching the barrier */
	pthread_barrier_wait(&init_barrier);

	for (i = 0; i < 256; i++) {
		if (uintr_register_sender(uintr_fd[i], 0) < 0)
			goto skip;
	}

	/*
	 * Register this thread itself as a sender for the 257th entry in the
	 * UITT table that is expected to fail.
	 */
	if (uintr_register_handler(uintr_empty_handler, 0))
		goto skip;

	fd = uintr_create_fd(0, 0);

	if (fd < 0)
		goto skip;

	printf("[RUN]\tMax UITT entries exceeded\n");
	if (uintr_register_sender(fd, 0) < 0) {
		printf("[OK]\tError returned\n");
	} else {
		printf("[FAIL]\tuintr_register_sender didn't throw any error\n");
		nerrs++;
	}

	goto exit;

skip:
	printf("[SKIP]\t%s:%d\n", __func__, __LINE__);
exit:

	/*
	 * All FDs and UINTR state will be automatically cleared by the kernel
	 * upon process exit.
	 */
	pthread_mutex_lock(&test_done_mtx);
	pthread_cond_broadcast(&test_done);
	pthread_mutex_unlock(&test_done_mtx);

	if (nerrs)
		exit(EXIT_FAILURE);
	else
		exit(EXIT_SUCCESS);
}

int main(void)
{
	if (!uintr_supported())
		return EXIT_SUCCESS;

	test_no_uintr_fd();

	test_invalid_uintr_fd();

	test_registering_uintrfd_again();

	//test_receiver_has_exited();

	/* Run this test in a separate process to ease state cleanup */
	if (fork() == 0)
		test_out_of_uitte();

	wait(NULL);

	return nerrs ? EXIT_FAILURE : EXIT_SUCCESS;
}
