// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <syscall.h>
#include <stdio.h>
#include <unistd.h>
#include <x86gprintrin.h>
#include <pthread.h>
#include <stdlib.h>

#ifndef __x86_64__
# error This test is 64-bit only
#endif

#ifndef __NR_uintr_register_handler
#define __NR_uintr_register_handler	449
#define __NR_uintr_unregister_handler	450
#define __NR_uintr_create_fd		451
#define __NR_uintr_register_sender	452
#define __NR_uintr_unregister_sender	453
#define __NR_uintr_wait			454
#endif

#define uintr_register_handler(handler, flags)	syscall(__NR_uintr_register_handler, handler, flags)
#define uintr_unregister_handler(flags)		syscall(__NR_uintr_unregister_handler, flags)
#define uintr_create_fd(vector, flags)		syscall(__NR_uintr_create_fd, vector, flags)
#define uintr_register_sender(fd, flags)	syscall(__NR_uintr_register_sender, fd, flags)
#define uintr_unregister_sender(fd, flags)	syscall(__NR_uintr_unregister_sender, fd, flags)
#define uintr_wait(flags)			syscall(__NR_uintr_wait, flags)

unsigned long uintr_received;
unsigned int uintr_fd;

void __attribute__((interrupt))__attribute__((target("general-regs-only", "inline-all-stringops")))
uintr_handler(struct __uintr_frame *ui_frame,
	      unsigned long long vector)
{
	uintr_received = 1;
}

void receiver_setup_interrupt(void)
{
	int vector = 0;
	int ret;

	/* Register interrupt handler */
	if (uintr_register_handler(uintr_handler, 0)) {
		printf("[FAIL]\tInterrupt handler register error\n");
		exit(EXIT_FAILURE);
	}

	/* Create uintr_fd */
	ret = uintr_create_fd(vector, 0);
	if (ret < 0) {
		printf("[FAIL]\tInterrupt vector registration error\n");
		exit(EXIT_FAILURE);
	}

	uintr_fd = ret;
}

void *sender_thread(void *arg)
{
	long sleep_usec = (long)arg;
	int uipi_index;

	uipi_index = uintr_register_sender(uintr_fd, 0);
	if (uipi_index < 0) {
		printf("[FAIL]\tSender register error\n");
		return NULL;
	}

	/* Sleep before sending IPI to allow the receiver to block in the kernel */
	if (sleep_usec)
		usleep(sleep_usec);

	printf("\tother thread: sending IPI\n");
	_senduipi(uipi_index);

	uintr_unregister_sender(uintr_fd, 0);

	return NULL;
}

static inline void cpu_relax(void)
{
	asm volatile("rep; nop" ::: "memory");
}

void test_base_ipi(void)
{
	pthread_t pt;

	uintr_received = 0;
	if (pthread_create(&pt, NULL, &sender_thread, NULL)) {
		printf("[FAIL]\tError creating sender thread\n");
		return;
	}

	printf("[RUN]\tSpin in userspace (waiting for interrupts)\n");
	// Keep spinning until interrupt received
	while (!uintr_received)
		cpu_relax();

	printf("[OK]\tUser interrupt received\n");
}

void test_blocking_ipi(void)
{
	pthread_t pt;
	long sleep_usec;

	uintr_received = 0;
	sleep_usec = 1000;
	if (pthread_create(&pt, NULL, &sender_thread, (void *)sleep_usec)) {
		printf("[FAIL]\tError creating sender thread\n");
		return;
	}

	printf("[RUN]\tBlock in the kernel (waiting for interrupts)\n");
	uintr_wait(0);
	if (uintr_received)
		printf("[OK]\tUser interrupt received\n");
	else
		printf("[FAIL]\tUser interrupt not received\n");
}

int main(int argc, char *argv[])
{
	receiver_setup_interrupt();

	/* Enable interrupts */
	_stui();

	test_base_ipi();

	test_blocking_ipi();

	close(uintr_fd);
	uintr_unregister_handler(0);

	exit(EXIT_SUCCESS);
}
