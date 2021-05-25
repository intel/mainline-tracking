// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <syscall.h>
#include <unistd.h>
#include <x86gprintrin.h>

#define __NR_uintr_register_handler	450
#define __NR_uintr_unregister_handler	451
#define __NR_uintr_create_fd		452
#define __NR_uintr_register_sender	453
#define __NR_uintr_unregister_sender	454

/* For simiplicity, until glibc support is added */
#define uintr_register_handler(handler, flags)	syscall(__NR_uintr_register_handler, handler, flags)
#define uintr_unregister_handler(flags)		syscall(__NR_uintr_unregister_handler, flags)
#define uintr_create_fd(vector, flags)		syscall(__NR_uintr_create_fd, vector, flags)
#define uintr_register_sender(fd, flags)	syscall(__NR_uintr_register_sender, fd, flags)
#define uintr_unregister_sender(fd, flags)	syscall(__NR_uintr_unregister_sender, fd, flags)

unsigned int uintr_received;
unsigned int uintr_fd;

void __attribute__ ((interrupt)) uintr_handler(struct __uintr_frame *ui_frame,
					       unsigned long long vector)
{
	static const char print[] = "\t-- User Interrupt handler --\n";

	write(STDOUT_FILENO, print, sizeof(print) - 1);
	uintr_received = 1;
}

void *sender_thread(void *arg)
{
	int uipi_index;

	uipi_index = uintr_register_sender(uintr_fd, 0);
	if (uipi_index < 0) {
		printf("Sender register error\n");
		exit(EXIT_FAILURE);
	}

	printf("Sending IPI from sender thread\n");
	_senduipi(uipi_index);

	uintr_unregister_sender(uintr_fd, 0);

	return NULL;
}

int main(int argc, char *argv[])
{
	pthread_t pt;
	int ret;

	if (uintr_register_handler(uintr_handler, 0)) {
		printf("Interrupt handler register error\n");
		exit(EXIT_FAILURE);
	}

	ret = uintr_create_fd(0, 0);
	if (ret < 0) {
		printf("Interrupt vector allocation error\n");
		exit(EXIT_FAILURE);
	}

	uintr_fd = ret;

	_stui();
	printf("Receiver enabled interrupts\n");

	if (pthread_create(&pt, NULL, &sender_thread, NULL)) {
		printf("Error creating sender thread\n");
		exit(EXIT_FAILURE);
	}

	/* Do some other work */
	while (!uintr_received)
		usleep(1);

	pthread_join(pt, NULL);
	close(uintr_fd);
	uintr_unregister_handler(0);

	printf("Success\n");
	exit(EXIT_SUCCESS);
}
