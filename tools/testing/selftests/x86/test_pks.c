// SPDX-License-Identifier: GPL-2.0-only

#define _GNU_SOURCE
#include <sched.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void)
{
	cpu_set_t cpuset;
	char result[32];
	pid_t pid;
	int fd;

	CPU_ZERO(&cpuset);
	CPU_SET(0, &cpuset);
	/* Two processes run on CPU 0 so that they go through context switch. */
	sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpuset);

	pid = fork();
	if (pid == 0) {
		fd = open("/sys/kernel/debug/x86/run_pks", O_RDWR);
		if (fd < 0) {
			printf("cannot open file\n");
			return -1;
		}

		/* Allocate test_pkey1 and run test. */
		write(fd, "0", 1);

		/* Arm for context switch test */
		write(fd, "1", 1);

		/* Context switch out... */
		sleep(4);

		/* Check msr restored */
		write(fd, "2", 1);
	} else {
		sleep(2);

		fd = open("/sys/kernel/debug/x86/run_pks", O_RDWR);
		if (fd < 0) {
			printf("cannot open file\n");
			return -1;
		}

		/* run test with alternate pkey */
		write(fd, "0", 1);
	}

	read(fd, result, 10);
	printf("#PF, context switch, pkey allocation and free tests: %s\n",
	       result);

	close(fd);

	return 0;
}
