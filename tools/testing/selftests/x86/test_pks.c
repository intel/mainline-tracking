// SPDX-License-Identifier: GPL-2.0-only

#define _GNU_SOURCE
#include <sched.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdbool.h>

#define PKS_TEST_FILE "/sys/kernel/debug/x86/run_pks"

#define RUN_ALLOCATE            "0"
#define SETUP_CTX_SWITCH        "1"
#define CHECK_CTX_SWITCH        "2"
#define RUN_ALLOCATE_ALL        "3"
#define RUN_ALLOCATE_DEBUG      "4"
#define RUN_ALLOCATE_ALL_DEBUG  "5"
#define RUN_CRASH_TEST          "9"

int main(int argc, char *argv[])
{
	cpu_set_t cpuset;
	char result[32];
	pid_t pid;
	int fd;
	int setup_done[2];
	int switch_done[2];
	int cpu = 0;
	int rc = 0;
	int c;
	bool debug = false;

	while (1) {
		int option_index = 0;
		static struct option long_options[] = {
			{"debug",  no_argument,	  0,  0 },
			{0,	   0,		  0,  0 }
		};

		c = getopt_long(argc, argv, "", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 0:
			debug = true;
			break;
		}
	}

	if (optind < argc)
		cpu = strtoul(argv[optind], NULL, 0);

	if (cpu >= sysconf(_SC_NPROCESSORS_ONLN)) {
		printf("CPU %d is invalid\n", cpu);
		cpu = sysconf(_SC_NPROCESSORS_ONLN) - 1;
		printf("   running on max CPU: %d\n", cpu);
	}

	CPU_ZERO(&cpuset);
	CPU_SET(cpu, &cpuset);
	/* Two processes run on CPU 0 so that they go through context switch. */
	sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpuset);

	if (pipe(setup_done))
		printf("Failed to create pipe\n");
	if (pipe(switch_done))
		printf("Failed to create pipe\n");

	pid = fork();
	if (pid == 0) {
		char done = 'y';

		fd = open(PKS_TEST_FILE, O_RDWR);
		if (fd < 0) {
			printf("cannot open %s\n", PKS_TEST_FILE);
			return -1;
		}

		cpu = sched_getcpu();
		printf("Child running on cpu %d...\n", cpu);

		/* Allocate test_pkey1 and run test. */
		if (debug)
			write(fd, RUN_ALLOCATE_DEBUG, 1);
		else
			write(fd, RUN_ALLOCATE, 1);

		/* Arm for context switch test */
		write(fd, SETUP_CTX_SWITCH, 1);

		printf("   tell parent to go\n");
		write(setup_done[1], &done, sizeof(done));

		/* Context switch out... */
		printf("   Waiting for parent...\n");
		read(switch_done[0], &done, sizeof(done));

		/* Check msr restored */
		printf("Checking result\n");
		write(fd, CHECK_CTX_SWITCH, 1);

		read(fd, result, 10);
		printf("   #PF, context switch, pkey allocation and free tests: %s\n", result);
		if (!strncmp(result, "PASS", 10)) {
			rc = -1;
			done = 'F';
		}

		/* Signal result */
		write(setup_done[1], &done, sizeof(done));
	} else {
		char done = 'y';

		read(setup_done[0], &done, sizeof(done));
		cpu = sched_getcpu();
		printf("Parent running on cpu %d\n", cpu);

		fd = open(PKS_TEST_FILE, O_RDWR);
		if (fd < 0) {
			printf("cannot open %s\n", PKS_TEST_FILE);
			return -1;
		}

		/* run test with alternate pkey */
		if (debug)
			write(fd, RUN_ALLOCATE_DEBUG, 1);
		else
			write(fd, RUN_ALLOCATE, 1);

		/* Signal child we are done.  */
		printf("   Telling child we are done.\n");
		write(switch_done[1], &done, sizeof(done));

		/* Wait for result */
		read(setup_done[0], &done, sizeof(done));
		if (done == 'F')
			rc = -1;
	}

	close(fd);

	return rc;
}
