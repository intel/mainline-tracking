// SPDX-License-Identifier: GPL-2.0-only
/* Quick tests to verify Shadow Stack and IBT are working */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <ucontext.h>

ucontext_t ucp;

enum {
	r_not_tested = -1,
	r_fail = 0,
	r_ok = 1,
};

int result[4] = {r_not_tested, r_not_tested, r_not_tested, r_not_tested};
int test_id;

/* Defined in include/uapi/asm-generic/siginfo.h */
#define SEGV_CPERR 8

/*
 * Do an indirect jmp to a location without endbr to trigger a control
 * protection fault.  Verify in segv_handler that ibt is working.
 */
void ibt_violation(void)
{
	void *ptr;

	asm volatile("lea 1f, %0\n\t"
		     "jmp *%0\n\t"
		     "1:" : "=r"(ptr));

	result[test_id] = r_fail;
	test_id++;
	setcontext(&ucp);
}

/*
 * Do a push and ret to cause shadow stack mismatch and trigger a control
 * protection fault.  Verify in segv_handler that shadow stack is working.
 */
void shstk_violation(void)
{
	void *ptr;

	asm volatile("lea 1f, %0\n\t"
		     "push %0\n\t"
		     "ret\n\t"
		     "1:" : "=r"(ptr));

	result[test_id] = r_fail;
	test_id++;
	setcontext(&ucp);
}

void segv_handler(int signum, siginfo_t *si, void *uc)
{
	if (si->si_code == SEGV_CPERR) {
		result[test_id] = r_ok;
		test_id++;
	} else {
		printf("Unexpected seg fault\n");
		exit(1);
	}

	setcontext(&ucp);
}

/*
 * Verify shadow stack and ibt are working in a signal handler.
 */
void user1_handler(int signum, siginfo_t *si, void *uc)
{
	if (test_id == 2)
		shstk_violation();

	if (test_id == 3)
		ibt_violation();
}

int main(int argc, char *argv[])
{
	struct sigaction sa;
	int r;

	r = sigemptyset(&sa.sa_mask);
	if (r)
		return -1;

	sa.sa_flags = SA_SIGINFO;

	/*
	 * Control protection fault handler
	 */
	sa.sa_sigaction = segv_handler;
	r = sigaction(SIGSEGV, &sa, NULL);
	if (r)
		return -1;

	/*
	 * Test shadow stack/ibt in signal handler
	 */
	sa.sa_sigaction = user1_handler;
	r = sigaction(SIGUSR1, &sa, NULL);
	if (r)
		return -1;

	test_id = 0;

	/*
	 * Pass or fail, each test returns here with test_id incremented to
	 * the next test.
	 */
	r = getcontext(&ucp);
	if (r)
		return -1;

	if (test_id == 0)
		shstk_violation();
	else if (test_id == 1)
		ibt_violation();
	else if (test_id == 2)
		raise(SIGUSR1);
	else if (test_id == 3)
		raise(SIGUSR1);

	r = 0;
	printf("[%s]\tShadow stack\n", result[0] == -1 ? "untested" :
	       (result[0] ? "OK" : "FAIL"));
	r += result[0];

	printf("[%s]\tIBT\n", result[1] == -1 ? "untested" :
	       (result[1] ? "OK" : "FAIL"));
	r += result[1];

	printf("[%s]\tShadow stack in signal\n", result[2] == -1 ? "untested" :
	       (result[2] ? "OK" : "FAIL"));
	r += result[2];

	printf("[%s]\tIBT in signal\n", result[3] == -1 ? "untested" :
	       (result[3] ? "OK" : "FAIL"));
	r += result[3];

	return r;
}
