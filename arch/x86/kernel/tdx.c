// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2020 Intel Corporation */

#undef pr_fmt
#define pr_fmt(fmt)     "x86/tdx: " fmt

#include <linux/protected_guest.h>

#include <asm/tdx.h>

/*
 * Wrapper for simple hypercalls that only return a success/error code.
 */
static inline u64 tdx_hypercall(u64 fn, u64 r12, u64 r13, u64 r14, u64 r15)
{
	u64 err;

	err = __tdx_hypercall(fn, r12, r13, r14, r15, NULL);

	if (err)
		pr_warn_ratelimited("TDVMCALL fn:%llx failed with err:%llx\n",
				    fn, err);

	return err;
}

/*
 * Wrapper for the semi-common case where user need single output
 * value (R11). Callers of this function does not care about the
 * hypercall error code (mainly for IN or MMIO usecase).
 */
static inline u64 tdx_hypercall_out_r11(u64 fn, u64 r12, u64 r13,
					u64 r14, u64 r15)
{
	struct tdx_hypercall_output out = {0};
	u64 err;

	err = __tdx_hypercall(fn, r12, r13, r14, r15, &out);

	if (err)
		pr_warn_ratelimited("TDVMCALL fn:%llx failed with err:%llx\n",
				    fn, err);

	return out.r11;
}

static inline bool cpuid_has_tdx_guest(void)
{
	u32 eax, sig[3];

	if (cpuid_eax(0) < TDX_CPUID_LEAF_ID)
		return false;

	cpuid_count(TDX_CPUID_LEAF_ID, 0, &eax, &sig[0], &sig[1], &sig[2]);

	return !memcmp("IntelTDX    ", sig, 12);
}

bool tdx_protected_guest_has(unsigned long flag)
{
	switch (flag) {
	case PR_GUEST_MEM_ENCRYPT:
	case PR_GUEST_MEM_ENCRYPT_ACTIVE:
	case PR_GUEST_UNROLL_STRING_IO:
	case PR_GUEST_SHARED_MAPPING_INIT:
	case PR_GUEST_TDX:
		return static_cpu_has(X86_FEATURE_TDX_GUEST);
	}

	return false;
}
EXPORT_SYMBOL_GPL(tdx_protected_guest_has);

void __init tdx_early_init(void)
{
	if (!cpuid_has_tdx_guest())
		return;

	setup_force_cpu_cap(X86_FEATURE_TDX_GUEST);

	pr_info("Guest is initialized\n");
}
