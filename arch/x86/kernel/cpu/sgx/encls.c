// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
// Copyright(c) 2016-19 Intel Corporation.

#include <asm/cpufeature.h>
#include <asm/traps.h>
#include "encls.h"
#include "sgx.h"

/* A per-cpu cache for the last known values of IA32_SGXLEPUBKEYHASHx MSRs. */
static DEFINE_PER_CPU(u64 [4], sgx_lepubkeyhash_cache);

static void sgx_update_lepubkeyhash_msrs(u64 *lepubkeyhash, bool enforce)
{
	u64 *cache;
	int i;

	cache = per_cpu(sgx_lepubkeyhash_cache, smp_processor_id());
	for (i = 0; i < 4; i++) {
		if (enforce || (lepubkeyhash[i] != cache[i])) {
			wrmsrl(MSR_IA32_SGXLEPUBKEYHASH0 + i, lepubkeyhash[i]);
			cache[i] = lepubkeyhash[i];
		}
	}
}

/**
 * sgx_einit - initialize an enclave
 * @sigstruct:		a pointer a SIGSTRUCT
 * @token:		a pointer an EINITTOKEN (optional)
 * @secs:		a pointer a SECS
 * @lepubkeyhash:	the desired value for IA32_SGXLEPUBKEYHASHx MSRs
 *
 * Execute ENCLS[EINIT], writing the IA32_SGXLEPUBKEYHASHx MSRs according
 * to @lepubkeyhash (if possible and necessary).
 *
 * Return:
 *   0 on success,
 *   -errno or SGX error on failure
 */
int sgx_einit(struct sgx_sigstruct *sigstruct, struct sgx_einittoken *token,
	      struct sgx_epc_page *secs, u64 *lepubkeyhash)
{
	int ret;

	if (!boot_cpu_has(X86_FEATURE_SGX_LC))
		return __einit(sigstruct, token, sgx_epc_addr(secs));

	preempt_disable();
	sgx_update_lepubkeyhash_msrs(lepubkeyhash, false);
	ret = __einit(sigstruct, token, sgx_epc_addr(secs));
	if (ret == SGX_INVALID_EINITTOKEN) {
		sgx_update_lepubkeyhash_msrs(lepubkeyhash, true);
		ret = __einit(sigstruct, token, sgx_epc_addr(secs));
	}
	preempt_enable();
	return ret;
}
