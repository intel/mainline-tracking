/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PKEYS_INTERNAL_H
#define _ASM_X86_PKEYS_INTERNAL_H

#define PKR_AD_BIT 0x1
#define PKR_WD_BIT 0x2
#define PKR_BITS_PER_PKEY 2

#define PKR_PKEY_SHIFT(pkey) (pkey * PKR_BITS_PER_PKEY)
#define PKR_PKEY_MASK(pkey)  (((1 << PKR_BITS_PER_PKEY) - 1) << PKR_PKEY_SHIFT(pkey))

/*
 * Generate an Access-Disable and Write-Disable mask for the given pkey.
 * Several of the AD's are OR'd together to generate a default pkey register
 * value.
 */
#define PKR_AD_KEY(pkey)     (PKR_AD_BIT << PKR_PKEY_SHIFT(pkey))
#define PKR_WD_KEY(pkey)     (PKR_WD_BIT << PKR_PKEY_SHIFT(pkey))

/*
 * Define a default PKRS value for each task.
 *
 * Key 0 has no restriction.  All other keys are set to the most restrictive
 * value which is access disabled (AD=1).
 *
 * NOTE: This needs to be a macro to be used as part of the INIT_THREAD macro.
 */
#define INIT_PKRS_VALUE (PKR_AD_KEY(1) | PKR_AD_KEY(2) | PKR_AD_KEY(3) | \
			 PKR_AD_KEY(4) | PKR_AD_KEY(5) | PKR_AD_KEY(6) | \
			 PKR_AD_KEY(7) | PKR_AD_KEY(8) | PKR_AD_KEY(9) | \
			 PKR_AD_KEY(10) | PKR_AD_KEY(11) | PKR_AD_KEY(12) | \
			 PKR_AD_KEY(13) | PKR_AD_KEY(14) | PKR_AD_KEY(15))

/*  PKS supports 16 keys. Key 0 is reserved for the kernel. */
#define        PKS_KERN_DEFAULT_KEY    0
#define        PKS_NUM_KEYS            16

void write_pkrs(u32 new_pkrs);

#ifdef CONFIG_ARCH_HAS_SUPERVISOR_PKEYS
/*
 * Define pks_init_task and pks_sched_in as macros to avoid requiring the
 * definition of struct task_struct in this header while keeping the supervisor
 * pkey #ifdefery out of process.c and process_64.c
 */

/*
 * New tasks get the most restrictive PKRS value.
 */
#define pks_init_task(tsk) \
	tsk->thread.saved_pkrs = INIT_PKRS_VALUE;

/*
 * PKRS is only temporarily changed during specific code paths.  Only a
 * preemption during these windows away from the default value would
 * require updating the MSR.  write_pkrs() handles this optimization.
 */
#define pks_sched_in() \
	write_pkrs(current->thread.saved_pkrs);

#else
#define pks_init_task(tsk)
#define pks_sched_in()
#endif

#endif /*_ASM_X86_PKEYS_INTERNAL_H */
