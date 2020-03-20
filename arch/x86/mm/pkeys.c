// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Memory Protection Keys management
 * Copyright (c) 2015, Intel Corporation.
 */
#undef pr_fmt
#define pr_fmt(fmt) "x86/pkeys: " fmt

#include <linux/debugfs.h>		/* debugfs_create_u32()		*/
#include <linux/mm_types.h>             /* mm_struct, vma, etc...       */
#include <linux/pkeys.h>                /* PKEY_*                       */
#include <uapi/asm-generic/mman-common.h>

#include <asm/cpufeature.h>             /* boot_cpu_has, ...            */
#include <asm/mmu_context.h>            /* vma_pkey()                   */
#include <asm/fpu/internal.h>		/* init_fpstate			*/
#include <asm/pks.h>

int __execute_only_pkey(struct mm_struct *mm)
{
	bool need_to_set_mm_pkey = false;
	int execute_only_pkey = mm->context.execute_only_pkey;
	int ret;

	/* Do we need to assign a pkey for mm's execute-only maps? */
	if (execute_only_pkey == -1) {
		/* Go allocate one to use, which might fail */
		execute_only_pkey = mm_pkey_alloc(mm);
		if (execute_only_pkey < 0)
			return -1;
		need_to_set_mm_pkey = true;
	}

	/*
	 * We do not want to go through the relatively costly
	 * dance to set PKRU if we do not need to.  Check it
	 * first and assume that if the execute-only pkey is
	 * write-disabled that we do not have to set it
	 * ourselves.
	 */
	if (!need_to_set_mm_pkey &&
	    !__pkru_allows_read(read_pkru(), execute_only_pkey)) {
		return execute_only_pkey;
	}

	/*
	 * Set up PKRU so that it denies access for everything
	 * other than execution.
	 */
	ret = arch_set_user_pkey_access(current, execute_only_pkey,
			PKEY_DISABLE_ACCESS);
	/*
	 * If the PKRU-set operation failed somehow, just return
	 * 0 and effectively disable execute-only support.
	 */
	if (ret) {
		mm_set_pkey_free(mm, execute_only_pkey);
		return -1;
	}

	/* We got one, store it and use it from here on out */
	if (need_to_set_mm_pkey)
		mm->context.execute_only_pkey = execute_only_pkey;
	return execute_only_pkey;
}

static inline bool vma_is_pkey_exec_only(struct vm_area_struct *vma)
{
	/* Do this check first since the vm_flags should be hot */
	if ((vma->vm_flags & VM_ACCESS_FLAGS) != VM_EXEC)
		return false;
	if (vma_pkey(vma) != vma->vm_mm->context.execute_only_pkey)
		return false;

	return true;
}

/*
 * This is only called for *plain* mprotect calls.
 */
int __arch_override_mprotect_pkey(struct vm_area_struct *vma, int prot, int pkey)
{
	/*
	 * Is this an mprotect_pkey() call?  If so, never
	 * override the value that came from the user.
	 */
	if (pkey != -1)
		return pkey;

	/*
	 * The mapping is execute-only.  Go try to get the
	 * execute-only protection key.  If we fail to do that,
	 * fall through as if we do not have execute-only
	 * support in this mm.
	 */
	if (prot == PROT_EXEC) {
		pkey = execute_only_pkey(vma->vm_mm);
		if (pkey > 0)
			return pkey;
	} else if (vma_is_pkey_exec_only(vma)) {
		/*
		 * Protections are *not* PROT_EXEC, but the mapping
		 * is using the exec-only pkey.  This mapping was
		 * PROT_EXEC and will no longer be.  Move back to
		 * the default pkey.
		 */
		return ARCH_DEFAULT_PKEY;
	}

	/*
	 * This is a vanilla, non-pkey mprotect (or we failed to
	 * setup execute-only), inherit the pkey from the VMA we
	 * are working on.
	 */
	return vma_pkey(vma);
}

/*
 * Make the default PKRU value (at execve() time) as restrictive
 * as possible.  This ensures that any threads clone()'d early
 * in the process's lifetime will not accidentally get access
 * to data which is pkey-protected later on.
 */
u32 init_pkru_value = PKR_AD_KEY( 1) | PKR_AD_KEY( 2) | PKR_AD_KEY( 3) |
		      PKR_AD_KEY( 4) | PKR_AD_KEY( 5) | PKR_AD_KEY( 6) |
		      PKR_AD_KEY( 7) | PKR_AD_KEY( 8) | PKR_AD_KEY( 9) |
		      PKR_AD_KEY(10) | PKR_AD_KEY(11) | PKR_AD_KEY(12) |
		      PKR_AD_KEY(13) | PKR_AD_KEY(14) | PKR_AD_KEY(15);

/*
 * Called from the FPU code when creating a fresh set of FPU
 * registers.  This is called from a very specific context where
 * we know the FPU regstiers are safe for use and we can use PKRU
 * directly.
 */
void copy_init_pkru_to_fpregs(void)
{
	u32 init_pkru_value_snapshot = READ_ONCE(init_pkru_value);
	/*
	 * Override the PKRU state that came from 'init_fpstate'
	 * with the baseline from the process.
	 */
	write_pkru(init_pkru_value_snapshot);
}

static ssize_t init_pkru_read_file(struct file *file, char __user *user_buf,
			     size_t count, loff_t *ppos)
{
	char buf[32];
	unsigned int len;

	len = sprintf(buf, "0x%x\n", init_pkru_value);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t init_pkru_write_file(struct file *file,
		 const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct pkru_state *pk;
	char buf[32];
	ssize_t len;
	u32 new_init_pkru;

	len = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;

	/* Make the buffer a valid string that we can not overrun */
	buf[len] = '\0';
	if (kstrtouint(buf, 0, &new_init_pkru))
		return -EINVAL;

	/*
	 * Don't allow insane settings that will blow the system
	 * up immediately if someone attempts to disable access
	 * or writes to pkey 0.
	 */
	if (new_init_pkru & (PKR_AD_BIT|PKR_WD_BIT))
		return -EINVAL;

	WRITE_ONCE(init_pkru_value, new_init_pkru);
	pk = get_xsave_addr(&init_fpstate.xsave, XFEATURE_PKRU);
	if (!pk)
		return -EINVAL;
	pk->pkru = new_init_pkru;
	return count;
}

static const struct file_operations fops_init_pkru = {
	.read = init_pkru_read_file,
	.write = init_pkru_write_file,
	.llseek = default_llseek,
};

static int __init create_init_pkru_value(void)
{
	debugfs_create_file("init_pkru", S_IRUSR | S_IWUSR,
			arch_debugfs_dir, NULL, &fops_init_pkru);
	return 0;
}
late_initcall(create_init_pkru_value);

static __init int setup_init_pkru(char *opt)
{
	u32 new_init_pkru;

	if (kstrtouint(opt, 0, &new_init_pkru))
		return 1;

	WRITE_ONCE(init_pkru_value, new_init_pkru);

	return 1;
}
__setup("init_pkru=", setup_init_pkru);

/*
 * Replace disable bits for @pkey with values from @flags
 *
 * Kernel users use the same flags as user space:
 *     PKEY_DISABLE_ACCESS
 *     PKEY_DISABLE_WRITE
 */
u32 update_pkey_val(u32 pk_reg, int pkey, unsigned int flags)
{
	/*  Mask out old bit values */
	pk_reg &= ~PKR_PKEY_MASK(pkey);

	/*  Or in new values */
	if (flags & PKEY_DISABLE_ACCESS)
		pk_reg |= PKR_AD_KEY(pkey);
	if (flags & PKEY_DISABLE_WRITE)
		pk_reg |= PKR_WD_KEY(pkey);

	return pk_reg;
}

#ifdef CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS

static DEFINE_PER_CPU(u32, pkrs_cache);

/*
 * write_pkrs() optimizes MSR writes by maintaining a per cpu cache which can
 * be checked quickly.
 *
 * It should also be noted that the underlying WRMSR(MSR_IA32_PKRS) is not
 * serializing but still maintains ordering properties similar to WRPKRU.
 * The current SDM section on PKRS needs updating but should be the same as
 * that of WRPKRU.  So to quote from the WRPKRU text:
 *
 *     WRPKRU will never execute transiently. Memory accesses
 *     affected by PKRU register will not execute (even transiently)
 *     until all prior executions of WRPKRU have completed execution
 *     and updated the PKRU register.
 */
void write_pkrs(u32 new_pkrs)
{
	u32 *pkrs;

	if (!static_cpu_has(X86_FEATURE_PKS))
		return;

	pkrs = get_cpu_ptr(&pkrs_cache);
	if (*pkrs != new_pkrs) {
		*pkrs = new_pkrs;
		wrmsrl(MSR_IA32_PKRS, new_pkrs);
	}
	put_cpu_ptr(pkrs);
}

/*
 * PKS is independent of PKU and either or both may be supported on a CPU.
 * Configure PKS if the CPU supports the feature.
 */
void setup_pks(void)
{
	if (!cpu_feature_enabled(X86_FEATURE_PKS))
		return;

	write_pkrs(INIT_PKRS_VALUE);
	cr4_set_bits(X86_CR4_PKS);
}

/*
 * Do not call this directly, see pks_mk*() below.
 *
 * @pkey: Key for the domain to change
 * @protection: protection bits to be used
 *
 * Protection utilizes the same protection bits specified for User pkeys
 *     PKEY_DISABLE_ACCESS
 *     PKEY_DISABLE_WRITE
 *
 */
static inline void pks_update_protection(int pkey, unsigned long protection)
{
	current->thread.saved_pkrs = update_pkey_val(current->thread.saved_pkrs,
						     pkey, protection);
	write_pkrs(current->thread.saved_pkrs);
}

/**
 * pks_mk_noaccess() - Disable all access to the domain
 * @pkey the pkey for which the access should change.
 *
 * Disable all access to the domain specified by pkey.  This is a global
 * update and only affects the current running thread.
 *
 * It is a bug for users to call this without a valid pkey returned from
 * pks_key_alloc()
 */
void pks_mk_noaccess(int pkey)
{
	pks_update_protection(pkey, PKEY_DISABLE_ACCESS);
}
EXPORT_SYMBOL_GPL(pks_mk_noaccess);

/**
 * pks_mk_readonly() - Make the domain Read only
 * @pkey the pkey for which the access should change.
 *
 * Allow read access to the domain specified by pkey.  This is a global update
 * and only affects the current running thread.
 *
 * It is a bug for users to call this without a valid pkey returned from
 * pks_key_alloc()
 */
void pks_mk_readonly(int pkey)
{
	pks_update_protection(pkey, PKEY_DISABLE_WRITE);
}
EXPORT_SYMBOL_GPL(pks_mk_readonly);

/**
 * pks_mk_readwrite() - Make the domain Read/Write
 * @pkey the pkey for which the access should change.
 *
 * Allow all access, read and write, to the domain specified by pkey.  This is
 * a global update and only affects the current running thread.
 *
 * It is a bug for users to call this without a valid pkey returned from
 * pks_key_alloc()
 */
void pks_mk_readwrite(int pkey)
{
	pks_update_protection(pkey, 0);
}
EXPORT_SYMBOL_GPL(pks_mk_readwrite);

static const char pks_key_user0[] = "kernel";

/* Store names of allocated keys for debug.  Key 0 is reserved for the kernel.  */
static const char *pks_key_users[PKS_NUM_KEYS] = {
	pks_key_user0
};

/*
 * Each key is represented by a bit.  Bit 0 is set for key 0 and reserved for
 * its use.  We use ulong for the bit operations but only 16 bits are used.
 */
static unsigned long pks_key_allocation_map = 1 << PKS_KERN_DEFAULT_KEY;

/**
 * pks_key_alloc() - Allocate a PKS key
 * @pkey_user: String stored for debugging of key exhaustion.  The caller is
 *             responsible to maintain this memory until pks_key_free().
 *
 * Return: pkey if success
 *         -EOPNOTSUPP if pks is not supported or not enabled
 *         -ENOSPC if no keys are available
 */
__must_check int pks_key_alloc(const char * const pkey_user)
{
	int nr;

	if (!cpu_feature_enabled(X86_FEATURE_PKS))
		return -EOPNOTSUPP;

	while (1) {
		nr = find_first_zero_bit(&pks_key_allocation_map, PKS_NUM_KEYS);
		if (nr >= PKS_NUM_KEYS) {
			pr_info("Cannot allocate supervisor key for %s.\n",
				pkey_user);
			return -ENOSPC;
		}
		if (!test_and_set_bit_lock(nr, &pks_key_allocation_map))
			break;
	}

	/* for debugging key exhaustion */
	pks_key_users[nr] = pkey_user;

	return nr;
}
EXPORT_SYMBOL_GPL(pks_key_alloc);

/**
 * pks_key_free() - Free a previously allocate PKS key
 * @pkey: Key to be free'ed
 */
void pks_key_free(int pkey)
{
	if (pkey >= PKS_NUM_KEYS || pkey <= PKS_KERN_DEFAULT_KEY) {
		pr_err("Invalid PKey value: %d\n", pkey);
		return;
	}

	/* Restore to default of no access */
	pks_mk_noaccess(pkey);
	pks_key_users[pkey] = NULL;
	clear_bit_unlock(pkey, &pks_key_allocation_map);
}
EXPORT_SYMBOL_GPL(pks_key_free);

#endif /* CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS */
