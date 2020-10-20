.. SPDX-License-Identifier: GPL-2.0

======================
Memory Protection Keys
======================

Memory Protection Keys provides a mechanism for enforcing page-based
protections, but without requiring modification of the page tables
when an application changes protection domains.

PKeys Userspace (PKU) is a feature which is found on Intel's Skylake "Scalable
Processor" Server CPUs and later.  And It will be available in future
non-server Intel parts and future AMD processors.

Future Intel processors will support Protection Keys for Supervisor pages
(PKS).

For anyone wishing to test or use user space pkeys, it is available in Amazon's
EC2 C5 instances and is known to work there using an Ubuntu 17.04 image.

pkeys work by dedicating 4 previously Reserved bits in each page table entry to
a "protection key", giving 16 possible keys.  User and Supervisor pages are
treated separately.

Protections for each page are controlled with per CPU registers for each type
of page User and Supervisor.  Each of these 32 bit register stores two separate
bits (Access Disable and Write Disable) for each key.

For Userspace the register is user-accessible (rdpkru/wrpkru).  For
Supervisor, the register (MSR_IA32_PKRS) is accessible only to the kernel.

Being a CPU register, pkeys are inherently thread-local, potentially giving
each thread an independent set of protections from every other thread.

There are two new instructions (RDPKRU/WRPKRU) for reading and writing
to the new register.  The feature is only available in 64-bit mode,
even though there is theoretically space in the PAE PTEs.  These
permissions are enforced on data access only and have no effect on
instruction fetches.

For kernel space rdmsr/wrmsr are used to access the kernel MSRs.


Syscalls for user space keys
============================

There are 3 system calls which directly interact with pkeys::

	int pkey_alloc(unsigned long flags, unsigned long init_access_rights)
	int pkey_free(int pkey);
	int pkey_mprotect(unsigned long start, size_t len,
			  unsigned long prot, int pkey);

Before a pkey can be used, it must first be allocated with
pkey_alloc().  An application calls the WRPKRU instruction
directly in order to change access permissions to memory covered
with a key.  In this example WRPKRU is wrapped by a C function
called pkey_set().
::

	int real_prot = PROT_READ|PROT_WRITE;
	pkey = pkey_alloc(0, PKEY_DISABLE_WRITE);
	ptr = mmap(NULL, PAGE_SIZE, PROT_NONE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);
	ret = pkey_mprotect(ptr, PAGE_SIZE, real_prot, pkey);
	... application runs here

Now, if the application needs to update the data at 'ptr', it can
gain access, do the update, then remove its write access::

	pkey_set(pkey, 0); // clear PKEY_DISABLE_WRITE
	*ptr = foo; // assign something
	pkey_set(pkey, PKEY_DISABLE_WRITE); // set PKEY_DISABLE_WRITE again

Now when it frees the memory, it will also free the pkey since it
is no longer in use::

	munmap(ptr, PAGE_SIZE);
	pkey_free(pkey);

.. note:: pkey_set() is a wrapper for the RDPKRU and WRPKRU instructions.
          An example implementation can be found in
          tools/testing/selftests/x86/protection_keys.c.

Behavior
========

The kernel attempts to make protection keys consistent with the
behavior of a plain mprotect().  For instance if you do this::

	mprotect(ptr, size, PROT_NONE);
	something(ptr);

you can expect the same effects with protection keys when doing this::

	pkey = pkey_alloc(0, PKEY_DISABLE_WRITE | PKEY_DISABLE_READ);
	pkey_mprotect(ptr, size, PROT_READ|PROT_WRITE, pkey);
	something(ptr);

That should be true whether something() is a direct access to 'ptr'
like::

	*ptr = foo;

or when the kernel does the access on the application's behalf like
with a read()::

	read(fd, ptr, 1);

The kernel will send a SIGSEGV in both cases, but si_code will be set
to SEGV_PKERR when violating protection keys versus SEGV_ACCERR when
the plain mprotect() permissions are violated.


Kernel API for PKS support
==========================

The following calls are used to allocate, use, and deallocate a pkey which
defines a 'protection domain' within the kernel.  Setting a pkey value in a
supervisor mapping adds that mapping to the protection domain.  Then calls can be
used to enable/disable read and/or write access to all of the pages mapped with
that key:

        int pks_key_alloc(const char * const pkey_user);
        #define PAGE_KERNEL_PKEY(pkey)
        #define _PAGE_KEY(pkey)
        int pks_update_protection(int pkey, unsigned long protection, bool global);
        void pks_key_free(int pkey);

In-kernel users must be prepared to set PAGE_KERNEL_PKEY() permission in the
page table entries for the mappings they want to protect.

'global' in pks_update_protection() specifies that the protection should be set
accross all threads (CPU's) not just the current running thread/CPU.  This
increases the overhead of PKS and lessens the protection so it should be used
sparingly.

WARNING: It is imperative that callers check for errors from pks_key_alloc()
because pkeys are a limited resource and so callers should be prepared to work
without PKS support.

For admins a debugfs interface provides a list of the current keys in use at:

        /sys/kernel/debug/x86/pks_keys_allocated

Example code can be found in lib/pks/pks_test.c
