/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_IRQ_VECTORS_H
#define _ASM_X86_IRQ_VECTORS_H

#include <linux/threads.h>
/*
 * Linux IRQ vector layout.
 *
 * There are 256 IDT entries (per CPU - each entry is 8 bytes) which can
 * be defined by Linux. They are used as a jump table by the CPU when a
 * given vector is triggered - by a CPU-external, CPU-internal or
 * software-triggered event.
 *
 * Linux sets the kernel code address each entry jumps to early during
 * bootup, and never changes them. This is the general layout of the
 * IDT entries:
 *
 *  Vectors   0 ...  31 : system traps and exceptions - hardcoded events
 *  Vectors  32 ... 127 : device interrupts
 *  Vector  128         : legacy int80 syscall interface
 *  Vectors 129 ... FIRST_SYSTEM_VECTOR-1 : device interrupts
 *  Vectors FIRST_SYSTEM_VECTOR ... 255   : special interrupts
 *
 * 64-bit x86 has per CPU IDT tables, 32-bit has one shared IDT table.
 *
 * This file enumerates the exact layout of them:
 */

/* This is used as an interrupt vector when programming the APIC. */
#define NMI_VECTOR			0x02

/*
 * IDT vectors usable for external interrupt sources start at 0x20.
 * (0x80 is the syscall vector, 0x30-0x3f are for ISA)
 */
#define FIRST_EXTERNAL_VECTOR		0x20

#define IA32_SYSCALL_VECTOR		0x80

/*
 * Vectors 0x30-0x3f are used for ISA interrupts.
 *   round up to the next 16-vector boundary
 */
#define ISA_IRQ_VECTOR(irq)		(((FIRST_EXTERNAL_VECTOR + 16) & ~15) + irq)

/*
 * Special IRQ vectors used by the SMP architecture, 0xf0-0xff
 *
 *  some of the following vectors are 'rare', they are merged
 *  into a single vector (CALL_FUNCTION_VECTOR) to save vector space.
 *  TLB, reschedule and local APIC vectors are performance-critical.
 */

#define SPURIOUS_APIC_VECTOR		0xff
/*
 * Sanity check
 */
#if ((SPURIOUS_APIC_VECTOR & 0x0F) != 0x0F)
# error SPURIOUS_APIC_VECTOR definition error
#endif

#define ERROR_APIC_VECTOR		0xfe
#define RESCHEDULE_VECTOR		0xfd
#define CALL_FUNCTION_VECTOR		0xfc
#define CALL_FUNCTION_SINGLE_VECTOR	0xfb
#define THERMAL_APIC_VECTOR		0xfa
#define THRESHOLD_APIC_VECTOR		0xf9
#define REBOOT_VECTOR			0xf8

/*
 * Generic system vector for platform specific use
 */
#define X86_PLATFORM_IPI_VECTOR		0xf7

/*
 * IRQ work vector:
 */
#define IRQ_WORK_VECTOR			0xf6

/* 0xf5 - unused, was UV_BAU_MESSAGE */
#define DEFERRED_ERROR_VECTOR		0xf4

/* Vector on which hypervisor callbacks will be delivered */
#define HYPERVISOR_CALLBACK_VECTOR	0xf3

/* Vector for KVM to deliver posted interrupt IPI */
#define POSTED_INTR_VECTOR		0xf2
#define POSTED_INTR_WAKEUP_VECTOR	0xf1
#define POSTED_INTR_NESTED_VECTOR	0xf0

#define MANAGED_IRQ_SHUTDOWN_VECTOR	0xef

#if IS_ENABLED(CONFIG_HYPERV)
#define HYPERV_REENLIGHTENMENT_VECTOR	0xee
#define HYPERV_STIMER0_VECTOR		0xed
#endif

#define LOCAL_TIMER_VECTOR		0xec

/*
 * Posted interrupt notification vector for all device MSIs delivered to
 * the host kernel.
 */
#define POSTED_MSI_NOTIFICATION_VECTOR	0xeb

#define NR_VECTORS			 256

#ifdef CONFIG_X86_LOCAL_APIC
#define FIRST_SYSTEM_VECTOR		POSTED_MSI_NOTIFICATION_VECTOR
#else
#define FIRST_SYSTEM_VECTOR		NR_VECTORS
#endif

#define NR_EXTERNAL_VECTORS		(FIRST_SYSTEM_VECTOR - FIRST_EXTERNAL_VECTOR)
#define NR_SYSTEM_VECTORS		(NR_VECTORS - FIRST_SYSTEM_VECTOR)

/*
 * When NMI-source reporting is supported, each logical processor maintains
 * an NMI-source bitmap and software can program different NMI originators
 * to use different vectors.
 *
 *   1) If a logical processor receives an NMI with a vector, it sets the
 *      bit corresponding to the vector as offset in the NMI-source bitmap.
 *
 *   2) If a logical processor receives an NMI without a vector, it sets
 *      bit 0 of the NMI-source bitmap.
 *
 *   3) NMIs are coalesced in the NMI-source bitmap until the following NMI
 *      delivery.
 *
 *   4) When a logical processor delivers an NMI, it saves the NMI-source
 *      bitmap on the stack as event data and clears it.
 *
 * The valid range of NMI-source vectors is 0~15, because the NMI-source
 * bitmap is defined as a 16-bit bitmap as of now, and 0 is used for all
 * unknown sources.  If an NMI is received with a vector out of the valid
 * range, bit 0 of the NMI-source bitmap is set.
 *
 * When bit 0 is set, software should invoke all registered NMI handlers as
 * if NMI-source reporting is not enabled.
 *
 * Vector 2 is set in local APIC LINT1 (= external NMI) for:
 *
 *   1) Platform NMIs routed through local APICs will be delivered with
 *      bit 2 set in the NMI-source bitmap.
 *
 *   2) Some third-party chipset might send NMI messages with a hardcoded
 *      vector of 2, whose delivery sets bit 2 of the NMI-source bitmap.
 *
 *   3) When bit 2 of the NMI-source bitmap is cleared, NMI handling code
 *      does NOT need to poll NMI handlers regsitered for vector 2.
 *
 * The NMI-source vectors are sorted by descending priority except 0 and 2;
 * vector 0 is the lowest priority NMIs, and vector 2 is the next lowest.
 */
#define NMI_SOURCE_VEC_UNKNOWN		0
#define NMI_SOURCE_VEC_IPI_REBOOT	1	/* Crash reboot */
#define NMI_SOURCE_VEC_EXT_NMI		2	/* Match IDT NMI vector 2 */
#define NMI_SOURCE_VEC_IPI_SMP_STOP	3	/* Panic stop CPU */
#define NMI_SOURCE_VEC_IPI_BT		4	/* CPU backtrace */
#define NMI_SOURCE_VEC_PMI		5	/* PerfMon counters */
#define NMI_SOURCE_VEC_IPI_KGDB		6	/* KGDB */
#define NMI_SOURCE_VEC_IPI_MCE		7	/* MCE injection */
#define NMI_SOURCE_VEC_IPI_TEST		8	/* For remote and local IPIs */
#define NR_NMI_SOURCE_VECTORS		9

/*
 * Size the maximum number of interrupts.
 *
 * If the irq_desc[] array has a sparse layout, we can size things
 * generously - it scales up linearly with the maximum number of CPUs,
 * and the maximum number of IO-APICs, whichever is higher.
 *
 * In other cases we size more conservatively, to not create too large
 * static arrays.
 */

#define NR_IRQS_LEGACY			16

#define CPU_VECTOR_LIMIT		(64 * NR_CPUS)
#define IO_APIC_VECTOR_LIMIT		(32 * MAX_IO_APICS)

#if defined(CONFIG_X86_IO_APIC) && defined(CONFIG_PCI_MSI)
#define NR_IRQS						\
	(CPU_VECTOR_LIMIT > IO_APIC_VECTOR_LIMIT ?	\
		(NR_VECTORS + CPU_VECTOR_LIMIT)  :	\
		(NR_VECTORS + IO_APIC_VECTOR_LIMIT))
#elif defined(CONFIG_X86_IO_APIC)
#define	NR_IRQS				(NR_VECTORS + IO_APIC_VECTOR_LIMIT)
#elif defined(CONFIG_PCI_MSI)
#define NR_IRQS				(NR_VECTORS + CPU_VECTOR_LIMIT)
#else
#define NR_IRQS				NR_IRQS_LEGACY
#endif

#endif /* _ASM_X86_IRQ_VECTORS_H */
