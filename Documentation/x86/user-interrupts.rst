.. SPDX-License-Identifier: GPL-2.0

=======================
User Interrupts (UINTR)
=======================

Overview
========
User Interrupts provides a low latency event delivery and inter process
communication mechanism. These events can be delivered directly to userspace
without a transition through the kernel.

In the User Interrupts architecture, a receiver is always expected to be a user
space task. However, a user interrupt can be sent by another user space task,
kernel or an external source (like a device). The feature that allows another
task to send an interrupt is referred to as User IPI.

Hardware Summary
================
User Interrupts is a posted interrupt delivery mechanism. The interrupts are
first posted to a memory location and then delivered to the receiver when they
are running with CPL=3.

Kernel managed architectural data structures
--------------------------------------------
UPID: User Posted Interrupt Descriptor - Holds receiver interrupt vector
information and notification state (like an ongoing notification, suppressed
notifications).

UITT: User Interrupt Target Table - Stores UPID pointer and vector information
for interrupt routing on the sender side. Referred by the senduipi instruction.

The interrupt state of each task is referenced via MSRs which are saved and
restored by the kernel during context switch.

Instructions
------------
senduipi <index> - send a user IPI to a target task based on the UITT index.

clui - Mask user interrupts by clearing UIF (User Interrupt Flag).

stui - Unmask user interrupts by setting UIF.

testui - Test current value of UIF.

uiret - return from a user interrupt handler.

User IPI
--------
When a User IPI sender executes 'senduipi <index>' the hardware refers the UITT
table entry pointed by the index and posts the interrupt vector into the
receiver's UPID.

If the receiver is running the sender cpu would send a physical IPI to the
receiver's cpu. On the receiver side this IPI is detected as a User Interrupt.
The User Interrupt handler for the receiver is invoked and the vector number is
pushed onto the stack.

Upon execution of 'uiret' in the interrupt handler, the control is transferred
back to instruction that was interrupted.

Refer the Intel Software Developer's Manual for more details.

Software Architecture
=====================
User Interrupts (Uintr) is an opt-in feature (unlike signals). Applications
wanting to use Uintr are expected to register themselves with the kernel using
the Uintr related system calls. A Uintr receiver is always a userspace task. A
Uintr sender can be another userspace task, kernel or a device.

1) A receiver can register/unregister an interrupt handler using the Uintr
receiver related syscalls.
		uintr_register_handler(handler, flags)

2) A syscall also allows a receiver to register a vector and create a user
interrupt file descriptor - uintr_fd.
		uintr_fd = uintr_create_fd(vector, flags)

Uintr can be useful in some of the usages where eventfd or signals are used for
frequent userspace event notifications. The semantics of uintr_fd are somewhat
similar to an eventfd() or the write end of a pipe.

3) Any sender with access to uintr_fd can use it to deliver events (in this
case - interrupts) to a receiver. A sender task can manage its connection with
the receiver using the sender related syscalls based on uintr_fd.
		uipi_index = uintr_register_sender(uintr_fd, flags)

Using an FD abstraction provides a secure mechanism to connect with a receiver.
The FD sharing and isolation mechanisms put in place by the kernel would extend
to Uintr as well.

4a) After the initial setup, a sender task can use the SENDUIPI instruction to
generate user IPIs without any kernel intervention.
		SENDUIPI <uipi_index>

If the receiver is running (CPL=3), then the user interrupt is delivered
directly without a kernel transition. If the receiver isn't running the
interrupt is delivered when the receiver gets context switched back. If the
receiver is blocked in the kernel, the user interrupt is delivered to the
kernel which then unblocks the intended receiver to deliver the interrupt.

4b) If the sender is the kernel or a device, the uintr_fd can be passed onto
the related kernel entity to allow them to setup a connection and then generate
a user interrupt for event delivery. <The exact details of this API are still
being worked upon.>

Refer the Uintr man-pages for details on the syscall interface.
