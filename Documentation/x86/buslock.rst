.. SPDX-License-Identifier: GPL-2.0

===============================
Bus lock detection and handling
===============================

:Copyright: |copy| 2021 Intel Corporation
:Authors: - Fenghua Yu <fenghua.yu@intel.com>
          - Tony Luck <tony.luck@intel.com>

Problem
=======

A split lock is any atomic operation whose operand crosses two cache lines.
Since the operand spans two cache lines and the operation must be atomic,
the system locks the bus while the CPU accesses the two cache lines.

A bus lock is acquired through either split locked access to writeback (WB)
memory or any locked access to non-WB memory. This is typically thousands of
cycles slower than an atomic operation within a cache line. It also disrupts
performance on other cores and brings the whole system to its knees.

Detection
=========

Currently Linux can trace split lock event counter sq_misc.split_lock
for debug purpose. But for a system deployed in the field, this event
counter after the fact is insufficient. We need a mechanism that
detects bus lock before it happens to ensure that bus lock is never
incurred.

#AC exception for split lock detection
--------------------------------------

Intel introduces a mechanism to detect split lock via Alignment Check
(#AC) exception before badly aligned atomic instructions might impact
whole system performance in Tremont and other future processors.

#DB exception for bus lock detection
------------------------------------

Although split lock can be detected by #AC trap, the trap is triggered
before the instruction acquires bus lock. This makes it difficult to
mitigate bus lock (e.g. throttle the user application). And it cannot
detect bus lock on non-WB memory.

Some CPUs have ability to notify the kernel by an #DB trap after a user
instruction acquires a bus lock and is executed. This allows the kernel
to enforce user application throttling or mitigation.

Software handling
=================

The kernel #AC and #DB handlers handle bus lock based on kernel parameter
"split_lock_detect". Here is a summary of different options:

+------------------+----------------------------+-----------------------+
|split_lock_detect=|#AC for split lock		|#DB for bus lock	|
+------------------+----------------------------+-----------------------+
|off	  	   |Do nothing			|Do nothing		|
+------------------+----------------------------+-----------------------+
|warn		   |Kernel OOPs			|Warn once per task and |
|(default)	   |Warn once per task and	|and continues to run.  |
|		   |disable future checking	|			|
|		   |When both features are	|			|
|		   |supported, warn in #AC	|			|
+------------------+----------------------------+-----------------------+
|fatal		   |Kernel OOPs			|Send SIGBUS to user.	|
|		   |Send SIGBUS to user		|			|
|		   |When both features are	|			|
|		   |supported, fatal in #AC	|			|
+------------------+----------------------------+-----------------------+

Usages
======

Detecting and handling bus lock may find usages in various areas:

It is critical for real time system designers who build consolidated real
time systems. These systems run hard real time code on some cores and
run "untrusted" user processes on some other cores. The hard real time
cannot afford to have any bus lock from the untrusted processes to hurt
real time performance. To date the designers have been unable to deploy
these solutions as they have no way to prevent the "untrusted" user code
from generating split lock and bus lock to block the hard real time code
to access memory during bus locking.

It may also find usage in cloud. A user process with bus lock running
in one guest can block other cores from accessing shared memory.

Bus lock may open a security hole where malicious user code may slow
down overall system by executing instructions with bus lock.


Guidance
========
warn
----

The bus lock is warned so that it can be found and fixed. This is the
default behavior.

It may be useful to find and fix bus lock. The warning information has
process id and faulting instruction address to help pin point bus lock
and fix it.

fatal
-----

In this case, the bus lock is not tolerated and the process is killed.

It is useful in hard real time system.
