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

Intel processors may support either or both of the following hardware
mechanisms to detect split locks and bus locks.

#AC exception for split lock detection
--------------------------------------

Beginning with the Tremont Atom CPU split lock operations may raise an
Alignment Check (#AC) exception when a split lock operation is attemped.

#DB exception for bus lock detection
------------------------------------

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
|ratelimit:N	   |Do nothing			|Limit bus lock rate to	|
|(0 < N <= 1000)   |				|N bus locks per second	|
|		   |				|system wide and warn on|
|		   |				|bus locks.		|
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
off
---

Disable checking for split lock and bus lock. This option may be
useful if there are legacy applications that trigger these events
at a low rate so that mitigation is not needed.

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

ratelimit
---------

A system wide bus lock rate limit N is specified where 0 < N <= 1000.
Less bus locks can be generated when N is smaller.

This may find usage in throttling malicious processes in cloud. For
example, a few malicious users may generate a lot of bus locks to launch
Denial of Service (DoS) attack. By setting ratelimit, the system wide
bus locks is rate limited by N bus locks per second and the DoS attack
will be mitigated. The bus locks are warned so that the system
administrator can found the malicious users and processes.

Selecting a rate limit of 1000 would allow the bus to be locked for
up to about seven million cycles each second (assuming 7000 cycles for
each bus lock). On a 2 GHz processor that would be about 0.35% system
impact.
