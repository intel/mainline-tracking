.. SPDX-License-Identifier: GPL-2.0

Intel(R) Dynamic Application Loader (Intel(R) DAL)
===================================================

Introduction
------------

The Intel(R) Dynamic Application Loader Intel(R) DAL is a Trusted
Execution Environment (TEE) which, as a part of the ME firmware,
enables users to directly access and run small portions of their code
on part of the system's root of trust.

Onto this firmware, the DAL user installs a small Java applet, called
a Trusted Application (TA), or just an applet. From the host application that
runs on the device's operating system, the below interfaces are used
to interact with the applet for running a small function that needs
to be run in a secure environment, outside of the operating system.

More detailed information can be obtained at:
https://software.intel.com/dal

---

DAL exposes two interfaces to the operating system, which serve as
the communication channels between trusted applications (TAs) and host
based applications. One from the user space, called Intel(R) DAL
Host Interface or historically JHI, and one from kernel space,
called Intel(R) Management Engine Interface and Dynamic Application Loader
(Intel(R) DAL), or shortly Kernel DAL Interface KDI.

Both kernel and user space applications can communicate
with installed TAs, but only user space applications can install
and uninstall TAs, hence KDI still requires help from JHI.

Intel(R) MEI DAL Linux Kernel Driver
------------------------------------

The Intel(R) Management Engine Interface and Dynamic Application
Loader (Intel(R) MEI DAL) is a kernel component that provides both user
space and kernel space communication interfaces with DAL client in CSE
firmware, enabling the direct usage of DAL by Linux kernel components.

User Space Interface:
---------------------

DAL FW runs 3 processes:
    * DAL Security Domains Manager (DAL SDM)
        - manages the applets and security domains life cycles
    * DAL Intel Virtual Machine (DAL IVM)
        - the VM that runs the applets byte code
    * DAL Launcher
       - A place holder for future second VM and native applets support

For each one of them, the driver exposes a char device called
 ``/dev/dal{i}``, for i in 0-2.

The character device is channel-only interface between user a space application
and the DAL FW, it allows sending and reception of raw messages without
any processing or modification. The messages are sent using the char device
c:func:`write()` and c:func:`read()` system calls. The management is done via JHI.
(for more information about JHI:
https://github.com/intel/dynamic-application-loader-host-interface)


.. kernel-render:: DOT
   :alt: DAL  digraph
   :caption: **DAL** User Space Stack

    digraph UDI {
        subgraph {
            rank = same;
            "TEE APP" -> JHI;
        }
        JHI -> "/dev/dalX";
        "/dev/dalX" -> "FW TA";
   }



Kernel Space Interface:
-----------------------

The kernel interface is also involved in applet and session management.

The driver exposes API in <linux/dal.h> file, to allow kernel space
modules to communicating with Intel DAL.

.. kernel-doc:: drivers/misc/mei/dal/dal_kdi.c
    :export:

.. kernel-doc:: drivers/misc/mei/dal/bh_external.c
    :export:

KDI Sample:
-----------
A KDI sample application can be found at:

https://software.intel.com/dal-developer-guide-sdk-contents-kdi-sample
