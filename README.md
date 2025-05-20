# Overview

## Description

This repository contains Intel's Linux mainline-tracking kernel releases include mainline-tracking source code releases, Preempt-rt kernel releases and CVE patch series releases - only for few selected branches.

Linux mainline-tracking kernel use Linux [kernel.org](https://www.kernel.org/) mainline branch as base and plus Intel Linux kernel patches.

mainline-tracking kernel releases should only be used for platform feature evaluation prior to the platform’s launch and NOT for production or deployment with commercial Linux distribution. If you decide to use them, please integrate the latest functional and/or security updates when they are available from the open-source community.

## Branch information

### Basic branch naming rule

[1] main ==> Default branch, contains README.md and other documentations.

[2] linux/vx.xx ==> upstream vx.xx kernel + Intel patches, example: linux/v6.14, linux/v6.15-rc3

[3] linux-cve/vx.xx ==> CVE patches in quilt form, example: linux-cve/v6.12

    Please note: for CVE patches with config changes make sure that you regenerate kernel config using "make oldconfig"

[4] preempt-rt/vx.xx-rtxx ==> [linux-rt-devel](https://git.kernel.org/pub/scm/linux/kernel/git/rt/linux-rt-devel.git/) kernel + Intel patches, example: preempt-rt/v6.11-rt7, preempt-rt/v6.11-rc3-rt3. There is no separate preempt-rt kernel branch starting from 6.12.

Note: mainline-tracking kernel is rolling development kernel. These mainline-tracking branches are not long term support branches, old kernel branches will be End Of Life once new branch is available.

### Reference kernel overlay repository

You can find reference kernel configuration, kernel patch series and build scripts from this [linux kernel overlay release repository](https://github.com/intel/linux-kernel-overlay).
