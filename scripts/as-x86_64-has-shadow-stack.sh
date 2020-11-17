#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

echo "wrussq %rax, (%rbx)" | $* -x assembler -c -
