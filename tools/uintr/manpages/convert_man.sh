#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

for file in *.man
do
  groff -mandoc -Tascii -rHY=0 $file | col -bx > "${file%.man}.txt"
done

