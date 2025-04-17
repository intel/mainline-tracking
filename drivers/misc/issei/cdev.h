/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023-2025 Intel Corporation */
#ifndef _ISSEI_CDEV_H_
#define _ISSEI_CDEV_H_

struct device;
struct issei_device;

int issei_register(struct issei_device *idev, struct device *parent);
void issei_deregister(struct issei_device *idev);

#endif /* _ISSEI_CDEV_H_ */
