// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Intel Corporation
 */

#ifndef _REGMAP_RETRY_H
#define _REGMAP_RETRY_H

#include <linux/regmap.h>

int regmap_read_retry(struct regmap *map, unsigned int reg, unsigned int *val);

int regmap_write_retry(struct regmap *map, unsigned int reg, unsigned int val);

int regmap_update_bits_retry(struct regmap *map, unsigned int reg,
			     unsigned int mask, unsigned int val);

#endif
