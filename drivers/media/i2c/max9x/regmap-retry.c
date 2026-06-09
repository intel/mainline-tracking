// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 Intel Corporation
 */

#include "regmap-retry.h"

int regmap_read_retry(struct regmap *map, unsigned int reg, unsigned int *val)
{
	int ret = 0;
	int retry = 50;

	while (retry--) {
		ret = regmap_read(map, reg, val);
		if (!ret)
			break;
		msleep(20);
	}

	return ret;
}

int regmap_write_retry(struct regmap *map, unsigned int reg, unsigned int val)
{
	int ret = 0;
	int retry = 50;

	while (retry--) {
		ret = regmap_write(map, reg, val);
		if (!ret)
			break;
		msleep(20);
	}

	return ret;
}

int regmap_update_bits_retry(struct regmap *map, unsigned int reg,
			     unsigned int mask, unsigned int val)
{
	int ret = 0;
	int retry = 50;

	while (retry--) {
		ret = regmap_update_bits(map, reg, mask, val);
		if (!ret)
			break;
		msleep(20);
	}

	return ret;
}
