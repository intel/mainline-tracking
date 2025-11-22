// SPDX-License-Identifier: MIT
/*
 * Copyright © 2025 Intel Corporation
 */

#include "xe_sc.h"

#include <linux/mutex.h>

#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "regs/xe_sc_regs.h"
#include "xe_device.h"
#include "xe_mmio.h"
#include "xe_platform_types.h"
#include "xe_soc_remapper.h"
#include "xe_sc_types.h"
#include "xe_tile.h"

static void xe_sc_remove(struct drm_device *drm, void *arg)
{
	struct xe_sc *sc = arg;
	struct xe_device *xe;

	if (!sc)
		return;

	xe = to_xe_device(drm);

	mutex_destroy(&sc->cmd_lock);

	xe_soc_remapper_set_sysctrl_region(xe, 0);

	xe->sc = NULL;
}

static int xe_sc_probe(struct xe_device *xe)
{
	struct xe_tile *tile = xe_device_get_root_tile(xe);
	struct xe_sc *sc;
	int ret;

	sc = drmm_kzalloc(&xe->drm, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->xe = xe;
	xe->sc = sc;

	ret = drmm_add_action_or_reset(&xe->drm, xe_sc_remove, sc);
	if (ret) {
		xe->sc = NULL;
		return ret;
	}

	xe_soc_remapper_set_sysctrl_region(xe, SYSCTRL_MAILBOX_INDEX);

	xe_mmio_init(&sc->mmio, tile, tile->mmio.regs, tile->mmio.regs_size);

	mutex_init(&sc->cmd_lock);

	sc->phase_bit = 0;

	return 0;
}

/**
 * xe_sc_init - Initialize SC subsystem
 * @xe: xe device instance
 *
 * Entry point for SC initialization, called from xe_device_probe().
 * This function checks platform support and calls the main probe function.
 *
 * Return: 0 on success, error code on failure
 */
int xe_sc_init(struct xe_device *xe)
{
	int ret;

	if (!xe->info.has_sysctrl)
		return 0;

	ret = xe_sc_probe(xe);
	if (ret)
		drm_err(&xe->drm, "sysctrl: Probe failed: %d\n", ret);

	return ret;
}
