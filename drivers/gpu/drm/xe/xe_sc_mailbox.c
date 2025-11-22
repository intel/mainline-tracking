// SPDX-License-Identifier: MIT
/*
 * Copyright © 2025 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include <drm/drm_print.h>

#include "regs/xe_sc_regs.h"
#include "xe_device.h"
#include "xe_mmio.h"
#include "xe_pm.h"
#include "xe_sc.h"
#include "xe_sc_mailbox.h"
#include "xe_sc_types.h"

static bool sc_mb_wait_bit_clear(struct xe_sc *sc, u32 bit_mask,
				 unsigned int timeout_ms)
{
	int ret;

	if (timeout_ms == 0) {
		ret = xe_mmio_wait32_not(&sc->mmio, SC_MB_CTRL, bit_mask, bit_mask,
					 0, NULL, false);
	} else {
		ret = xe_mmio_wait32_not(&sc->mmio, SC_MB_CTRL, bit_mask, bit_mask,
					 timeout_ms * 1000, NULL, false);
	}

	return ret == 0;
}

static bool sc_mb_wait_bit_set(struct xe_sc *sc, u32 bit_mask,
			       unsigned int timeout_ms)
{
	int ret;

	if (timeout_ms == 0) {
		ret = xe_mmio_wait32(&sc->mmio, SC_MB_CTRL, bit_mask, bit_mask,
				     0, NULL, false);
	} else {
		ret = xe_mmio_wait32(&sc->mmio, SC_MB_CTRL, bit_mask, bit_mask,
				     timeout_ms * 1000, NULL, false);
	}

	return ret == 0;
}

static void sc_mb_write_frame(struct xe_sc *sc, const void *buffer,
			      size_t offset)
{
	const u32 *data = (const u32 *)((const u8 *)buffer + offset);

	xe_mmio_write32(&sc->mmio, SC_MB_DATA0, data[0]);
	xe_mmio_write32(&sc->mmio, SC_MB_DATA1, data[1]);
	xe_mmio_write32(&sc->mmio, SC_MB_DATA2, data[2]);
	xe_mmio_write32(&sc->mmio, SC_MB_DATA3, data[3]);
}

static void sc_mb_read_frame(struct xe_sc *sc, void *buffer,
			     size_t offset)
{
	u32 *data = (u32 *)((u8 *)buffer + offset);

	data[0] = xe_mmio_read32(&sc->mmio, SC_MB_DATA0);
	data[1] = xe_mmio_read32(&sc->mmio, SC_MB_DATA1);
	data[2] = xe_mmio_read32(&sc->mmio, SC_MB_DATA2);
	data[3] = xe_mmio_read32(&sc->mmio, SC_MB_DATA3);
}

static void sc_mb_clear_response(struct xe_sc *sc)
{
	xe_mmio_write32(&sc->mmio, SC_MB_CTRL, SC_MB_CTRL_RUN_BUSY_OUT);
}

static int sc_mb_prepare_command(struct xe_sc *sc,
				 const struct xe_sc_mailbox_app_msg_hdr *msg_hdr,
				 const void *data_in, size_t data_in_len,
				 u8 **cmd_buffer, size_t *cmd_size)
{
	struct xe_sc_mailbox_mkhi_msg_hdr mkhi_hdr = {0};

	mkhi_hdr.group_id = msg_hdr->group_id;
	mkhi_hdr.command = msg_hdr->command & 0x7F;
	mkhi_hdr.is_response = 0;
	mkhi_hdr.reserved = 0;
	mkhi_hdr.result = 0;

	*cmd_size = sizeof(struct xe_sc_mailbox_mkhi_msg_hdr) + data_in_len;

	if (*cmd_size > SC_MB_MAX_MESSAGE_SIZE) {
		drm_err(&sc->xe->drm, "SC: Message too large: %zu bytes (max %u)\n",
			*cmd_size, SC_MB_MAX_MESSAGE_SIZE);
		return -EINVAL;
	}

	*cmd_buffer = kmalloc(*cmd_size, GFP_KERNEL);
	if (!*cmd_buffer)
		return -ENOMEM;

	memcpy(*cmd_buffer, &mkhi_hdr, sizeof(struct xe_sc_mailbox_mkhi_msg_hdr));
	if (data_in && data_in_len)
		memcpy(*cmd_buffer + sizeof(struct xe_sc_mailbox_mkhi_msg_hdr),
		       data_in, data_in_len);

	drm_dbg(&sc->xe->drm, "SC: request: group=0x%02x cmd=0x%02x payload=%zu bytes\n",
		mkhi_hdr.group_id, mkhi_hdr.command, data_in_len);

	return 0;
}

static int sc_mb_send_frames(struct xe_sc *sc, const u8 *cmd_buffer,
			     size_t cmd_size, unsigned int timeout_ms)
{
	u32 ctrl_reg, total_frames, current_frame;
	size_t bytes_sent, bytes_to_send;

	total_frames = DIV_ROUND_UP(cmd_size, SC_MB_FRAME_SIZE);
	if (total_frames > SC_MB_MAX_FRAMES) {
		drm_err(&sc->xe->drm, "SC: Message too large: %zu bytes (%u frames, max %u)\n",
			cmd_size, total_frames, SC_MB_MAX_FRAMES);
		return -EINVAL;
	}

	if (!sc_mb_wait_bit_clear(sc, SC_MB_CTRL_RUN_BUSY, timeout_ms)) {
		drm_err(&sc->xe->drm, "SC: Mailbox busy (RUN_BUSY timeout)\n");
		return -EBUSY;
	}

	sc->phase_bit ^= 1;

	drm_dbg(&sc->xe->drm, "SC: Sending message: %zu bytes, %u frames, phase=%u\n",
		cmd_size, total_frames, sc->phase_bit);

	bytes_sent = 0;
	for (current_frame = 0; current_frame < total_frames; current_frame++) {
		bytes_to_send = min(cmd_size - bytes_sent, (size_t)SC_MB_FRAME_SIZE);

		sc_mb_write_frame(sc, cmd_buffer, bytes_sent);

		ctrl_reg = SC_MB_CTRL_RUN_BUSY |
			   FIELD_PREP(MKHI_FRAME_CURRENT, current_frame) |
			   FIELD_PREP(MKHI_FRAME_TOTAL, total_frames - 1) |
			   FIELD_PREP(MKHI_FRAME_COMMAND, SC_MKHI_COMMAND);
		if (sc->phase_bit)
			ctrl_reg |= FIELD_PREP(MKHI_FRAME_PHASE, 1);

		xe_mmio_write32(&sc->mmio, SC_MB_CTRL, ctrl_reg);

		drm_dbg(&sc->xe->drm, "SC: Sent frame %u/%u\n",
			current_frame, total_frames - 1);

		if (!sc_mb_wait_bit_clear(sc, SC_MB_CTRL_RUN_BUSY, timeout_ms)) {
			drm_err(&sc->xe->drm, "SC: Frame %u acknowledgment timeout\n",
				current_frame);
			return -ETIMEDOUT;
		}

		bytes_sent += bytes_to_send;
	}

	drm_dbg(&sc->xe->drm, "SC: All frames sent successfully (%zu bytes)\n", bytes_sent);
	return 0;
}

static int sc_mb_validate_response_header(struct xe_sc *sc,
					  const struct xe_sc_mailbox_app_msg_hdr *msg_hdr,
					  const struct xe_sc_mailbox_mkhi_msg_hdr *resp_hdr)
{
	if (!resp_hdr->is_response ||
	    resp_hdr->group_id != msg_hdr->group_id ||
	    resp_hdr->command != (msg_hdr->command & 0x7F)) {
		drm_err(&sc->xe->drm, "SC: Invalid response header\n");
		return -EPROTO;
	}

	if (resp_hdr->result != 0) {
		drm_err(&sc->xe->drm, "SC: Firmware error: result=0x%02x\n",
			resp_hdr->result);
		return -EIO;
	}

	drm_dbg(&sc->xe->drm, "SC: response: group=0x%02x cmd=0x%02x\n",
		resp_hdr->group_id, resp_hdr->command);

	return 0;
}

static int sc_mb_receive_frames(struct xe_sc *sc,
				const struct xe_sc_mailbox_app_msg_hdr *msg_hdr,
				void *data_out, size_t data_out_len,
				size_t *bytes_received, unsigned int timeout_ms)
{
	u32 ctrl_reg, total_frames, current_frame;
	size_t payload_size;
	int ret;

	*bytes_received = 0;

	do {
		ctrl_reg = xe_mmio_read32(&sc->mmio, SC_MB_CTRL);
		current_frame = FIELD_GET(MKHI_FRAME_CURRENT, ctrl_reg);
		total_frames = FIELD_GET(MKHI_FRAME_TOTAL, ctrl_reg) + 1;

		drm_dbg(&sc->xe->drm, "SC: Receiving frame %u/%u\n",
			current_frame, total_frames - 1);

		if (current_frame == 0) {
			u32 temp_frame[4];
			struct xe_sc_mailbox_mkhi_msg_hdr *resp_hdr;

			sc_mb_read_frame(sc, temp_frame, 0);
			resp_hdr = (struct xe_sc_mailbox_mkhi_msg_hdr *)temp_frame;

			ret = sc_mb_validate_response_header(sc, msg_hdr, resp_hdr);
			if (ret)
				return ret;

			payload_size = SC_MB_FRAME_SIZE - sizeof(struct xe_sc_mailbox_mkhi_msg_hdr);
			if (payload_size > data_out_len) {
				drm_err(&sc->xe->drm, "SC: Response buffer too small\n");
				return -ENOSPC;
			}

			memcpy(data_out,
			       (u8 *)temp_frame + sizeof(struct xe_sc_mailbox_mkhi_msg_hdr),
			       payload_size);
			*bytes_received = payload_size;
		} else {
			size_t frame_size = SC_MB_FRAME_SIZE;

			if (current_frame == total_frames - 1) {
				size_t total_response = (total_frames - 1) *
							SC_MB_FRAME_SIZE +
							sizeof(struct xe_sc_mailbox_mkhi_msg_hdr);
				size_t remaining = total_response -
						   *bytes_received -
						   sizeof(struct xe_sc_mailbox_mkhi_msg_hdr);
				frame_size = min(frame_size, remaining);
			}

			if (*bytes_received + frame_size > data_out_len) {
				drm_err(&sc->xe->drm, "SC: Response buffer too small\n");
				return -ENOSPC;
			}

			sc_mb_read_frame(sc, data_out, *bytes_received);
			*bytes_received += frame_size;
		}

		sc_mb_clear_response(sc);

		if (current_frame + 1 < total_frames &&
		    !sc_mb_wait_bit_set(sc, SC_MB_CTRL_RUN_BUSY_OUT, timeout_ms)) {
			drm_err(&sc->xe->drm, "SC: Response frame %u timeout\n",
				current_frame + 1);
			return -ETIMEDOUT;
		}

	} while (current_frame + 1 < total_frames);

	return 0;
}

static int sc_mb_send_command(struct xe_sc *sc,
			      const struct xe_sc_mailbox_app_msg_hdr *msg_hdr,
			      const u8 *cmd_buffer, size_t cmd_size,
			      void *data_out, size_t data_out_len,
			      size_t *rdata_len, unsigned int timeout_ms)
{
	size_t bytes_received;
	int ret;

	if (rdata_len)
		*rdata_len = 0;

	ret = sc_mb_send_frames(sc, cmd_buffer, cmd_size, timeout_ms);
	if (ret)
		return ret;

	if (!data_out) {
		drm_dbg(&sc->xe->drm, "SC: Command completed (no response expected)\n");
		return 0;
	}

	if (!sc_mb_wait_bit_set(sc, SC_MB_CTRL_RUN_BUSY_OUT, timeout_ms)) {
		drm_err(&sc->xe->drm, "SC: Response timeout (RUN_BUSY_OUT not set)\n");
		return -ETIMEDOUT;
	}

	ret = sc_mb_receive_frames(sc, msg_hdr, data_out, data_out_len,
				   &bytes_received, timeout_ms);
	if (ret) {
		sc_mb_clear_response(sc);
		return ret;
	}

	if (rdata_len)
		*rdata_len = bytes_received;

	drm_dbg(&sc->xe->drm, "SC: MKHI message completed: %zu bytes payload received\n",
		bytes_received);

	return 0;
}

/**
 * xe_sc_mailbox_send_command - Send command to System Controller via mailbox
 * @handle: XE device handle containing the system controller
 * @cmd_buffer: Pointer to xe_sc_mailbox_command structure
 * @rdata_len: Pointer to store actual response data size (can be NULL)
 *
 * Send a command to the System Controller using MKHI protocol. Handles
 * command preparation, fragmentation, transmission, and response reception.
 * Optimized to move memory allocation outside the critical section.
 *
 * Return: 0 on success, negative error code on failure
 */
int xe_sc_mailbox_send_command(void *handle, void *cmd_buffer, size_t *rdata_len)
{
	struct xe_device *xe = handle;
	struct xe_sc *sc;
	struct xe_sc_mailbox_command *cmd = cmd_buffer;
	u8 *command_buffer = NULL;
	size_t command_size;
	int ret;

	if (!xe || !cmd)
		return -EINVAL;

	sc = xe->sc;
	if (!sc)
		return -ENODEV;

	if (!cmd->data_in && cmd->data_in_len)
		return -EINVAL;

	if (!cmd->data_out && cmd->data_out_len)
		return -EINVAL;

	might_sleep();

	ret = sc_mb_prepare_command(sc, &cmd->header, cmd->data_in, cmd->data_in_len,
				    &command_buffer, &command_size);
	if (ret) {
		drm_err(&sc->xe->drm, "SC: Failed to prepare command: %d\n", ret);
		return ret;
	}

	mutex_lock(&sc->cmd_lock);

	xe_pm_runtime_get(xe);

	ret = sc_mb_send_command(sc, &cmd->header, command_buffer, command_size,
				 cmd->data_out, cmd->data_out_len, rdata_len,
				 SC_MB_DEFAULT_TIMEOUT_MS);
	if (ret)
		drm_err(&sc->xe->drm, "SC: Mailbox command failed: %d\n", ret);

	xe_pm_runtime_put(xe);

	mutex_unlock(&sc->cmd_lock);

	kfree(command_buffer);

	return ret;
}
