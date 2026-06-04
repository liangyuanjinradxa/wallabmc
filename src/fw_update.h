/*
 * Firmware Update via WebUI
 *
 * Receives firmware image chunks via HTTP POST and writes them to MCUboot slot1.
 * After all chunks are received, verifies the image and reboots to trigger MCUboot swap.
 *
 * SPDX-FileCopyrightText: © 2025-2026 Tenstorrent USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Maximum firmware image size (must fit in slot1_partition) */
#define FW_UPDATE_MAX_SIZE (896 * 1024)

/* Firmware update states */
enum fw_update_state {
	FW_UPDATE_IDLE,
	FW_UPDATE_ERASING,      /* Erasing slot1 in background */
	FW_UPDATE_IN_PROGRESS,  /* Receiving data + writing to slot1 */
	FW_UPDATE_COMPLETE,     /* All data received and verified */
	FW_UPDATE_ERROR,        /* Error occurred */
};

/**
 * @brief Initialize the firmware update subsystem.
 *
 * Must be called once at boot. Opens the slot1 flash device.
 *
 * @return 0 on success, negative errno on failure.
 */
int fw_update_init(void);

/**
 * @brief Start a firmware update.
 *
 * Erases slot1 and prepares to receive firmware data.
 *
 * @param total_size Expected total size of the firmware image.
 * @return 0 on success, negative errno on failure.
 */
int fw_update_start(size_t total_size);

/**
 * @brief Write a chunk of firmware data to slot1.
 *
 * Each call writes len bytes at the current write offset.
 * The caller must yield to the network stack between chunks.
 *
 * @param data Pointer to the firmware data chunk.
 * @param len Length of the chunk.
 * @return 0 on success, negative errno on failure.
 */
int fw_update_write(const uint8_t *data, size_t len);

/**
 * @brief Finish the firmware update.
 *
 * Verifies the written image by checking its header magic.
 *
 * @return 0 on success, negative errno on failure.
 */
int fw_update_finish(void);

/**
 * @brief Cancel an in-progress firmware update.
 */
void fw_update_cancel(void);

/**
 * @brief Get the current firmware update state.
 *
 * @return Current state.
 */
enum fw_update_state fw_update_get_state(void);

/**
 * @brief Get number of bytes written so far.
 *
 * @return Bytes written.
 */
size_t fw_update_bytes_written(void);

/**
 * @brief Reboot to apply the firmware update.
 *
 * MCUboot will detect the new image in slot1 and swap it into slot0.
 *
 * @return Does not return on success.
 */
int fw_update_apply_and_reboot(void);

/**
 * @brief Initialize firmware update Redfish handlers.
 *
 * Called from main() after HTTP server init.
 *
 * @return 0 on success, negative errno on failure.
 */
int fw_update_init_redfish(void);

#endif /* FW_UPDATE_H */
