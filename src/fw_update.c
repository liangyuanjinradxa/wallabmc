/*
 * Firmware Update via WebUI
 *
 * Flash erase runs in a background thread to avoid blocking the HTTP server.
 * Flash writes are synchronous (fast, microsecond-scale per byte).
 *
 * SPDX-FileCopyrightText: © 2025-2026 Tenstorrent USA, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <string.h>
#include <stm32f7xx.h>

#include "fw_update.h"

LOG_MODULE_REGISTER(wallabmc_fw_update, LOG_LEVEL_INF);

#if !defined(PARTITION_EXISTS)
#define PARTITION_EXISTS FIXED_PARTITION_EXISTS
#define PARTITION_ID FIXED_PARTITION_ID
#define PARTITION_DEVICE FIXED_PARTITION_DEVICE
#define PARTITION_OFFSET FIXED_PARTITION_OFFSET
#define PARTITION_SIZE FIXED_PARTITION_SIZE
#endif

#define IMAGE_MAGIC 0x96f3b83d
#define FLASH_RETRY_COUNT 3

/* Background writer thread (for long-running erase only) */
#define FW_WRITER_STACK_SIZE 4096
#define FW_WRITER_PRIORITY 5

enum fw_writer_cmd { FW_CMD_ERASE, FW_CMD_FINISH, FW_CMD_CANCEL };

struct fw_writer_msg {
	enum fw_writer_cmd cmd;
};

#define FW_WRITER_QUEUE_SIZE 8
K_MSGQ_DEFINE(fw_writer_msgq, sizeof(struct fw_writer_msg), FW_WRITER_QUEUE_SIZE, 4);

static struct k_thread fw_writer_thread;
static K_THREAD_STACK_DEFINE(fw_writer_stack, FW_WRITER_STACK_SIZE);

static const struct device *flash_dev;
static enum fw_update_state fw_state = FW_UPDATE_IDLE;
static size_t fw_total_size;
static size_t fw_bytes_written;
static bool writer_active;

/* ====== Flash register helpers ====== */

/* Clear stale error flags and unlock - minimal logging */
static void flash_clear_errors(void)
{
	FLASH_TypeDef *flash = FLASH;
	uint32_t sr = flash->SR;
	uint32_t cr = flash->CR;

	if (cr & FLASH_CR_LOCK) {
		LOG_WRN("Unlock...");
		flash->KEYR = 0x45670123;
		flash->KEYR = 0xCDEF89AB;
	}
	sr = flash->SR;
	if (sr & (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
		  FLASH_SR_PGPERR | FLASH_SR_ERSERR)) {
		LOG_DBG("Clear SR=0x%08lx", (unsigned long)sr);
		flash->SR = FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
			    FLASH_SR_PGPERR | FLASH_SR_ERSERR;
	}
}

/* Wait for BSY, yielding periodically */
static int flash_wait_idle(FLASH_TypeDef *flash)
{
	for (int i = 0; i < 10000000; i++) {
		if (!(flash->SR & FLASH_SR_BSY)) {
			uint32_t err = flash->SR & (FLASH_SR_OPERR|FLASH_SR_WRPERR|
				FLASH_SR_PGAERR|FLASH_SR_PGPERR|FLASH_SR_ERSERR);
			return err ? -EIO : 0;
		}
		if ((i & 0xFFF) == 0) k_yield();
	}
	return -ETIMEDOUT;
}

/* Compute sector SNB for erase */
static uint32_t sector_to_snb(off_t offset)
{
	struct flash_pages_info info;
	if (flash_get_page_info_by_offs(flash_dev, offset, &info) < 0) return 0;
	uint32_t s = info.index;
	if (s > 11) s += 4U; /* dual-bank 2MB remap */
	return s;
}

/* Erase one sector via direct register access */
static int flash_erase_sector(off_t offset)
{
	FLASH_TypeDef *flash = FLASH;
	flash_clear_errors();
	int rc = flash_wait_idle(flash);
	if (rc < 0) return rc;

	uint32_t snb = sector_to_snb(offset);
	flash->CR = (flash->CR & ~(FLASH_CR_PSIZE|FLASH_CR_SNB)) |
		FLASH_CR_SER | (snb << FLASH_CR_SNB_Pos) | FLASH_CR_STRT;
	rc = flash_wait_idle(flash);
	flash->CR &= ~(FLASH_CR_SER|FLASH_CR_SNB);
	return rc;
}

/* Write 32 bytes (one 256-bit row), with PSIZE=00 (byte mode).
 * Exactly 32 byte writes per PG session - this is the minimum row size. */
static int flash_write_row(uint32_t offset, const uint8_t *data)
{
	FLASH_TypeDef *flash = FLASH;

	/* Unlock if needed */
	if (flash->CR & FLASH_CR_LOCK) {
		flash->KEYR = 0x45670123;
		flash->KEYR = 0xCDEF89AB;
	}
	flash->SR = FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
		    FLASH_SR_PGPERR | FLASH_SR_ERSERR;

	/* Wait for BSY only */
	while (flash->SR & FLASH_SR_BSY) { k_yield(); }

	/* Write with PSIZE=00 (byte), PG=1 */
	unsigned int key = irq_lock();
	flash->CR = (flash->CR & ~FLASH_CR_PSIZE) | FLASH_CR_PG;
	__DSB(); __ISB();

	volatile uint8_t *faddr = (volatile uint8_t *)(0x08000000UL + offset);
	faddr[0] = data[0];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[1] = data[1];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[2] = data[2];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[3] = data[3];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[4] = data[4];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[5] = data[5];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[6] = data[6];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[7] = data[7];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[8] = data[8];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[9] = data[9];  __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[10] = data[10]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[11] = data[11]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[12] = data[12]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[13] = data[13]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[14] = data[14]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[15] = data[15]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[16] = data[16]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[17] = data[17]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[18] = data[18]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[19] = data[19]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[20] = data[20]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[21] = data[21]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[22] = data[22]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[23] = data[23]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[24] = data[24]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[25] = data[25]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[26] = data[26]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[27] = data[27]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[28] = data[28]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[29] = data[29]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[30] = data[30]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}
	faddr[31] = data[31]; __DSB(); while (flash->SR & FLASH_SR_BSY) {}

	flash->CR &= ~FLASH_CR_PG;
	irq_unlock(key);

	uint32_t werr = flash->SR & (FLASH_SR_OPERR|FLASH_SR_WRPERR|
				     FLASH_SR_PGAERR|FLASH_SR_PGPERR);
	if (werr) {
		LOG_ERR("Row err SR=0x%08lx CR=0x%08lx",
			(unsigned long)flash->SR, (unsigned long)flash->CR);
		return -EIO;
	}
	return 0;
}

/* Write chunk using 32-byte rows */
static int flash_write_chunk(uint32_t offset, const uint8_t *data, size_t len)
{
	while (len >= 32) {
		int rc = flash_write_row(offset, data);
		if (rc < 0) return rc;
		offset += 32; data += 32; len -= 32;
	}
	/* Remaining bytes (partial row) — read-modify-write.
	 * Use 0xFF (erased flash value) instead of reading back the flash,
	 * because the read may go beyond the flash address range and
	 * trigger a bus fault (e.g. trailer at end of slot1).
	 */
	if (len > 0) {
		uint8_t tmp[32];
		memset(tmp, 0xff, sizeof(tmp));
		memcpy(tmp, data, len);
		return flash_write_row(offset, tmp);
	}
	return 0;
}

/* Write with retry - 32-byte rows, progress every 64KB */
static int flash_write_retry(uint32_t offset, const uint8_t *data, size_t len)
{
	FLASH->ACR |= FLASH_ACR_ARTEN | FLASH_ACR_PRFTEN;
	size_t done = 0, last = 0;
	for (int r = 0; r < FLASH_RETRY_COUNT; r++) {
		while (done < len) {
			size_t c = MIN(4096, len - done); /* 4KB chunks */
			int rc = flash_write_chunk(offset + done, data + done, c);
			if (rc < 0) { r = FLASH_RETRY_COUNT; break; }
			done += c;
			/* Let log thread run so progress prints show on serial */
			k_msleep(1);
			if (done - last >= 65536) {
				last = done;
				LOG_INF("Write %d%% (%zu/%zu KB)",
					(int)(done * 100 / len),
					done / 1024, len / 1024);
			}
		}
		if (done >= len) return 0;
		k_msleep(10);
	}
	return -EIO;
}

/* ====== Background writer thread ====== */

static void fw_writer_thread_fn(void *a1, void *a2, void *a3)
{
	ARG_UNUSED(a1); ARG_UNUSED(a2); ARG_UNUSED(a3);
	struct fw_writer_msg msg;

	while (true) {
		if (k_msgq_get(&fw_writer_msgq, &msg, K_FOREVER) != 0) continue;
		switch (msg.cmd) {
		case FW_CMD_ERASE: {
			off_t off = PARTITION_OFFSET(slot1_partition);
			size_t rem = PARTITION_SIZE(slot1_partition);
			size_t total = rem;
			flash_clear_errors();
			int ok = 0;
			while (rem > 0) {
				struct flash_pages_info inf;
				if (flash_get_page_info_by_offs(flash_dev, off, &inf) < 0) break;
				size_t sz = inf.size;
				if (sz > rem) sz = rem;
				int rc;
				for (int r = 0; r < 5; r++) {
					rc = flash_erase_sector(off);
					if (rc == 0) break;
					flash_clear_errors();
					k_msleep(50 * (r + 1));
				}
			if (rc < 0) { fw_state = FW_UPDATE_ERROR; break; }
			off += sz; rem -= sz; ok++;
			LOG_INF("Erase %d%% (%d/%zu KB)", 
				(int)((total - rem) * 100 / total),
				(int)((total - rem) / 1024), total / 1024);
			/* Let log thread run so progress prints show on serial */
			k_msleep(5);
			}
			if (fw_state != FW_UPDATE_ERROR) {
				fw_state = FW_UPDATE_IN_PROGRESS;
			}
			writer_active = true;
			LOG_INF("Erase done");
			break;
		}
		case FW_CMD_FINISH:
			if (!writer_active) break;
			writer_active = false;
			if (fw_state == FW_UPDATE_ERROR) break;
			if (fw_bytes_written != fw_total_size) {
				LOG_ERR("Incomplete %zu/%zu", fw_bytes_written, fw_total_size);
				fw_state = FW_UPDATE_ERROR; break;
			}
			{ /* Verify header magic */
				uint32_t magic;
				uint8_t b[4];
				flash_read(flash_dev, PARTITION_OFFSET(slot1_partition), b, 4);
				magic = b[0]|(b[1]<<8)|(b[2]<<16)|(b[3]<<24);
				if (magic != IMAGE_MAGIC) {
					LOG_ERR("Bad header magic 0x%08x", magic);
					fw_state = FW_UPDATE_ERROR; break;
				}
			}
		{ /* Write MCUboot trailer magic at end of slot1.
		     Slot1 ends at 2MB flash boundary, so write at slot1_end - 32
		     to keep the full 32-byte row within valid flash range. */
			uint8_t trail[32] = {
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f,
				0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6, 0x79, 0x80,
			};
			uint32_t slot_sz = PARTITION_SIZE(slot1_partition);
			uint32_t magic_off = PARTITION_OFFSET(slot1_partition) + slot_sz - 32;
			flash_write_row(magic_off, trail);
			}
			fw_state = FW_UPDATE_COMPLETE;
			LOG_INF("Update complete: %zu bytes", fw_bytes_written);
			break;
		case FW_CMD_CANCEL:
			writer_active = false;
			fw_state = FW_UPDATE_IDLE;
			fw_total_size = fw_bytes_written = 0;
			break;
		}
		k_yield();
	}
}

/* ====== Public API ====== */

int fw_update_init(void)
{
	flash_dev = NULL;
#if PARTITION_EXISTS(slot1_partition)
	flash_dev = PARTITION_DEVICE(slot1_partition);
#endif
	if (!flash_dev || !device_is_ready(flash_dev)) {
		LOG_WRN("slot1 flash not available"); return -ENODEV;
	}
	LOG_INF("Slot1 @ 0x%08lx, %zu KB",
		(unsigned long)PARTITION_OFFSET(slot1_partition),
		PARTITION_SIZE(slot1_partition) / 1024);
	return 0;
}

int fw_update_start(size_t total_size)
{
	if (total_size == 0 || total_size > FW_UPDATE_MAX_SIZE) return -EINVAL;
	if (!flash_dev) return -ENODEV;

	/* If a previous update was interrupted (NetworkError, etc.), reset state */
	if (fw_state != FW_UPDATE_IDLE) {
		fw_update_cancel();
		k_msleep(50);
	}

	static bool thread_started;
	if (!thread_started) {
		k_thread_create(&fw_writer_thread, fw_writer_stack, FW_WRITER_STACK_SIZE,
				fw_writer_thread_fn, NULL, NULL, NULL,
				FW_WRITER_PRIORITY, 0, K_NO_WAIT);
		thread_started = true;
	}

	fw_total_size = total_size;
	fw_bytes_written = 0;
	fw_state = FW_UPDATE_ERASING;

	struct fw_writer_msg msg = { .cmd = FW_CMD_ERASE };
	k_msgq_put(&fw_writer_msgq, &msg, K_FOREVER);
	return 0;
}

/* Synchronous write – fast enough (byte writes, ~µs each) */
int fw_update_write(const uint8_t *data, size_t len)
{
	if (fw_state == FW_UPDATE_ERASING) {
		LOG_INF("Waiting for erase...");
		int t = 60000;
		while (fw_state == FW_UPDATE_ERASING && t > 0) { k_msleep(10); t -= 10; }
		if (fw_state == FW_UPDATE_ERASING) return -ETIMEDOUT;
	}
	if (fw_state != FW_UPDATE_IN_PROGRESS) return -EINVAL;

	uint32_t flash_offset = PARTITION_OFFSET(slot1_partition) + fw_bytes_written;
	int rc = flash_write_retry(flash_offset, data, len);
	if (rc < 0) { fw_state = FW_UPDATE_ERROR; return rc; }
	fw_bytes_written += len;
	return 0;
}

int fw_update_finish(void)
{
	struct fw_writer_msg msg = { .cmd = FW_CMD_FINISH };
	k_msgq_put(&fw_writer_msgq, &msg, K_FOREVER);
	for (int i = 0; i < 100; i++) {
		if (fw_state == FW_UPDATE_COMPLETE || fw_state == FW_UPDATE_ERROR) break;
		k_msleep(50);
	}
	if (fw_state == FW_UPDATE_ERROR) return -EIO;
	if (fw_state != FW_UPDATE_COMPLETE) return -ETIMEDOUT;
	return 0;
}

void fw_update_cancel(void)
{	struct fw_writer_msg msg = { .cmd = FW_CMD_CANCEL };
	k_msgq_put(&fw_writer_msgq, &msg, K_NO_WAIT); }

enum fw_update_state fw_update_get_state(void) { return fw_state; }
size_t fw_update_bytes_written(void) { return fw_bytes_written; }

int fw_update_apply_and_reboot(void)
{
	if (fw_state != FW_UPDATE_COMPLETE) return -EINVAL;
	LOG_WRN("Rebooting..."); k_msleep(500);
	sys_reboot(SYS_REBOOT_WARM);
	sys_reboot(SYS_REBOOT_COLD);
	return -EIO;
}

/* ====== Shell demo command ====== */
static int cmd_flash_test(const struct shell *sh, size_t argc, char **argv)
{
	off_t offset = PARTITION_OFFSET(slot1_partition);
	size_t size = PARTITION_SIZE(slot1_partition);
	shell_print(sh, "Slot1 @ 0x%08lx size=%zu", (long)offset, size);

	shell_print(sh, "Erase...");
	flash_clear_errors();
	off_t off = offset; size_t rem = size; int ok = 0, fail = 0;
	while (rem > 0) {
		struct flash_pages_info inf;
		flash_get_page_info_by_offs(flash_dev, off, &inf);
		size_t sz = MIN(inf.size, rem);
		int rc;
		for (int r = 0; r < 3; r++) {
			rc = flash_erase_sector(off);
			if (rc == 0) break;
			flash_clear_errors(); k_msleep(100);
		}
		if (rc < 0) { fail++; } else { ok++; }
		off += sz; rem -= MIN(rem, sz); k_yield();
	}
	shell_print(sh, "Erase: %dOK %dFAIL", ok, fail);
	if (fail) return -EIO;

	uint8_t pat[256];
	for (int i = 0; i < 256; i++) pat[i] = i;
	int rc = flash_write_chunk(offset, pat, 256);
	if (rc < 0) { shell_error(sh, "Write fail %d", rc); return rc; }

	shell_print(sh, "Verify 256 bytes...");
	uint8_t buf[256];
	memcpy(buf, (void *)(0x08000000UL + offset), 256);
	int match = 1;
	for (int i = 0; i < 256; i++)
		if (buf[i] != pat[i]) { shell_error(sh, "Mismatch %d", i); match = 0; }
	shell_print(sh, match ? "✅ OK" : "❌ FAIL");
	return match ? 0 : -EIO;
}
SHELL_CMD_REGISTER(flashtest, NULL, "Erase+Write+Verify slot1", cmd_flash_test);
