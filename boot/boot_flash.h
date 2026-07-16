/*
 * boot_flash — write the incoming firmware image to the app flash region +
 * verify it. Motor-specific: STM32G474 dual-bank, 2 KB pages (DBANK=1),
 * mirrors src/mc_flash_port_stm32g474.c's HAL usage.
 *
 * Owns the app region 0x08008800..0x0807EFFF (~474 KB):
 *   bank1 pages 17..127  (0x08008800..0x0803FFFF)
 *   bank2 pages 0..125   (0x08040000..0x0807EFFF)
 * The erase spans exactly this range — it does NOT touch the boot-flag page
 * (bank1 pg16) or the config persist A/B pages (bank2 pg126/127), so a
 * firmware update preserves the boot flag and every PERSIST config value.
 *
 * State machine mirrors MC_IF_FLASH_* (mc_if_od.h):
 *   IDLE / ERASING / PROGRAMMING / VERIFYING / FAULT.
 */

#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    BOOT_FLASH_IDLE        = 0,
    BOOT_FLASH_ERASING     = 1,
    BOOT_FLASH_PROGRAMMING = 2,
    BOOT_FLASH_VERIFYING   = 3,
    BOOT_FLASH_FAULT       = 4,
} boot_flash_state_t;

/* App region boundaries. Must match STM32G474RETX_FLASH.ld (relocated app)
 * and the boot-flag / config page reservations. */
#define BOOT_FLASH_APP_BASE     0x08008800u
#define BOOT_FLASH_APP_END      0x0807F000u   /* one past last byte = config persist base */
#define BOOT_FLASH_APP_MAX_LEN  (BOOT_FLASH_APP_END - BOOT_FLASH_APP_BASE)

void               boot_flash_init(void);
boot_flash_state_t boot_flash_get_state(void);

/* Begin a session: erase every page in the app region, seed the write cursor
 * at APP_BASE, transition to PROGRAMMING. Blocking (several seconds). */
bool boot_flash_begin(uint32_t total_bytes);

/* Append n bytes to the running write. n need not be dword-aligned; the tail
 * is buffered until 8 accumulate. false + FAULT on any HAL error / overflow. */
bool boot_flash_write(const uint8_t *src, size_t n);

/* CRC32 over the bytes written so far vs expected. IDLE on match, FAULT on
 * mismatch. (Optional on-chip check — the PC tool also verifies via 0x1F56.) */
bool boot_flash_verify(uint32_t expected_crc32);

/* CRC32 over the currently-installed app image — answers OD reads of 0x1F56
 * program_software_id. */
uint32_t boot_flash_current_image_crc32(void);

/* Abort a session: forget cursor + state, back to IDLE. Does not erase. */
void boot_flash_abort(void);

#endif
