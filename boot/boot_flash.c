/*
 * boot_flash — see boot_flash.h.
 *
 * Uses the STM32 HAL FLASH driver directly (same approach as
 * src/mc_flash_port_stm32g474.c). The doubleword accumulator handles
 * segments that don't fall on 8-byte boundaries (segment payload is up to
 * ~49 bytes, so most straddle a dword).
 */

#include "boot_flash.h"

#include "stm32g4xx_hal.h"

#include <string.h>

/* Dual-bank page layout of the app region (2 KB pages, DBANK=1):
 *   bank1 pages 17..127  (0x08008800..0x0803FFFF)  -> 111 pages
 *   bank2 pages 0..125   (0x08040000..0x0807EFFF)  -> 126 pages
 * Boot-flag (bank1 pg16) and config A/B (bank2 pg126/127) are deliberately
 * excluded so an update preserves them. */
#define MC_FLASH_PAGE       2048u  /* 2 KB page */
#define APP_B1_FIRST_PAGE   17u
#define APP_B1_NPAGES       111u   /* pages 17..127 */
#define APP_B2_FIRST_PAGE   0u
#define APP_B2_NPAGES       126u   /* pages 0..125 */

static boot_flash_state_t s_state;
static uint32_t           s_cursor;          /* next byte address to write */
static uint32_t           s_total_expected;  /* total_bytes from PROG_START */
static uint32_t           s_bytes_written;   /* bytes actually programmed */

static uint8_t            s_dw_buf[8];
static uint8_t            s_dw_fill;

/* Bytewise CRC32 (IEEE 802.3, poly 0xEDB88320) — same as boot_flag.c /
 * mc_boot_meta.c. No table; fine at update-flow speeds. */
static uint32_t crc32_calc(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1u));
        }
    }
    return ~crc;
}

/* Erase a consecutive page span within one bank. Returns true on success. */
static bool erase_span(uint32_t bank, uint32_t first_page, uint32_t npages)
{
    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0u;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = bank;
    ei.Page      = first_page;
    ei.NbPages   = npages;
    return HAL_FLASHEx_Erase(&ei, &page_err) == HAL_OK;
}

void boot_flash_init(void)
{
    s_state          = BOOT_FLASH_IDLE;
    s_cursor         = BOOT_FLASH_APP_BASE;
    s_total_expected = 0u;
    s_bytes_written  = 0u;
    s_dw_fill        = 0u;
    memset(s_dw_buf, 0xFFu, sizeof(s_dw_buf));
}

boot_flash_state_t boot_flash_get_state(void) { return s_state; }

bool boot_flash_begin(uint32_t total_bytes)
{
    if (s_state != BOOT_FLASH_IDLE && s_state != BOOT_FLASH_FAULT) return false;
    if (total_bytes == 0u || total_bytes > BOOT_FLASH_APP_MAX_LEN) {
        s_state = BOOT_FLASH_FAULT;
        return false;
    }

    s_state = BOOT_FLASH_ERASING;
    if (HAL_FLASH_Unlock() != HAL_OK) { s_state = BOOT_FLASH_FAULT; return false; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    /* Erase ONLY the pages the image will occupy (from APP_BASE = bank1 page 17,
     * spilling into bank2 for images > ~222 KB) -- NOT the whole 474 KB region.
     * A full-region erase is ~237 pages / ~10 s and overruns the master's INIT
     * timeout (REQ-0015 bring-up). Old data beyond the new image is never
     * executed (the app is total_bytes long) and never CRC'd (0x1F56 covers
     * bytes-written only). */
    uint32_t npages = (total_bytes + (MC_FLASH_PAGE - 1u)) / MC_FLASH_PAGE;
    bool ok = true;
    if (npages > 0u) {
        uint32_t b1_avail = 128u - APP_B1_FIRST_PAGE;                    /* pages 17..127 = 111 */
        uint32_t b1 = (npages < b1_avail) ? npages : b1_avail;
        ok = erase_span(FLASH_BANK_1, APP_B1_FIRST_PAGE, b1);
        uint32_t rem = npages - b1;
        if (ok && rem > 0u) {
            uint32_t b2 = (rem < APP_B2_NPAGES) ? rem : APP_B2_NPAGES;   /* cap at bank2 pg0..125 */
            ok = erase_span(FLASH_BANK_2, APP_B2_FIRST_PAGE, b2);
        }
    }
    HAL_FLASH_Lock();
    if (!ok) { s_state = BOOT_FLASH_FAULT; return false; }

    s_cursor         = BOOT_FLASH_APP_BASE;
    s_bytes_written  = 0u;
    s_total_expected = total_bytes;
    s_dw_fill        = 0u;
    memset(s_dw_buf, 0xFFu, sizeof(s_dw_buf));
    s_state          = BOOT_FLASH_PROGRAMMING;
    return true;
}

bool boot_flash_write(const uint8_t *src, size_t n)
{
    if (s_state != BOOT_FLASH_PROGRAMMING) return false;
    if (src == NULL)                       { s_state = BOOT_FLASH_FAULT; return false; }
    if (s_bytes_written + n > s_total_expected) { s_state = BOOT_FLASH_FAULT; return false; }

    if (HAL_FLASH_Unlock() != HAL_OK) { s_state = BOOT_FLASH_FAULT; return false; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    while (n > 0u) {
        size_t room = 8u - s_dw_fill;
        size_t take = (n < room) ? n : room;
        memcpy(&s_dw_buf[s_dw_fill], src, take);
        s_dw_fill       += (uint8_t)take;
        src             += take;
        n               -= take;
        s_bytes_written += (uint32_t)take;

        if (s_dw_fill == 8u) {
            uint64_t v;
            memcpy(&v, s_dw_buf, sizeof(v));
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, s_cursor, v) != HAL_OK) {
                HAL_FLASH_Lock();
                s_state = BOOT_FLASH_FAULT;
                return false;
            }
            s_cursor += 8u;
            s_dw_fill = 0u;
            memset(s_dw_buf, 0xFFu, sizeof(s_dw_buf));
        }
    }

    /* Last write completes the image — flush the tail, padded to a dword. */
    if (s_bytes_written == s_total_expected && s_dw_fill != 0u) {
        uint64_t v;
        memcpy(&v, s_dw_buf, sizeof(v));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, s_cursor, v) != HAL_OK) {
            HAL_FLASH_Lock();
            s_state = BOOT_FLASH_FAULT;
            return false;
        }
        s_cursor += 8u;
        s_dw_fill = 0u;
    }

    HAL_FLASH_Lock();
    return true;
}

bool boot_flash_verify(uint32_t expected_crc32)
{
    if (s_state != BOOT_FLASH_PROGRAMMING)      return false;
    if (s_bytes_written != s_total_expected)    { s_state = BOOT_FLASH_FAULT; return false; }
    s_state = BOOT_FLASH_VERIFYING;
    uint32_t crc = crc32_calc((const uint8_t *)BOOT_FLASH_APP_BASE, s_bytes_written);
    if (crc != expected_crc32) { s_state = BOOT_FLASH_FAULT; return false; }
    s_state = BOOT_FLASH_IDLE;
    return true;
}

uint32_t boot_flash_current_image_crc32(void)
{
    /* CRC only the bytes programmed this session. NEVER CRC the whole region --
     * it contains never-programmed (erased) flash, and reading erased flash on
     * STM32G4 raises an ECC NMI (hangs the bootloader). 0 = "no image / unknown"
     * (the PC tool treats it as such). */
    if (s_bytes_written == 0u) { return 0u; }
    return crc32_calc((const uint8_t *)BOOT_FLASH_APP_BASE, s_bytes_written);
}

void boot_flash_abort(void)
{
    s_state          = BOOT_FLASH_IDLE;
    s_cursor         = BOOT_FLASH_APP_BASE;
    s_bytes_written  = 0u;
    s_total_expected = 0u;
    s_dw_fill        = 0u;
}
