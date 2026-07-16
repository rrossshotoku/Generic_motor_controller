/**
 * @file mc_boot_meta.c
 * @brief See mc_boot_meta.h. Writes/reads the boot-flag blob on its own flash
 *        page (bank1 page 16, 0x08008000) via the HAL directly — deliberately
 *        NOT through mc_flash_port (which owns the config A/B slots in bank 2).
 *
 * Blob layout (must match boot/boot_flag.c):
 *   [ 16-byte header: "PRST" magic, u16 version, u16 reserved, u32 payload_size,
 *     u32 crc32-of-payload ] [ u32 magic ]  (padded to 24 B = 3 doublewords)
 */

#include "mc_boot_meta.h"

#include "stm32g4xx_hal.h"   /* HAL FLASH + NVIC_SystemReset -- boundary module */

#include <string.h>

#define PERSIST_MAGIC          0x54535250u  /* "PRST" LE */
#define BOOT_META_BLOB_VERSION 1u

/* Boot-flag page: bank1 page 16, the 2 KB above the 32 KB bootloader. Not in
 * the app-erase range, so it survives every firmware update. */
#define BOOT_FLAG_ADDR         0x08008000u
#define BOOT_FLAG_PAGE         16u
#define BOOT_FLAG_BANK         FLASH_BANK_1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t payload_size_bytes;
    uint32_t crc32;
} boot_flag_header_t;

_Static_assert(sizeof(boot_flag_header_t) == 16, "boot flag header must be 16 B");

static bool          s_flag_was_set;      /* snapshot at boot */
static bool          s_cleared_this_boot; /* have we written CLEAR yet? */
static uint32_t      s_boot_ms;           /* HAL_GetTick() at init */
static volatile bool s_enter_pending;     /* set by the OD write (ISR), serviced in the slow loop */

/* Bytewise CRC32 (poly 0xEDB88320) — same as boot/boot_flag.c. */
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

/* Erase page 16 + program the blob (3 doublewords). Returns true on success. */
static bool write_flag(uint32_t magic)
{
    boot_flag_header_t hdr;
    hdr.magic              = PERSIST_MAGIC;
    hdr.version            = BOOT_META_BLOB_VERSION;
    hdr.reserved           = 0u;
    hdr.payload_size_bytes = sizeof(uint32_t);
    hdr.crc32              = crc32_calc((const uint8_t *)&magic, sizeof(magic));

    uint8_t buf[24];
    memset(buf, 0xFFu, sizeof(buf));
    memcpy(buf,      &hdr,   sizeof(hdr));   /* 16 B */
    memcpy(buf + 16, &magic, sizeof(magic)); /* 4 B  */

    if (HAL_FLASH_Unlock() != HAL_OK) { return false; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0u;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = BOOT_FLAG_BANK;
    ei.Page      = BOOT_FLAG_PAGE;
    ei.NbPages   = 1u;
    if (HAL_FLASHEx_Erase(&ei, &page_err) != HAL_OK) { HAL_FLASH_Lock(); return false; }

    bool ok = true;
    for (uint32_t i = 0u; i < sizeof(buf); i += 8u) {
        uint64_t dw;
        memcpy(&dw, &buf[i], 8u);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_FLAG_ADDR + i, dw) != HAL_OK) {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

/* Read the on-flash magic, or CLEAR on any failure (safe default → run app). */
static uint32_t read_flag(void)
{
    const boot_flag_header_t *hdr = (const boot_flag_header_t *)BOOT_FLAG_ADDR;
    if (hdr->magic != PERSIST_MAGIC)                 return MC_BOOT_META_CLEAR_MAGIC;
    if (hdr->version != BOOT_META_BLOB_VERSION)      return MC_BOOT_META_CLEAR_MAGIC;
    if (hdr->payload_size_bytes != sizeof(uint32_t)) return MC_BOOT_META_CLEAR_MAGIC;

    const uint8_t *payload = (const uint8_t *)(BOOT_FLAG_ADDR + sizeof(boot_flag_header_t));
    if (crc32_calc(payload, sizeof(uint32_t)) != hdr->crc32) return MC_BOOT_META_CLEAR_MAGIC;

    uint32_t magic;
    memcpy(&magic, payload, sizeof(magic));
    return magic;
}

void MC_BootMeta_Init(void)
{
    s_flag_was_set      = (read_flag() == MC_BOOT_META_STAY_MAGIC);
    s_cleared_this_boot = false;
    s_enter_pending     = false;
    s_boot_ms           = HAL_GetTick();
}

void MC_BootMeta_RequestEnterBootloader(void) { s_enter_pending = true; }

bool MC_BootMeta_FlagWasSetAtBoot(void) { return s_flag_was_set; }

void MC_BootMeta_Tick(void)
{
    /* Bootloader entry requested from the OD write handler (ISR context): do the
       STAY-flag flash write + reset HERE, in the slow/supervisory context. Flash
       writes must never run in the SPI/medium/fast ISR contexts (ADR-010 rule) --
       that is why the earlier in-ISR write failed with NOT_READY. */
    if (s_enter_pending) {
        MC_BootMeta_EnterBootloader();   /* writes STAY + resets; returns only on flash failure */
        s_enter_pending = false;         /* failed -> clear; a re-issued PROG_START retries */
    }

    if (s_cleared_this_boot) return;
    if (!s_flag_was_set)     return;
    /* The slow loop calling us IS the health evidence — if the app had
     * crashed it would have reset before reaching MC_BOOT_META_HEALTHY_MS,
     * leaving the flag STAY (brick-proof). */
    if ((uint32_t)(HAL_GetTick() - s_boot_ms) < MC_BOOT_META_HEALTHY_MS) return;

    if (write_flag(MC_BOOT_META_CLEAR_MAGIC)) {
        s_cleared_this_boot = true;
    }
    /* On failure, leave s_cleared_this_boot false → retry next tick. */
}

void MC_BootMeta_EnterBootloader(void)
{
    if (!write_flag(MC_BOOT_META_STAY_MAGIC)) {
        return;   /* caller reports NOT_READY; PC tool retries */
    }
    /* Brief spin so the OD_WRITE_RESP for the PROG_START write has time to be
     * clocked out to the master before we reset. */
    uint32_t deadline = HAL_GetTick() + 50u;
    while (HAL_GetTick() < deadline) { /* spin */ }
    NVIC_SystemReset();
    /* not reached */
}
