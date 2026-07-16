/*
 * boot_flag — see boot_flag.h. Reads the persistent flag without linking the
 * app's mc_persistent_store (which would drag the whole OD/scheduler in). The
 * persist header format is duplicated here — keep in sync with
 * src/mc_boot_meta.c if the layout ever changes.
 */

#include "boot_flag.h"

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <string.h>

/* Must match src/mc_boot_meta.c: PERSIST_MAGIC + boot_flag_header_t + magics. */
#define PERSIST_MAGIC           0x54535250u  /* "PRST" LE */
#define BOOT_META_STAY_MAGIC    0xB007107Du  /* see mc_boot_meta.h */
#define BOOT_META_BLOB_VERSION  1u

/* Boot-flag page: bank1 page 16, first 2 KB above the 32 KB bootloader. Not
 * in the app-erase range, so it survives every firmware update. */
#define BOOT_FLAG_BASE_ADDR     0x08008000u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t payload_size_bytes;
    uint32_t crc32;
} boot_flag_header_t;

_Static_assert(sizeof(boot_flag_header_t) == 16, "boot flag header must be 16 B");

/* Bytewise CRC32 (IEEE 802.3, poly 0xEDB88320) — same as src/mc_boot_meta.c.
 * Duplicated so the bootloader stays self-contained; no perf concern for a
 * 4-byte payload. */
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

bool boot_flag_is_stay(void)
{
    const boot_flag_header_t *hdr = (const boot_flag_header_t *)BOOT_FLAG_BASE_ADDR;
    if (hdr->magic != PERSIST_MAGIC)                 return false;
    if (hdr->version != BOOT_META_BLOB_VERSION)      return false;
    if (hdr->payload_size_bytes != sizeof(uint32_t)) return false;

    const uint8_t *payload = (const uint8_t *)(BOOT_FLAG_BASE_ADDR + sizeof(boot_flag_header_t));
    if (crc32_calc(payload, sizeof(uint32_t)) != hdr->crc32) return false;

    uint32_t magic;
    memcpy(&magic, payload, sizeof(magic));
    return magic == BOOT_META_STAY_MAGIC;
}

void boot_flag_set_stay(void)
{
    uint32_t magic = BOOT_META_STAY_MAGIC;
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

    if (HAL_FLASH_Unlock() != HAL_OK) { return; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0u;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_1;
    ei.Page      = 16u;                       /* boot-flag page (0x08008000) */
    ei.NbPages   = 1u;
    if (HAL_FLASHEx_Erase(&ei, &page_err) == HAL_OK) {
        for (uint32_t i = 0u; i < sizeof(buf); i += 8u) {
            uint64_t dw;
            memcpy(&dw, &buf[i], 8u);
            (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_FLAG_BASE_ADDR + i, dw);
        }
    }
    HAL_FLASH_Lock();
}
