#include "mc_pos_recall_port.h"
#include "stm32g4xx_hal.h"   /* HAL FLASH -- boundary module */
#include <string.h>

/** @file mc_pos_recall_port_stm32g474.c
 *  @brief STM32G474 flash backend for the position-recall journal. See ADR-067.
 *
 *  Region is located by the linker (`_pos_recall_origin`/`_pos_recall_length`, POS_RECALL MEMORY
 *  region = bank2 pg124/125, 4 KB). Reads are plain memcpy (same proven pattern as
 *  mc_flash_port / mc_persistent_store -- a single-record read of erased flash is safe here; the
 *  bootloader ECC-NMI only bit when CRC-ing a large never-written span). Bank2 pages permit
 *  read-while-write against bank1 code, and we only ever write in the slow context anyway.
 */

/* Linker-provided region bounds (see STM32G474RETX_FLASH.ld). Addresses, taken via &symbol. */
extern uint32_t _pos_recall_origin;
extern uint32_t _pos_recall_length;

#define MC_FLASH_PAGE      2048u
#define MC_FLASH_BANK2_ORG 0x08040000u   /* bank2 base (DBANK=1, 2 KB pages) */

static uint32_t region_base(void) { return (uint32_t)&_pos_recall_origin; }
static uint32_t region_len(void)  { return (uint32_t)&_pos_recall_length; }

uint32_t MC_PosRecallPort_Size(void)
{
    return region_len();
}

void MC_PosRecallPort_Read(uint32_t offset, void *dst, uint32_t len)
{
    memcpy(dst, (const void *)(region_base() + offset), len);
}

MC_Status_t MC_PosRecallPort_Program(uint32_t offset, const void *src, uint32_t len)
{
    if ((len % 8u) != 0u)             { return MC_ERR_INVALID_ARG; }
    if (offset + len > region_len())  { return MC_ERR_RANGE; }

    if (HAL_FLASH_Unlock() != HAL_OK) { return MC_ERR_FAULT; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    const uint32_t base = region_base() + offset;
    MC_Status_t result = MC_OK;
    for (uint32_t i = 0u; i < len; i += 8u)
    {
        uint64_t dw;
        memcpy(&dw, (const uint8_t *)src + i, 8u);   /* avoid unaligned 64-bit load */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, base + i, dw) != HAL_OK)
        {
            result = MC_ERR_FAULT;
            break;
        }
    }
    HAL_FLASH_Lock();
    return result;
}

MC_Status_t MC_PosRecallPort_EraseAll(void)
{
    const uint32_t base   = region_base();
    const uint32_t npages = region_len() / MC_FLASH_PAGE;
    /* POS_RECALL lives wholly in bank2 (0x0807E000). Derive the first page from the base. */
    const uint32_t first_page = (base - MC_FLASH_BANK2_ORG) / MC_FLASH_PAGE;

    if (HAL_FLASH_Unlock() != HAL_OK) { return MC_ERR_FAULT; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0u;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_2;
    ei.Page      = first_page;
    ei.NbPages   = npages;

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&ei, &page_err);
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? MC_OK : MC_ERR_FAULT;
}
