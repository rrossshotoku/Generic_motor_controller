#include "mc_flash_port.h"
#include "stm32g4xx_hal.h"   /* HAL FLASH -- boundary module */
#include <string.h>

/** @file mc_flash_port_stm32g474.c
 *  @brief STM32G474 flash-slot backend for the NV store. See ADR-010.
 *
 *  Dual-bank, 2 KB pages (DBANK=1, confirmed from bldc_axis_controller's flash_storage). The NV
 *  region is the top 4 KB of flash -- bank 2 pages 126/127 -- reserved in the linker script so
 *  code never lands there. Putting NV in bank 2 (opposite the code in bank 1) also permits
 *  read-while-write, so an erase/program need not stall the running loops (we still gate saves
 *  on the power stage being off for now).
 */

#define MC_NV_SLOT_SIZE   2048u
#define MC_NV_SLOT0_ADDR  0x0807F000u   /* bank 2 page 126 */
#define MC_NV_SLOT1_ADDR  0x0807F800u   /* bank 2 page 127 */
#define MC_NV_SLOT0_PAGE  126u
#define MC_NV_SLOT1_PAGE  127u

static uint32_t slot_addr(uint8_t slot) { return (slot == 0u) ? MC_NV_SLOT0_ADDR : MC_NV_SLOT1_ADDR; }
static uint32_t slot_page(uint8_t slot) { return (slot == 0u) ? MC_NV_SLOT0_PAGE : MC_NV_SLOT1_PAGE; }

uint32_t MC_FlashPort_SlotSize(void)
{
    return MC_NV_SLOT_SIZE;
}

void MC_FlashPort_Read(uint8_t slot, uint32_t offset, void *dst, uint32_t len)
{
    memcpy(dst, (const void *)(slot_addr(slot) + offset), len);
}

MC_Status_t MC_FlashPort_EraseSlot(uint8_t slot)
{
    if (slot >= MC_FLASH_NV_SLOTS) { return MC_ERR_INVALID_ARG; }

    if (HAL_FLASH_Unlock() != HAL_OK) { return MC_ERR_FAULT; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef ei;
    uint32_t page_err = 0u;
    ei.TypeErase = FLASH_TYPEERASE_PAGES;
    ei.Banks     = FLASH_BANK_2;
    ei.Page      = slot_page(slot);
    ei.NbPages   = 1u;

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&ei, &page_err);
    HAL_FLASH_Lock();
    return (st == HAL_OK) ? MC_OK : MC_ERR_FAULT;
}

MC_Status_t MC_FlashPort_Program(uint8_t slot, uint32_t offset, const void *src, uint32_t len)
{
    if (slot >= MC_FLASH_NV_SLOTS) { return MC_ERR_INVALID_ARG; }
    if ((len % 8u) != 0u)          { return MC_ERR_INVALID_ARG; }

    if (HAL_FLASH_Unlock() != HAL_OK) { return MC_ERR_FAULT; }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    const uint32_t base = slot_addr(slot) + offset;
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
