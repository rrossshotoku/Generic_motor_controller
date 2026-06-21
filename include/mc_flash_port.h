#ifndef MC_FLASH_PORT_H
#define MC_FLASH_PORT_H
#include "mc_types.h"

/** @file mc_flash_port.h
 *  @brief Minimal flash-slot interface for the NV parameter store (boundary seam).
 *  @ingroup mc_persistence
 *
 *  The persistent store is generic over N erase-slots (one flash page each). The STM32 boundary
 *  module (mc_flash_port_stm32g474.c) maps slots to physical pages and provides read / erase /
 *  program. Keeping this seam HAL-free lets the generic store be reasoned about and host-tested.
 */
#define MC_FLASH_NV_SLOTS 2u   /* A/B ping-pong */

/** @brief Bytes per slot (one flash page). */
uint32_t    MC_FlashPort_SlotSize(void);
/** @brief Read @p len bytes at @p offset within @p slot (memory-mapped flash). */
void        MC_FlashPort_Read(uint8_t slot, uint32_t offset, void *dst, uint32_t len);
/** @brief Erase a whole slot (one page). Blocking; call only in slow/background context. */
MC_Status_t MC_FlashPort_EraseSlot(uint8_t slot);
/** @brief Program @p len bytes (must be a multiple of 8) at @p offset within @p slot. */
MC_Status_t MC_FlashPort_Program(uint8_t slot, uint32_t offset, const void *src, uint32_t len);

#endif /* MC_FLASH_PORT_H */
