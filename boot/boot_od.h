/*
 * boot_od — OD dispatcher for the motor bootloader over the inter-MCU SPI2
 * slave link. Uses the app's frame format (MC_IfFrameHeader_t + CRC16-Modbus)
 * and handles the reduced OD subset (0x1F50/51/56/57) + the segmented-SDO
 * message types.
 *
 * boot_od_on_frame() runs in the SPI DMA transfer-complete callback (ISR) and
 * ALWAYS stages a valid mc_if frame into tx (a real response, else a HEARTBEAT
 * or a CYCLIC_STATUS with node_state=BOOTLOADER) so the CMC master never sees an
 * unframed MISO transaction. The slow PROG_START erase is deferred to
 * boot_od_pump() (main loop).
 */

#ifndef BOOT_OD_H
#define BOOT_OD_H

#include <stdint.h>
#include <stdbool.h>

void boot_od_init(void);

/* Seed an outbound buffer with a valid idle (HEARTBEAT) frame. */
void boot_od_build_idle(uint8_t *tx);

/* Per-transaction handler (called from the SPI DMA callback): parse rx, stage a
 * valid frame into tx (MC_IF_FRAME_SIZE each). Must be fast — no blocking flash
 * erase here. */
void boot_od_on_frame(const uint8_t *rx, uint8_t *tx);

/* Main-loop worker: performs the deferred app-region erase requested by a
 * DOWNLOAD_INIT. Call every main-loop iteration. */
void boot_od_pump(void);

/* True once PROG_COMMIT was accepted; boot_main drains the OK then jumps. */
bool boot_od_commit_pending(void);

#endif
