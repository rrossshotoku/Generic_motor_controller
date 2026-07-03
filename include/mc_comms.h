#ifndef MC_COMMS_H
#define MC_COMMS_H
#include <stdint.h>
#include <stdbool.h>

/** @file mc_comms.h
 *  @brief Inter-MCU SPI protocol handler (HAL-free) — slave side. See ADR-016.
 *
 *  Implements the shared contract (../Generic_axis_controller/Generic_axis_controller/Interface/mc_if_protocol.h + mc_if_od.h):
 *  decode a received 64-byte frame, dispatch cyclic commands and OD read/write to the OD, build
 *  the cyclic telemetry frame (with the configurable 0x2A00 map), and the command dead-man
 *  watchdog. The SPI2 DMA wiring lives in the STM32 boundary (mc_spi_slave_stm32g474.c, F2b);
 *  this module is pure logic and host/watch-window inspectable.
 */

/** @brief Observability counters (watch window). */
typedef struct
{
    uint32_t cyclic_cmds;
    uint32_t od_reads;
    uint32_t od_writes;
    uint32_t frames_crc_err;
    uint32_t frames_bad;
    uint8_t  map_count;
    uint8_t  map_version;
    bool     cmd_timeout;
} MC_CommsStats_t;

extern volatile MC_CommsStats_t g_comms_stats;

/** @brief Initialise the protocol handler (clears the telemetry map + watchdog). */
void MC_Comms_Init(void);

/** @brief Build the frame to send before the first transaction (idle telemetry). */
void MC_Comms_BuildIdle(uint8_t *tx);

/**
 * @brief Process one received frame and produce the frame to send on the NEXT transaction.
 *        Both buffers are MC_IF_FRAME_SIZE bytes. Called per SPI transaction by the boundary.
 */
void MC_Comms_HandleTransaction(const uint8_t *rx, uint8_t *tx_next);

/**
 * @brief Command dead-man: call from the slow loop (~100 Hz). Returns true if no valid cyclic
 *        command has arrived within MC_IF_COMMAND_TIMEOUT_MS (caller should quick-stop).
 */
bool MC_Comms_CommandTimedOut(void);

/**
 * @brief Set the movement_status bitfield (MC_IF_MOVE_*) published in the cyclic status header.
 *        Called by the scheduler each medium tick (REQ-0013 / ADR-033).
 */
void MC_Comms_SetMovementStatus(uint16_t status);

#endif /* MC_COMMS_H */
