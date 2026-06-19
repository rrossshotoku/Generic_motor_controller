#ifndef MC_SPI_PROTOCOL_H
#define MC_SPI_PROTOCOL_H
#include "mc_types.h"
#include "mc_config.h"

/** @file mc_spi_protocol.h
 *  @brief Robust framed SPI protocol between network MCU and motor-control MCU.
 *  @ingroup mc_spi
 */

#define MC_SPI_SYNC_WORD (0xA55Au)

typedef enum
{
    MC_SPI_MSG_CYCLIC_CMD      = 0x01,
    MC_SPI_MSG_CYCLIC_STATUS   = 0x02,
    MC_SPI_MSG_OD_READ_REQ     = 0x10,
    MC_SPI_MSG_OD_READ_RESP    = 0x11,
    MC_SPI_MSG_OD_WRITE_REQ    = 0x12,
    MC_SPI_MSG_OD_WRITE_RESP   = 0x13,
    MC_SPI_MSG_HEARTBEAT       = 0x20,
    MC_SPI_MSG_ERROR           = 0x7F
} MC_SpiMessageType_t;

typedef struct __attribute__((packed))
{
    uint16_t sync;
    uint8_t version;
    uint8_t message_type;
    uint16_t payload_length;
    uint16_t sequence;
    uint16_t header_crc;
} MC_SpiFrameHeader_t;

typedef struct __attribute__((packed))
{
    uint16_t payload_crc;
} MC_SpiFrameFooter_t;

typedef struct __attribute__((packed))
{
    uint16_t controlword;
    int8_t mode_of_operation;
    int32_t target_position;
    int32_t target_velocity;
    int32_t target_torque_current;
    uint32_t profile_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t command_counter;
} MC_SpiCyclicCommand_t;

typedef struct __attribute__((packed))
{
    uint16_t statusword;
    int8_t mode_display;
    int32_t position_actual;
    int32_t velocity_actual;
    int32_t torque_current_actual;
    uint16_t error_code;
    uint32_t status_counter;
} MC_SpiCyclicStatus_t;

typedef struct __attribute__((packed))
{
    uint16_t index;
    uint8_t subindex;
    uint8_t expected_type;
} MC_SpiOdReadReq_t;

typedef struct __attribute__((packed))
{
    uint16_t index;
    uint8_t subindex;
    uint8_t type;
    uint16_t data_length;
    uint8_t data[8];
} MC_SpiOdWriteReq_t;

uint16_t MC_Spi_Crc16(const uint8_t *data, uint32_t length);
MC_Status_t MC_Spi_EncodeFrame(uint8_t msg_type, uint16_t seq,
                               const void *payload, uint16_t payload_len,
                               uint8_t *frame, uint16_t frame_capacity,
                               uint16_t *frame_len);
MC_Status_t MC_Spi_DecodeFrame(const uint8_t *frame, uint16_t frame_len,
                               uint8_t *msg_type, uint16_t *seq,
                               const uint8_t **payload, uint16_t *payload_len);

#endif
