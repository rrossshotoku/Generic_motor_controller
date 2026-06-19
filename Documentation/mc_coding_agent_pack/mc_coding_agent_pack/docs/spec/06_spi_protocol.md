# SPI Inter-MCU Protocol

## Purpose

SPI connects the network MCU and motor-control MCU. It carries both:

1. Cyclic process data for real-time command/status.
2. Object read/write requests for configuration, tuning, diagnostics, and calibration.

## Protocol requirements

- Robust framed protocol.
- Version field.
- Sync word.
- Message type.
- Payload length.
- Sequence counter.
- Header CRC.
- Payload CRC.
- Error responses.
- Timeout handling.
- Unsupported-version rejection.
- Invalid-CRC rejection.

## Message types

| Value | Name | Direction | Purpose |
|---:|---|---|---|
| 0x01 | CYCLIC_CMD | network -> motor | controlword, mode, targets, profile limits |
| 0x02 | CYCLIC_STATUS | motor -> network | statusword, actuals, errors |
| 0x10 | OD_READ_REQ | network -> motor | read index/subindex |
| 0x11 | OD_READ_RESP | motor -> network | read result |
| 0x12 | OD_WRITE_REQ | network -> motor | write index/subindex |
| 0x13 | OD_WRITE_RESP | motor -> network | write result |
| 0x20 | HEARTBEAT | both | link supervision |
| 0x7F | ERROR | both | protocol/object error |

## Frame shape

```c
typedef struct __attribute__((packed))
{
    uint16_t sync;
    uint8_t  version;
    uint8_t  message_type;
    uint16_t payload_length;
    uint16_t sequence;
    uint16_t header_crc;
} MC_SpiFrameHeader_t;

typedef struct __attribute__((packed))
{
    uint16_t payload_crc;
} MC_SpiFrameFooter_t;
```

## Cyclic command payload

```c
typedef struct __attribute__((packed))
{
    uint16_t controlword;
    int8_t   mode_of_operation;
    int32_t  target_position;
    int32_t  target_velocity;
    int32_t  target_torque_current;
    uint32_t profile_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t command_counter;
} MC_SpiCyclicCommand_t;
```

## Cyclic status payload

```c
typedef struct __attribute__((packed))
{
    uint16_t statusword;
    int8_t   mode_display;
    int32_t  position_actual;
    int32_t  velocity_actual;
    int32_t  torque_current_actual;
    uint16_t error_code;
    uint32_t status_counter;
} MC_SpiCyclicStatus_t;
```

## Safe-state behaviour

- If cyclic command timeout occurs, fault manager shall request quick stop if feedback/control valid.
- If timeout persists or feedback invalid, severe inhibit may disable PWM/current.
- Invalid object-access requests shall return OD error but not corrupt active motion.
