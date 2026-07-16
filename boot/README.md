# Motor-MCU field-update bootloader (REQ-0015 Phase 2)

A separately-linked 32 KB binary at `0x08000000` that serves firmware updates
over the inter-MCU SPI2 link (segmented SDO, protocol v5) and jumps to the app.
Mirrors the CMC-side bootloader (`../../Generic_axis_controller/.../boot/`),
adapted to the motor's SPI-slave transport and dual-bank flash.

## Flash map (STM32G474RE, 512 KB, dual-bank, 2 KB pages)

| Region | Address | Size | Pages | Erased by update? |
|---|---|---|---|---|
| Bootloader | `0x08000000` | 32 KB | bank1 0–15 | no |
| Boot-flag page | `0x08008000` | 2 KB | bank1 16 | no |
| **App** (relocated) | `0x08008800` | 474 KB | bank1 17–127 + bank2 0–125 | **yes — only this** |
| Config persist A/B | `0x0807F000` | 4 KB | bank2 126/127 | **no — untouched** |

The `PROG_START` erase covers **only** the app region, so the boot flag and
every `PERSIST` config value survive updates. The app's `STM32G474RETX_FLASH.ld`
`ORIGIN` is moved to `0x08008800`; `main()` sets `SCB->VTOR = 0x08008800`.

## Boot flag (brick-proof handshake)

A 32-bit magic wrapped in a 16-byte persist header (`"PRST"` + version +
payload_size + CRC32) on the boot-flag page:

- `0xB007107D` **STAY** — serve an update on next boot.
- `0x00000000` **CLEAR** — jump to the app.
- anything else → treated as CLEAR.

Only the **app** writes the flag (`src/mc_boot_meta.c`): STAY on
`0x1F51:1 = PROG_START`, CLEAR after `MC_BOOT_META_HEALTHY_MS` (5 s) of runtime.
The bootloader never writes it. If a freshly-programmed app crashes before the
healthy window, the flag stays STAY → next reboot re-enters the bootloader.

## Build

```sh
make -C boot                       # -> boot/build/boot.bin (+ .elf/.hex/.map)
make -C boot TOOLCHAIN_BIN=/path   # if your arm-none-eabi bin dir differs
```

Reuses the app's `Core/`, `Drivers/` (HAL), and `startup_stm32g474retx.s`;
`system_stm32g4xx.c` is compiled with `VECT_TAB_OFFSET=0U` so the bootloader's
VTOR stays at `0x08000000`. It ships its own `boot_spi.c` (SPI2 slave, no DMA)
and `boot_stubs.c` (clock config + minimal IRQ handlers).

## Flash (one-time provisioning, JTAG/SWD)

```
boot/build/boot.bin   -> 0x08000000     (bootloader)
<app>.bin             -> 0x08008800     (relocated app)
```

Flash the app as a **raw `.bin`** at `0x08008800`, NOT the `.elf`: the app's
loadable segment is padded down to the sector boundary, so flashing the `.elf`
writes its ELF header into the boot-flag page at `0x08008000`. Harmless (the
bootloader reads it as non-`PRST` → CLEAR) but unclean. If you do flash the
`.elf`, erase the flag-page sector afterwards.

Example (STM32CubeProgrammer CLI, ST-Link):
```
STM32_Programmer_CLI -c port=SWD mode=UR \
  -w boot/build/boot.bin 0x08000000 \
  -w <app>.bin 0x08008800 -rst
# flag page reads 0xFF -> CLEAR on first boot; the app manages it thereafter.
```

## End-to-end update (no JTAG)

1. PC tool → CMC → motor: `OD_WRITE(0x1F51:1 = PROG_START)`.
2. App writes STAY, `NVIC_SystemReset()`.
3. Bootloader boots, sees STAY, serves via SPI (node_state → `BOOTLOADER` 0x07).
4. `DOWNLOAD_INIT` (erases app region, blocking ~seconds) → N × `SEGMENT`
   (toggle + last-segment, write-through to flash).
5. `PROG_VERIFY` (PC compares `0x1F56` live CRC32 to the `.bin` CRC).
6. `PROG_COMMIT` → bootloader sends OK, drains ~100 ms, **jumps** to the app
   (does **not** reset — that would deadlock re-reading STAY).
7. New app runs, clears the flag after 5 s. Later reboots go straight to the app.

Interrupted upload (power yanked mid-download): the flag is still STAY, so the
next boot re-enters the bootloader and the PC tool retries. No JTAG required.

## Transport note

The bootloader is an SPI **slave**: `boot_main.c` runs a blocking
`HAL_SPI_TransmitReceive` exchange loop and `boot_od_handle_transaction()`
parses each frame + stages the next (same pipelined-by-one model as the app's
`mc_comms.c`; the CMC master correlates by sequence). This is simpler than the
app's DMA double-buffer — the update flow is request/response and low-rate. If
SPI timing proves marginal at higher master rates, move to the DMA +
`HAL_SPI_TxRxCpltCallback` model from `mc_spi_slave_stm32g474.c`.

## On-target verification checklist

- [ ] `make -C boot` links; `.bin` ≤ 32 KB; app `.bin` ≤ 474 KB.
- [ ] Cold boot with CLEAR flag jumps to app (no `HAL_Delay` hang → confirms
      `__enable_irq()` before the jump).
- [ ] `PROG_START` reboots into bootloader; `CYCLIC_STATUS.node_state == 0x07`.
- [ ] Full download → `0x1F56` CRC matches the `.bin`.
- [ ] `PROG_COMMIT` runs the new app; flag auto-clears after 5 s.
- [ ] Power-cycle mid-download recovers into the bootloader.
- [ ] Config (gains/cal) unchanged across an update.
