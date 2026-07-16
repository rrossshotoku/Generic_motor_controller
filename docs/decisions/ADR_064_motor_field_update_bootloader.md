# ADR-064: Motor-MCU field-update bootloader (dual-bootloader Phase 2)

- Status: Accepted
- Date: 2026-07-07
- Related: ADR-061 (adopt shared contract v5 — bootloader OD 0x1F5x, node-state,
  segmented-SDO messages, result codes), ADR-010 (NV parameter store /
  mc_flash_port), ADR-016 (inter-MCU SPI slave), REQ-0015 (spec + the 2026-07-07
  CMC-side bring-up decisions). Reference: `Generic_axis_controller/.../boot/`.

## Context

Contract v5 defined the bootloader control surface (0x1F5x, `MC_IF_PROG_*`,
`MC_IF_FLASH_*`, download messages 0x14/15/16, `MC_IF_NODE_BOOTLOADER`, new
result codes). Phase 1 (ADR-061) adopted the wire contract; the app skips the
`MC_IF_OWNER_BOOTLOADER` rows. Phase 2 implements the actual motor-side
field-update path so firmware can be pushed over the inter-MCU SPI link (via the
CMC pass-through) with no JTAG. The CMC bootloader is already done; we mirror it,
swapping the CMC's UDP/W6100 transport for the motor's SPI-slave framing and its
single-bank-style flash for the motor's dual-bank layout.

## Decision

**Flash map** (512 KB, dual-bank, 2 KB pages). A separately-linked 32 KB
bootloader owns the reset vector; the app relocates up; config is preserved:

| Region | Address | Pages | Erased by update |
|---|---|---|---|
| Bootloader | 0x08000000 (32 K) | bank1 0–15 | no |
| Boot-flag page | 0x08008000 (2 K) | bank1 16 | no |
| App (relocated) | 0x08008800 (474 K) | bank1 17–127 + bank2 0–125 | **yes (only this)** |
| Config persist A/B | 0x0807F000 (4 K) | bank2 126/127 | no |

The `PROG_START` erase is scoped to the app region only, so the boot flag and
every `PERSIST` config value survive updates. The config store is unchanged
(still bank2 pg126/127, `mc_flash_port_stm32g474.c`) — the app relocation does
not move it (fixed top-of-flash address).

**App relocation.** `STM32G474RETX_FLASH.ld` `ORIGIN` → 0x08008800, `LENGTH` →
474 K. `main()` sets `SCB->VTOR = 0x08008800` (USER CODE BEGIN 1); the bootloader
also sets VTOR before jumping. The app now boots only via the bootloader.

**Bootloader modules** (`boot/`, built by `boot/Makefile` — self-contained
arm-none-eabi build reusing the app's HAL/Core/startup):
- `boot_main.c` — flag check → jump (CLEAR/valid) or serve (STAY/bad image);
  the SPI-slave exchange loop; `boot_jump_to_app` (jump, never reset).
- `boot_flag.c` — reads the STAY flag; duplicates the persist header format.
- `boot_flash.c` — dual-bank app-region erase + doubleword program + CRC32.
- `boot_seg_sdo.c` — CiA-301 segmented-SDO receiver (lifted verbatim from CMC,
  transport-agnostic; write-through, no image buffering).
- `boot_od.c` — 0x1F5x OD dispatch + download handling using the app's frame
  format (`MC_IfFrameHeader_t` + CRC16-Modbus); deferred `PROG_COMMIT`.
- `boot_spi.c` / `boot_stubs.c` — SPI2 slave (no DMA) + clock/IRQ stubs.

**App side** (`src/mc_boot_meta.c`, wired into `mc_comms.c` + `mc_scheduler.c`):
writes STAY on `0x1F51:1 = PROG_START` then resets; clears the flag after 5 s of
healthy runtime; the OD write path intercepts the bootloader range (PROG_START →
enter; other 0x1F5x → `NOT_BOOTLOADER`).

**Mirrored critical decisions (REQ-0015, 2026-07-07):** flash trigger marker
(not RAM); app clears the flag, bootloader never touches it; `PROG_COMMIT`
**jumps** (does not reset — resetting deadlocks re-reading STAY); `__enable_irq()`
before the jump (else SysTick-blocked `HAL_Delay` hangs); write-through segments;
toggle + last-segment per INTERFACE_SPEC §7c; response-then-jump with a ~100 ms
drain; PROG_VERIFY is a no-op on-chip (PC compares the `0x1F56` live CRC32).

## Consequences

- Field updates over SPI, brick-proof (crash-before-healthy re-enters the
  bootloader), config-preserving. No contract change (v5 already covers the
  wire); no `MC_IF_PROTOCOL_VERSION` bump.
- The app can no longer boot standalone from cold (needs the bootloader at
  0x08000000). Debug/provisioning flashes both binaries; see `boot/README.md`.
- The SPI-slave serve loop is blocking (single-buffered) — adequate for the
  request/response update flow. If master-rate timing proves marginal, move to
  the DMA double-buffer model of `mc_spi_slave_stm32g474.c`.
## Bring-up fixes (2026-07-07, on hardware)

- **Flashed + validated cold-boot on target**: bootloader boots → CLEAR → jumps to
  the relocated app (uwTick advancing; the `__enable_irq()` before the jump is
  required — without it SysTick is masked and app `HAL_Delay` hangs).
- **PROG_START must defer the flash write off the SPI ISR.** `MC_Comms_HandleTransaction`
  (the OD write handler) runs in `HAL_SPI_TxRxCpltCallback` — an ISR. Calling
  `MC_BootMeta_EnterBootloader()` (erase+program the flag page) directly there
  FAILED (returned NOT_READY 0x08; the 20 kHz/1 kHz loops preempt and stall on the
  bank being erased) — so the motor never entered the bootloader and the master
  wrongly streamed segments to the running app ("malformed reply"). Fix: the ISR
  handler calls `MC_BootMeta_RequestEnterBootloader()` (sets a flag, returns OK);
  the STAY write + reset happen in `MC_BootMeta_Tick()` (slow loop) — the same
  supervisory context as `params_save` (ADR-010 rule: no flash writes off the
  slow context). Validated on hardware via SWD flag-injection: CLEAR→app-runs→
  inject→valid STAY blob written→app resets into bootloader.
- Bootloader SPI download path (INIT/SEGMENT/COMMIT) not yet exercised on target —
  needs the CMC pass-through / PC tool.
- Not yet verified on target (no hardware in this session): needs `make -C boot`
  + the on-target checklist in `boot/README.md`. The HAL-free logic
  (`boot_od`/`boot_seg_sdo`/`boot_flag`, and the app integration) host-compiles
  clean against the real contract headers.
