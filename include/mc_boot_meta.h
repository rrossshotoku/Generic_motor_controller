/**
 * @file mc_boot_meta.h
 * @brief App side of the dual-bootloader handshake (REQ-0015 Phase 2, ADR-064).
 *
 * A persistent 32-bit magic in the boot-flag page (0x08008000, bank1 page 16 —
 * OUTSIDE the app-erase range) coordinates entry into the field-update
 * bootloader. The bootloader (boot/) reads this flag; only the app writes it.
 *
 * Flag states (wrapped in a 16-byte persist header + CRC32 on flash):
 *   0xB007107D  STAY   — app asked to update; the bootloader stays resident on
 *                        the next boot and serves a segmented firmware push.
 *   0x00000000  CLEAR  — app confirmed healthy; the bootloader validates +
 *                        jumps to the app on the next boot.
 *   anything else       treated as CLEAR (blank/corrupt → run the app).
 *
 * Sequence:
 *   1. PC tool writes 0x1F51:1 = MC_IF_PROG_START to the motor (via the CMC).
 *   2. mc_comms intercepts it → MC_BootMeta_EnterBootloader(): write STAY, reset.
 *   3. Bootloader boots, sees STAY, serves the download, jumps to the new app.
 *   4. New app runs; after MC_BOOT_META_HEALTHY_MS of runtime, MC_BootMeta_Tick
 *      writes CLEAR.
 *   5. Any later power-cycle without step 1 → bootloader sees CLEAR → runs app.
 *
 * Brick-proof: if the new app crashes before step 4, the flag stays STAY and
 * the next reboot re-enters the bootloader — the operator can retry with no
 * JTAG. That is why the APP clears the flag (not the bootloader).
 */

#ifndef MC_BOOT_META_H
#define MC_BOOT_META_H

#include <stdint.h>
#include <stdbool.h>

#define MC_BOOT_META_STAY_MAGIC   (0xB007107Du)
#define MC_BOOT_META_CLEAR_MAGIC  (0x00000000u)

/* Runtime the app must accumulate before declaring itself healthy and clearing
 * the flag. Long enough to survive normal bring-up (SPI link-up, first cyclic
 * exchange) but short enough that a crash loop is caught within a few boots. */
#define MC_BOOT_META_HEALTHY_MS   (5000u)

/* Read the flag once at boot (call from framework init). Safe if the blob is
 * missing/corrupt — treated as CLEAR. */
void MC_BootMeta_Init(void);

/* Was the flag STAY at boot? (observability). */
bool MC_BootMeta_FlagWasSetAtBoot(void);

/* Call periodically (slow loop). After MC_BOOT_META_HEALTHY_MS of runtime, if
 * the flag was set at boot, writes CLEAR. No-op once cleared or if it was
 * already CLEAR (avoids flash wear). */
void MC_BootMeta_Tick(void);

/* Request bootloader entry from an ISR/any context. Sets a flag; the actual
 * flash write + reset happens in the next MC_BootMeta_Tick() (slow loop) so no
 * flash write runs in the SPI/fast/medium ISR context. This is what the OD
 * write handler for 0x1F51:1 = MC_IF_PROG_START calls. */
void MC_BootMeta_RequestEnterBootloader(void);

/* Write STAY and NVIC_SystemReset(). MUST be called only from the slow/
 * supervisory context (it erases + programs flash). Does not return on success;
 * returns (without resetting) if the flash write fails. Prefer
 * MC_BootMeta_RequestEnterBootloader() from ISR/command contexts. */
void MC_BootMeta_EnterBootloader(void);

#endif /* MC_BOOT_META_H */
