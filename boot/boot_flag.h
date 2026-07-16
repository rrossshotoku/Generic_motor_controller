/*
 * boot_flag — read the persistent "stay in bootloader" flag from the
 * boot-flag page (0x08008000, bank1 page 16).
 *
 * Mirrors the blob format that the app side (src/mc_boot_meta.c) writes:
 * a 16-byte persist header ("PRST" magic + version + payload_size + CRC32)
 * followed by a 4-byte payload (the STAY / CLEAR magic).
 *
 * Fail-safe reads: any header inconsistency, CRC mismatch, wrong size, or
 * unknown payload magic is treated as CLEAR — the bootloader then jumps to
 * the app rather than getting stuck. (Recovery for a genuinely-bad app image
 * is the "app image looks valid" check in boot_main.c.)
 *
 * The bootloader NEVER writes this flag — only the app does (sets STAY on
 * PROG_START, clears it after the healthy window). See mc_boot_meta.h.
 */

#ifndef BOOT_FLAG_H
#define BOOT_FLAG_H

#include <stdbool.h>

/* Returns true iff the flag was successfully read AND equals STAY_MAGIC. */
bool boot_flag_is_stay(void);

/* Write the STAY blob to the flag page. The bootloader calls this when it begins
 * an OTA erase so that a failed/partial download leaves the flag STAY -- the next
 * boot then stays resident instead of jumping into a half-written app. Only the
 * app ever clears the flag (after its healthy window). Main-loop context only. */
void boot_flag_set_stay(void);

#endif
