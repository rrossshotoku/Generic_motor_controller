/*
 * boot_seg_sdo — CiA-301 §7.2.4.3 segmented-SDO receiver for firmware
 * download. Only accepts writes to 0x1F50:1 program_data.
 *
 * Transport-agnostic: lifted from the CMC's boot/boot_seg_sdo.c unchanged —
 * it knows nothing about UDP vs SPI, it just consumes the decoded
 * MC_IfOdDownload* bodies and drives boot_flash.
 *
 *   IDLE     -- (INIT ok)                        --> IN_PROGRESS
 *   IN_PROG  -- (SEGMENT good toggle, more)      --> IN_PROGRESS
 *   IN_PROG  -- (SEGMENT good toggle, last=1)    --> IDLE (writes committed)
 *   any      -- (INIT during session)            --> reject BOOTLOADER_BUSY
 *   any      -- (SEGMENT wrong toggle)           --> reply the OTHER toggle,
 *                                                    don't advance the cursor
 *
 * Each segment's payload is handed straight to boot_flash_write — the whole
 * image is never buffered (RAM = O(one segment)).
 */

#ifndef BOOT_SEG_SDO_H
#define BOOT_SEG_SDO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "mc_if_protocol.h"

void boot_seg_sdo_init(void);

/* Open a download session (main-loop pump only, after boot_flash_begin). */
void boot_seg_sdo_start(void);

/* True while a session is open and ready to accept segments. */
bool boot_seg_sdo_active(void);

/* Handle a MC_IF_MSG_OD_DOWNLOAD_SEGMENT body. body_len = payload bytes
 * (the sizeof-struct portion incl. seg_length + data). */
size_t boot_seg_sdo_on_segment(const MC_IfOdDownloadSegment_t *seg,
                               uint8_t body_len,
                               MC_IfOdDownloadResp_t *out_resp);

/* Forget any in-flight session (program_control STOP / ABORT). */
void boot_seg_sdo_abort(void);

#endif
