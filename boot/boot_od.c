/*
 * boot_od — see boot_od.h.
 *
 * Frame encode/validate mirrors src/mc_comms.c (CRC16-Modbus, MC_IfFrameHeader_t,
 * payload_crc at HEADER+plen). OD dispatch mirrors the CMC's boot/boot_od.c.
 *
 * KEY: the CMC master ticks SPI at ~1 kHz and marks EVERY transaction whose MISO
 * isn't a valid mc_if frame as an error. So boot_od_on_frame() ALWAYS stages a
 * valid frame — a real response when it has one, else a HEARTBEAT / a
 * CYCLIC_STATUS(node_state=BOOTLOADER). It runs in the SPI DMA callback (ISR),
 * so it must be fast: the multi-second PROG_START erase is deferred to
 * boot_od_pump() in the main loop (the ISR heartbeats until it completes, then
 * stages the INIT's DOWNLOAD_RESP). Per-segment flash program (~0.6 ms) fits the
 * 1 ms frame budget and runs in the ISR.
 */

#include "boot_od.h"
#include "boot_flag.h"
#include "boot_flash.h"
#include "boot_seg_sdo.h"

#include "mc_if_od.h"
#include "mc_if_protocol.h"

#include <string.h>

extern void boot_jump_to_app(void);

static volatile bool     s_commit_pending = false;

/* INIT/erase handshake: the ISR (handle_download_init) records a request; the
 * main-loop pump does the slow erase + opens the session. The INIT response is
 * derived from flash state + boot_seg_sdo_active() — no ISR/pump-shared response
 * buffer, so no race against the first segment. */
static volatile bool     s_erase_req   = false;   /* ISR -> pump: begin erase */
static volatile uint32_t s_erase_total = 0u;      /* total_length from INIT   */
static volatile bool     s_verify_req  = false;   /* ISR -> pump: (re)compute image CRC */
static volatile uint32_t s_image_crc   = 0u;      /* cached CRC32; 0x1F56 returns this  */
static volatile uint16_t s_init_seq    = 0u;      /* INIT seq to answer once the erase finishes */
static volatile bool     s_init_answer = false;   /* pump -> ISR: send the deferred INIT resp    */
static volatile uint8_t  s_init_result = 0u;      /* OK / FLASH_LOCKED for that answer           */

bool boot_od_commit_pending(void) { return s_commit_pending; }

/* ===== CRC16/Modbus + frame encode — identical to mc_comms.c ===== */
static uint16_t crc16(const uint8_t *d, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    for (uint32_t i = 0u; i < n; i++) {
        c ^= d[i];
        for (uint8_t b = 0u; b < 8u; b++) {
            c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
        }
    }
    return c;
}

static void encode(uint8_t msg_type, uint16_t seq, const void *payload, uint16_t plen, uint8_t *out)
{
    MC_IfFrameHeader_t h;
    h.sync = MC_IF_SYNC_WORD; h.version = MC_IF_PROTOCOL_VERSION;
    h.message_type = msg_type; h.payload_length = plen; h.sequence = seq; h.header_crc = 0u;
    h.header_crc = crc16((const uint8_t *)&h, MC_IF_HEADER_SIZE - 2u);

    memcpy(out, &h, MC_IF_HEADER_SIZE);
    if (plen != 0u) { memcpy(out + MC_IF_HEADER_SIZE, payload, plen); }
    const uint16_t pcrc = crc16(out + MC_IF_HEADER_SIZE, plen);
    memcpy(out + MC_IF_HEADER_SIZE + plen, &pcrc, 2u);
    const uint16_t used = (uint16_t)(MC_IF_HEADER_SIZE + plen + 2u);
    if (used < MC_IF_FRAME_SIZE) { memset(out + used, 0, MC_IF_FRAME_SIZE - used); }
}

/* Cheapest valid frame: HEARTBEAT, zero payload. Keeps CMC framing happy when we
 * have nothing to say (idle ticks, mid-erase). */
static void build_heartbeat(uint16_t seq, uint8_t *out)
{
    encode(MC_IF_MSG_HEARTBEAT, seq, NULL, 0u, out);
}

/* CYCLIC_STATUS with node_state = BOOTLOADER — the reply to a CYCLIC_CMD. Seeing
 * this makes the CMC pause cyclic commands (INTERFACE_SPEC §7a). */
static void build_status(uint16_t seq, uint8_t *out)
{
    MC_IfCyclicStatusHeader_t hdr; memset(&hdr, 0, sizeof(hdr));
    hdr.node_state     = MC_IF_NODE_BOOTLOADER;
    hdr.map_byte_count = 0u;
    uint8_t pl[MC_IF_STATUS_HEADER_SIZE];
    memcpy(pl, &hdr, MC_IF_STATUS_HEADER_SIZE);
    encode(MC_IF_MSG_CYCLIC_STATUS, seq, pl, MC_IF_STATUS_HEADER_SIZE, out);
}

void boot_od_build_idle(uint8_t *tx) { build_heartbeat(0u, tx); }

/* ===== OD dispatch ===== */
static uint16_t flash_state_to_wire(boot_flash_state_t s)
{
    switch (s) {
        case BOOT_FLASH_IDLE:        return MC_IF_FLASH_IDLE;
        case BOOT_FLASH_ERASING:     return MC_IF_FLASH_ERASING;
        case BOOT_FLASH_PROGRAMMING: return MC_IF_FLASH_PROGRAMMING;
        case BOOT_FLASH_VERIFYING:   return MC_IF_FLASH_VERIFYING;
        case BOOT_FLASH_FAULT:       return MC_IF_FLASH_FAULT;
        default:                     return MC_IF_FLASH_FAULT;
    }
}

static void handle_read(uint16_t seq, const uint8_t *pl, uint16_t plen, uint8_t *out)
{
    if (plen < sizeof(MC_IfOdReadReq_t)) { build_heartbeat(seq, out); return; }
    const MC_IfOdReadReq_t *r = (const MC_IfOdReadReq_t *)pl;

    MC_IfOdReadResp_t resp; memset(&resp, 0, sizeof(resp));
    resp.index = r->index; resp.subindex = r->subindex;

    if (r->index == 0x1F56u && r->subindex == 1u) {
        uint32_t crc = s_image_crc;   /* cached — computed in the pump on VERIFY (CRC over 87 KB is too slow for the ISR) */
        resp.type = MC_IF_T_U32; resp.result = MC_IF_OD_OK; resp.data_length = 4u;
        memcpy(resp.data, &crc, 4u);
    } else if (r->index == 0x1F57u && r->subindex == 1u) {
        uint16_t st = flash_state_to_wire(boot_flash_get_state());
        resp.type = MC_IF_T_U16; resp.result = MC_IF_OD_OK; resp.data_length = 2u;
        memcpy(resp.data, &st, 2u);
    } else {
        resp.type = r->expected_type; resp.result = MC_IF_OD_ERR_NO_OBJECT; resp.data_length = 0u;
    }
    encode(MC_IF_MSG_OD_READ_RESP, seq, &resp, (uint16_t)sizeof(resp), out);
}

static uint8_t apply_program_control(uint8_t cmd)
{
    switch (cmd) {
        case MC_IF_PROG_STOP:
        case MC_IF_PROG_ABORT:  boot_seg_sdo_abort(); return MC_IF_OD_OK;
        case MC_IF_PROG_START:  return MC_IF_OD_OK;   /* already in bootloader */
        case MC_IF_PROG_VERIFY: s_verify_req = true; return MC_IF_OD_OK;  /* pump computes the CRC; PC reads 0x1F56 */
        case MC_IF_PROG_COMMIT: {
            /* Refuse to jump to an erased/invalid app (initial SP must sit in
             * SRAM). Prevents a hard-fault if COMMIT is issued before a
             * completed download — the bootloader stays resident so the PC
             * tool can (re)download. */
            uint32_t app_sp = *(volatile uint32_t *)0x08008800u;
            if (app_sp < 0x20000000u || app_sp > 0x20020000u) {
                return MC_IF_OD_ERR_CRC;
            }
            s_commit_pending = true;
            return MC_IF_OD_OK;
        }
        default:                return MC_IF_OD_ERR_RANGE;
    }
}

static void handle_write(uint16_t seq, const uint8_t *pl, uint16_t plen, uint8_t *out)
{
    if (plen < sizeof(MC_IfOdWriteReq_t)) { build_heartbeat(seq, out); return; }
    const MC_IfOdWriteReq_t *w = (const MC_IfOdWriteReq_t *)pl;

    MC_IfOdWriteResp_t resp;
    resp.index = w->index; resp.subindex = w->subindex;
    if (w->index == 0x1F51u && w->subindex == 1u && w->data_length >= 1u) {
        resp.result = apply_program_control(w->data[0]);
    } else {
        resp.result = MC_IF_OD_ERR_NO_OBJECT;
    }
    encode(MC_IF_MSG_OD_WRITE_RESP, seq, &resp, (uint16_t)sizeof(resp), out);
}

static void handle_download_init(uint16_t seq, const uint8_t *pl, uint16_t plen, uint8_t *out)
{
    if (plen < sizeof(MC_IfOdDownloadInit_t)) { build_heartbeat(seq, out); return; }
    const MC_IfOdDownloadInit_t *r = (const MC_IfOdDownloadInit_t *)pl;

    if (r->index != 0x1F50u || r->subindex != 1u) {
        MC_IfOdDownloadResp_t resp = { 0u, (uint8_t)MC_IF_OD_ERR_NO_OBJECT, 0u, 0u };
        encode(MC_IF_MSG_OD_DOWNLOAD_RESP, seq, &resp, (uint16_t)sizeof(resp), out);
        return;
    }
    /* Response derived from state -> naturally idempotent; a retry never
     * re-erases and never races the first segment:
     *   session open -> OK (erase done, ready for segments)
     *   FAULT        -> FLASH_LOCKED
     *   IDLE         -> request the erase (main-loop pump) + heartbeat
     *   ERASING      -> heartbeat (client retries) */
    if (boot_seg_sdo_active()) {
        MC_IfOdDownloadResp_t resp = { 0u, (uint8_t)MC_IF_OD_OK, 0u, 0u };
        encode(MC_IF_MSG_OD_DOWNLOAD_RESP, seq, &resp, (uint16_t)sizeof(resp), out);
        return;
    }
    if (boot_flash_get_state() == BOOT_FLASH_FAULT) {
        MC_IfOdDownloadResp_t resp = { 0u, (uint8_t)MC_IF_OD_ERR_FLASH_LOCKED, 0u, 0u };
        encode(MC_IF_MSG_OD_DOWNLOAD_RESP, seq, &resp, (uint16_t)sizeof(resp), out);
        return;
    }
    if (boot_flash_get_state() == BOOT_FLASH_IDLE && !s_erase_req) {
        s_erase_total = r->total_length;
        s_init_seq    = seq;   /* answered by the pump once the erase completes */
        s_erase_req   = true;
    }
    build_heartbeat(seq, out);
}

static void handle_download_segment(uint16_t seq, const uint8_t *pl, uint16_t plen, uint8_t *out)
{
    if (plen < 3u) { build_heartbeat(seq, out); return; }
    const MC_IfOdDownloadSegment_t *seg = (const MC_IfOdDownloadSegment_t *)pl;
    MC_IfOdDownloadResp_t resp;
    (void)boot_seg_sdo_on_segment(seg, (uint8_t)plen, &resp);   /* programs flash (~0.6 ms) */
    encode(MC_IF_MSG_OD_DOWNLOAD_RESP, seq, &resp, (uint16_t)sizeof(resp), out);
}

/* ===== init + main-loop pump + per-frame handler ===== */

void boot_od_init(void)
{
    boot_flash_init();
    boot_seg_sdo_init();
    s_commit_pending = false;
    s_erase_req = false; s_erase_total = 0u;
    /* Do NOT compute a boot-time CRC over the whole region -- it reads erased
     * flash (ECC NMI). 0x1F56 stays 0 until a VERIFY (over written bytes only);
     * the PC tool treats 0 as "unknown". */
    s_verify_req = false; s_image_crc = 0u;
    s_init_answer = false;
}

void boot_od_pump(void)
{
    if (s_erase_req) {
        s_erase_req = false;
        /* Mark OTA-in-progress: set the boot flag STAY before erasing. If the
         * download then fails/partially completes (no COMMIT), the next boot
         * sees STAY and stays resident instead of jumping into a half-written
         * app (which hard-faults). Only the app clears the flag, after its
         * healthy window. */
        boot_flag_set_stay();
        /* Multi-second erase runs here in the main loop (never the ISR). On
         * success open the session; handle_download_init then acks OK off
         * boot_seg_sdo_active(). On failure the flash state stays FAULT. */
        if (boot_flash_begin(s_erase_total)) {
            boot_seg_sdo_start();
            s_init_result = (uint8_t)MC_IF_OD_OK;
        } else {
            s_init_result = (uint8_t)MC_IF_OD_ERR_FLASH_LOCKED;
        }
        s_init_answer = true;   /* ISR stages DOWNLOAD_RESP(s_init_seq) next frame */
    }
    if (s_verify_req) {
        s_verify_req = false;
        /* CRC32 over the whole image (~12 ms) — in the main loop, not the ISR,
         * so the SPI slave keeps answering. 0x1F56 returns this cached value. */
        s_image_crc = boot_flash_current_image_crc32();
    }
}

void boot_od_on_frame(const uint8_t *rx, uint8_t *tx)
{
    /* Erase finished -> answer the INIT that triggered it (the master waits on
     * the MISO for this seq and does not re-send INIT). Stage it once. Safe: the
     * session is already open, and rx is a snapshot, so no race with a segment. */
    if (s_init_answer) {
        MC_IfOdDownloadResp_t resp = { 0u, s_init_result, 0u, 0u };
        encode(MC_IF_MSG_OD_DOWNLOAD_RESP, s_init_seq, &resp, (uint16_t)sizeof(resp), tx);
        s_init_answer = false;
        return;
    }

    const MC_IfFrameHeader_t *h = (const MC_IfFrameHeader_t *)rx;
    uint16_t seq = h->sequence;

    /* Validate. Anything wrong -> still stage a valid HEARTBEAT (never leave the
     * MISO unframed). */
    bool ok = (h->sync == MC_IF_SYNC_WORD)
           && (h->version == MC_IF_PROTOCOL_VERSION)
           && (crc16(rx, MC_IF_HEADER_SIZE - 2u) == h->header_crc)
           && (h->payload_length <= MC_IF_MAX_PAYLOAD);
    if (ok) {
        const uint8_t *pl = rx + MC_IF_HEADER_SIZE;
        uint16_t pcrc; memcpy(&pcrc, pl + h->payload_length, 2u);
        ok = (crc16(pl, h->payload_length) == pcrc);
    }
    if (!ok) { build_heartbeat(0u, tx); return; }

    const uint8_t *pl = rx + MC_IF_HEADER_SIZE;
    switch (h->message_type) {
        case MC_IF_MSG_OD_READ_REQ:         handle_read            (seq, pl, h->payload_length, tx); break;
        case MC_IF_MSG_OD_WRITE_REQ:        handle_write           (seq, pl, h->payload_length, tx); break;
        case MC_IF_MSG_OD_DOWNLOAD_INIT:    handle_download_init   (seq, pl, h->payload_length, tx); break;
        case MC_IF_MSG_OD_DOWNLOAD_SEGMENT: handle_download_segment(seq, pl, h->payload_length, tx); break;
        case MC_IF_MSG_CYCLIC_CMD:          build_status   (seq, tx); break;  /* -> node_state BOOTLOADER */
        default:                            build_heartbeat(seq, tx); break;  /* HEARTBEAT / unknown */
    }
}
