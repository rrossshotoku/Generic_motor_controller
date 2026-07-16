/*
 * boot_seg_sdo — see boot_seg_sdo.h. Lifted from the CMC bootloader
 * unchanged (transport-agnostic state machine).
 */

#include "boot_seg_sdo.h"
#include "boot_flash.h"

#include "mc_if_od.h"

#include <string.h>

/* Segment flag bits (INTERFACE_SPEC.md §7c). */
#define SEG_FLAG_TOGGLE  0x01u
#define SEG_FLAG_LAST    0x02u

static bool     s_in_session;
static uint8_t  s_expected_toggle;
static uint32_t s_bytes_accepted;

static size_t build_resp(MC_IfOdDownloadResp_t *out,
                         uint8_t toggle_ack, MC_IfOdResult_t result)
{
    out->toggle_ack     = toggle_ack;
    out->result         = (uint8_t)result;
    out->reserved       = 0;
    out->bytes_accepted = s_bytes_accepted;
    return sizeof(*out);
}

void boot_seg_sdo_init(void)
{
    s_in_session      = false;
    s_expected_toggle = 0u;
    s_bytes_accepted  = 0u;
}

/* Open a session. Called by boot_od's main-loop pump AFTER boot_flash_begin
 * has erased the app region — never from the SPI ISR, so there is no race
 * against boot_seg_sdo_on_segment (which runs in the ISR). */
void boot_seg_sdo_start(void)
{
    s_in_session      = true;
    s_expected_toggle = 0u;
    s_bytes_accepted  = 0u;
}

bool boot_seg_sdo_active(void) { return s_in_session; }

size_t boot_seg_sdo_on_segment(const MC_IfOdDownloadSegment_t *seg,
                               uint8_t body_len,
                               MC_IfOdDownloadResp_t *out_resp)
{
    if (!s_in_session) {
        return build_resp(out_resp, 0u, MC_IF_OD_ERR_NOT_READY);
    }

    uint8_t got_toggle = (seg->flags & SEG_FLAG_TOGGLE) ? 1u : 0u;
    if (got_toggle != s_expected_toggle) {
        /* Sender missed our last ack — reply with what they should use,
         * don't touch the write cursor. Sender resends. */
        return build_resp(out_resp, s_expected_toggle, MC_IF_OD_OK);
    }

    /* Fixed portion of MC_IfOdDownloadSegment_t before data[] = 3 bytes. */
    const uint8_t seg_hdr_bytes = 3u;
    if (body_len < seg_hdr_bytes || (uint32_t)(seg_hdr_bytes + seg->seg_length) > body_len) {
        s_in_session = false;
        boot_flash_abort();
        return build_resp(out_resp, s_expected_toggle, MC_IF_OD_ERR_SIZE);
    }

    if (seg->seg_length > 0u) {
        if (!boot_flash_write(seg->data, seg->seg_length)) {
            s_in_session = false;
            return build_resp(out_resp, s_expected_toggle, MC_IF_OD_ERR_FLASH_LOCKED);
        }
        s_bytes_accepted += seg->seg_length;
    }

    s_expected_toggle ^= 1u;   /* flip for the next expected segment */

    if (seg->flags & SEG_FLAG_LAST) {
        /* Last segment — session complete; caller issues VERIFY next. */
        s_in_session = false;
    }
    return build_resp(out_resp, got_toggle, MC_IF_OD_OK);
}

void boot_seg_sdo_abort(void)
{
    s_in_session      = false;
    s_expected_toggle = 0u;
    s_bytes_accepted  = 0u;
    boot_flash_abort();
}
