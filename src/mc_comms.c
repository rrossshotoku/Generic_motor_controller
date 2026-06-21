#include "mc_comms.h"
#include "mc_if_protocol.h"   /* shared contract (add ../Lightweight_CMC/Interface to the include path) */
#include "mc_if_od.h"
#include "mc_od.h"
#include "mc_debug.h"         /* g_mc_inject (command apply), g_mc_debug (status) */
#include <string.h>

/** @file mc_comms.c
 *  @brief Inter-MCU SPI protocol handler (slave). See ADR-016 and the shared INTERFACE_SPEC.md.
 */

volatile MC_CommsStats_t g_comms_stats;

/* Telemetry (TX-PDO) map: OD 0x2A00. Owned here (not in the mc_od table). */
static uint32_t s_map[MC_IF_TLM_MAX_ENTRIES];
static uint8_t  s_map_count;
static uint8_t  s_map_version;

/* Command watchdog. */
static uint32_t s_last_cmd_counter;
static bool     s_cmd_fresh;
static bool     s_link_active;     /* true once a cyclic command has ever been received */
static uint8_t  s_stall_ticks;

/* ===== CRC16/Modbus (poly 0xA001, init 0xFFFF) ===== */
static uint16_t crc16(const uint8_t *d, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    for (uint32_t i = 0u; i < n; i++)
    {
        c ^= d[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
        }
    }
    return c;
}

/* ===== Frame encode ===== */
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

/* ===== Telemetry frame (status header + 0x2A00 mapped blob) ===== */
static uint16_t build_telemetry(uint8_t *payload)
{
    MC_IfCyclicStatusHeader_t hdr;
    hdr.statusword     = (uint16_t)((g_mc_debug.pwm_enabled ? MC_IF_SW_ENABLED : 0u)
                                  | (g_mc_debug.overcurrent_trip ? MC_IF_SW_FAULT : 0u)
                                  | MC_IF_SW_READY);
    hdr.mode_display   = (int8_t)(g_mc_inject.velocity_enable ? MC_IF_MODE_PROFILE_VELOCITY
                                                              : MC_IF_MODE_DISABLED);
    hdr.node_state     = (uint8_t)(g_mc_debug.overcurrent_trip ? MC_IF_NODE_FAULT
                                  : (g_mc_debug.pwm_enabled ? MC_IF_NODE_RUNNING : MC_IF_NODE_READY));
    hdr.error_code     = 0u;
    hdr.map_version    = s_map_version;
    hdr.status_counter = s_last_cmd_counter;

    uint8_t *blob = payload + MC_IF_STATUS_HEADER_SIZE;
    uint8_t nbytes = 0u;
    for (uint8_t i = 0u; i < s_map_count; i++)
    {
        const uint16_t idx = MC_IF_TLM_MAP_INDEX_OF(s_map[i]);
        const uint8_t  sub = MC_IF_TLM_MAP_SUB_OF(s_map[i]);
        const uint8_t  nb  = (uint8_t)(MC_IF_TLM_MAP_BITS_OF(s_map[i]) / 8u);
        if ((nbytes + nb) > MC_IF_TLM_BLOB_MAX) { break; }
        MC_OdType_t t; uint32_t len;
        if ((MC_Od_ReadRaw(idx, sub, blob + nbytes, nb, &t, &len) == MC_OD_OK) && (len == nb))
        {
            nbytes = (uint8_t)(nbytes + nb);
        }
        else
        {
            memset(blob + nbytes, 0, nb);   /* unreadable -> zeros, keep layout */
            nbytes = (uint8_t)(nbytes + nb);
        }
    }
    hdr.map_byte_count = nbytes;
    memcpy(payload, &hdr, MC_IF_STATUS_HEADER_SIZE);
    return (uint16_t)(MC_IF_STATUS_HEADER_SIZE + nbytes);
}

/* ===== OD result mapping (engine -> wire) ===== */
static uint8_t od_result(MC_OdStatus_t s)
{
    switch (s)
    {
        case MC_OD_OK:            return MC_IF_OD_OK;
        case MC_OD_ERR_NOT_FOUND: return MC_IF_OD_ERR_NO_OBJECT;
        case MC_OD_ERR_ACCESS:    return MC_IF_OD_ERR_ACCESS;
        case MC_OD_ERR_TYPE:      return MC_IF_OD_ERR_TYPE;
        case MC_OD_ERR_RANGE:     return MC_IF_OD_ERR_RANGE;
        case MC_OD_ERR_SIZE:      return MC_IF_OD_ERR_SIZE;
        default:                  return MC_IF_OD_ERR_CALLBACK;
    }
}

/* ===== Telemetry-map (0x2A00) write handler ===== */
static uint8_t map_write(uint8_t sub, const uint8_t *data, uint8_t len)
{
    if (sub == 0u)                       /* count -> (re)activate the map */
    {
        if (len < 1u) { return MC_IF_OD_ERR_SIZE; }
        const uint8_t count = data[0];
        if (count > MC_IF_TLM_MAX_ENTRIES) { return MC_IF_OD_ERR_RANGE; }
        /* validate total bytes + each entry exists, is PDO, size matches */
        uint16_t total = 0u;
        for (uint8_t i = 0u; i < count; i++)
        {
            const uint16_t idx = MC_IF_TLM_MAP_INDEX_OF(s_map[i]);
            const uint8_t  esub = MC_IF_TLM_MAP_SUB_OF(s_map[i]);
            const uint8_t  nb   = (uint8_t)(MC_IF_TLM_MAP_BITS_OF(s_map[i]) / 8u);
            const MC_OdEntry_t *e = MC_Od_Find(idx, esub);
            if ((e == 0) || (!e->pdo_mappable)) { return MC_IF_OD_ERR_NO_OBJECT; }
            total = (uint16_t)(total + nb);
        }
        if (total > MC_IF_TLM_BLOB_MAX) { return MC_IF_OD_ERR_RANGE; }
        s_map_count = count;
        s_map_version++;
        g_comms_stats.map_count = count;
        g_comms_stats.map_version = s_map_version;
        return MC_IF_OD_OK;
    }
    if ((sub >= 1u) && (sub <= MC_IF_TLM_MAX_ENTRIES)) /* map slot */
    {
        if (len < 4u) { return MC_IF_OD_ERR_SIZE; }
        uint32_t v; memcpy(&v, data, 4u);
        s_map[sub - 1u] = v;
        return MC_IF_OD_OK;
    }
    return MC_IF_OD_ERR_NO_SUB;
}

/* ===== Cyclic command apply (minimal: enable + velocity/jog). Full mode manager: later. ===== */
static void apply_cyclic(const MC_IfCyclicCommand_t *c)
{
    const bool enable = (c->controlword & MC_IF_CW_ENABLE) != 0u;
    g_mc_inject.inject_enable = enable;
    g_mc_inject.foc_enable    = enable;

    if ((c->mode_of_operation == MC_IF_MODE_JOYSTICK_VELOCITY) ||
        (c->mode_of_operation == MC_IF_MODE_PROFILE_VELOCITY))
    {
        g_mc_inject.velocity_enable    = true;
        g_mc_inject.velocity_cmd_rad_s = (float)c->target_velocity * MC_IF_VEL_SCALE;
    }
    else
    {
        g_mc_inject.velocity_enable = false;
    }

    s_last_cmd_counter = c->command_counter;
    s_cmd_fresh = true;
    s_link_active = true;
}

void MC_Comms_Init(void)
{
    memset(s_map, 0, sizeof(s_map));
    s_map_count = 0u;
    s_map_version = 0u;
    s_last_cmd_counter = 0u;
    s_cmd_fresh = false;
    s_stall_ticks = 0u;
    memset((void *)&g_comms_stats, 0, sizeof(g_comms_stats));
}

void MC_Comms_BuildIdle(uint8_t *tx)
{
    uint8_t pl[MC_IF_MAX_PAYLOAD];
    const uint16_t n = build_telemetry(pl);
    encode(MC_IF_MSG_CYCLIC_STATUS, 0u, pl, n, tx);
}

void MC_Comms_HandleTransaction(const uint8_t *rx, uint8_t *tx_next)
{
    const MC_IfFrameHeader_t *h = (const MC_IfFrameHeader_t *)rx;
    uint8_t  resp_type = 0u;
    uint8_t  resp_pl[MC_IF_MAX_PAYLOAD];
    uint16_t resp_len = 0u;
    uint16_t seq = h->sequence;

    /* Validate frame. */
    bool ok = true;
    if (h->sync != MC_IF_SYNC_WORD)                        { ok = false; }
    else if (h->version != MC_IF_PROTOCOL_VERSION)         { ok = false; }
    else if (crc16(rx, MC_IF_HEADER_SIZE - 2u) != h->header_crc) { ok = false; g_comms_stats.frames_crc_err++; }
    else if (h->payload_length > MC_IF_MAX_PAYLOAD)        { ok = false; }
    else
    {
        const uint8_t *pl = rx + MC_IF_HEADER_SIZE;
        uint16_t pcrc; memcpy(&pcrc, pl + h->payload_length, 2u);
        if (crc16(pl, h->payload_length) != pcrc)          { ok = false; g_comms_stats.frames_crc_err++; }
    }

    if (!ok)
    {
        g_comms_stats.frames_bad++;
        /* Reply with the latest telemetry (don't stall the cyclic stream on a bad frame). */
        resp_len = build_telemetry(resp_pl);
        encode(MC_IF_MSG_CYCLIC_STATUS, seq, resp_pl, resp_len, tx_next);
        return;
    }

    const uint8_t *pl = rx + MC_IF_HEADER_SIZE;
    switch (h->message_type)
    {
        case MC_IF_MSG_CYCLIC_CMD:
            apply_cyclic((const MC_IfCyclicCommand_t *)pl);
            g_comms_stats.cyclic_cmds++;
            break;

        case MC_IF_MSG_OD_READ_REQ:
        {
            const MC_IfOdReadReq_t *r = (const MC_IfOdReadReq_t *)pl;
            MC_IfOdReadResp_t resp; memset(&resp, 0, sizeof(resp));
            resp.index = r->index; resp.subindex = r->subindex;
            MC_OdType_t t; uint32_t len = 0u;
            const MC_OdStatus_t s = MC_Od_ReadRaw(r->index, r->subindex, resp.data, sizeof(resp.data), &t, &len);
            resp.result = od_result(s);
            resp.type = (uint8_t)t; resp.data_length = (uint8_t)len;
            resp_type = MC_IF_MSG_OD_READ_RESP;
            memcpy(resp_pl, &resp, sizeof(resp)); resp_len = (uint16_t)sizeof(resp);
            g_comms_stats.od_reads++;
            break;
        }

        case MC_IF_MSG_OD_WRITE_REQ:
        {
            const MC_IfOdWriteReq_t *w = (const MC_IfOdWriteReq_t *)pl;
            uint8_t result;
            if (w->index == MC_IF_TLM_MAP_INDEX)
            {
                result = map_write(w->subindex, w->data, w->data_length);
            }
            else
            {
                const MC_OdStatus_t s = MC_Od_Write(w->index, w->subindex, w->data,
                                                    w->data_length, (MC_OdType_t)w->type);
                result = od_result(s);
            }
            MC_IfOdWriteResp_t resp;
            resp.index = w->index; resp.subindex = w->subindex; resp.result = result;
            resp_type = MC_IF_MSG_OD_WRITE_RESP;
            memcpy(resp_pl, &resp, sizeof(resp)); resp_len = (uint16_t)sizeof(resp);
            g_comms_stats.od_writes++;
            break;
        }

        case MC_IF_MSG_HEARTBEAT:
        default:
            break;   /* nothing to apply; reply with telemetry below */
    }

    /* Build the outgoing frame: an OD response if one was produced, else telemetry. */
    if (resp_type != 0u)
    {
        encode(resp_type, seq, resp_pl, resp_len, tx_next);
    }
    else
    {
        resp_len = build_telemetry(resp_pl);
        encode(MC_IF_MSG_CYCLIC_STATUS, seq, resp_pl, resp_len, tx_next);
    }
}

bool MC_Comms_CommandTimedOut(void)
{
    if (!s_link_active)   /* no master yet (watch-window bring-up) -> never trip */
    {
        g_comms_stats.cmd_timeout = false;
        return false;
    }
    if (s_cmd_fresh)
    {
        s_cmd_fresh = false;
        s_stall_ticks = 0u;
    }
    else if (s_stall_ticks < 255u)
    {
        s_stall_ticks++;
    }
    /* Slow loop ~100 Hz (10 ms/tick); timeout after MC_IF_COMMAND_TIMEOUT_MS. */
    const uint8_t limit = (uint8_t)(MC_IF_COMMAND_TIMEOUT_MS / 10u);
    const bool timed_out = (s_stall_ticks >= (limit == 0u ? 1u : limit));
    g_comms_stats.cmd_timeout = timed_out;
    return timed_out;
}
