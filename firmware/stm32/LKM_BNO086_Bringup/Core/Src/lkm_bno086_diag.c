#include "lkm_bno086_diag.h"
#include "lkm_bno086.h"
#include "lkm_bno086_probe_duplex.h"
#include "lkm_bno086_stream.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* One BNO086 owner: the original Product ID probe and Stream Link share the
 * SAME device, scratch packet and control-channel TX sequence. */
static lkm_bno086_t g_device;
static lkm_bno086_packet_t g_packet;
static lkm_bno086_product_id_t g_product;
static lkm_bno086_stream_t g_stream;
static lkm_bno086_link_t g_link;
static bool g_product_verified = false;
static lkm_bno086_diag_state_t g_state = LKM_BNO086_DIAG_UNINITIALIZED;
static lkm_link_result_t g_last_link = LKM_LINK_NOT_READY;
static uint32_t g_retry_at_ms = 0u;
static uint32_t g_confirm_since_ms = 0u;
static uint32_t g_service_calls = 0u;
static uint8_t g_handshake_failures = 0u;

#define LKM_DIAG_MAX_HANDSHAKE_FAILURES 3u
#define LKM_DIAG_HANDSHAKE_BACKOFF_MS   50u
#define LKM_DIAG_FEATURE_CONFIRM_MS   750u

static void log_line(const char *text)
{
    if (text == NULL) return;
    const size_t n = strlen(text);
    if (n > 0u && n <= 65535u) {
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)text, (uint16_t)n, 100u);
    }
}

static void fault(const char *phase, lkm_link_result_t cause)
{
    g_state = LKM_BNO086_DIAG_FAULT;
    g_last_link = cause;
    /* Fail closed even when the link itself returned a non-latched error. */
    g_stream.sample_valid = false;
    g_stream.feature_confirmed = false;
    char line[128];
    (void)snprintf(line, sizeof(line),
                   "[LKM BNO086] Diagnostic fault in %s: %s\r\n",
                   phase, lkm_bno086_link_result_text(cause));
    log_line(line);
}

static bool retry_is_due(uint32_t now_ms)
{
    /* Monotonic relative time; valid for < 2^31 ms intervals. */
    return (uint32_t)(now_ms - g_retry_at_ms) < 0x80000000u;
}

static void handle_tx_result(lkm_link_result_t result,
                             lkm_bno086_diag_state_t next,
                             const char *phase)
{
    g_last_link = result;
    if (result == LKM_LINK_OK) {
        g_handshake_failures = 0u;
        g_state = next;
        if (next == LKM_BNO086_DIAG_WAIT_FEATURE_CONFIRM)
            g_confirm_since_ms = HAL_GetTick();
        return;
    }
    if (result == LKM_LINK_HANDSHAKE_TIMEOUT) {
        /* No CS was asserted, so the Link guarantees the sequence can be
         * retried. Unlike an ambiguous SPI failure, this retry is bounded. */
        ++g_handshake_failures;
        if (g_handshake_failures >= LKM_DIAG_MAX_HANDSHAKE_FAILURES) {
            fault(phase, result);
        } else {
            g_retry_at_ms = HAL_GetTick() + LKM_DIAG_HANDSHAKE_BACKOFF_MS;
        }
        return;
    }
    fault(phase, result);
}

void LKM_BNO086_Diag_Init(void)
{
    const lkm_bno086_io_t io = {
        .spi = &hspi2,
        .cs_port = BNO_CS_GPIO_Port,
        .cs_pin = BNO_CS_Pin,
        .irq_port = BNO_INTN_GPIO_Port,
        .irq_pin = BNO_INTN_Pin,
        .reset_port = BNO_RST_GPIO_Port,
        .reset_pin = BNO_RST_Pin,
        .wake_port = BNO_PS0_WAKE_GPIO_Port,
        .wake_pin = BNO_PS0_WAKE_Pin
    };
    const lkm_stream_config_t config = {
        .report_id = LKM_ATTITUDE_ROTATION_VECTOR,
        .report_interval_us = 10000u, /* 100 Hz request, NOT measured output */
        .stale_after_ms = 50u,        /* diagnostic freshness gate */
        .minimum_accuracy = 1u
    };

    g_product_verified = false;
    g_state = LKM_BNO086_DIAG_UNINITIALIZED;
    g_last_link = LKM_LINK_NOT_READY;
    g_retry_at_ms = 0u;
    g_confirm_since_ms = 0u;
    g_service_calls = 0u;
    g_handshake_failures = 0u;
    memset(&g_packet, 0, sizeof(g_packet));
    memset(&g_stream, 0, sizeof(g_stream));
    memset(&g_link, 0, sizeof(g_link));
    lkm_bno086_bind(&g_device, &io);
    log_line("\r\n[LKM BNO086] SHTP Product ID probe begin\r\n");

    const lkm_bno086_status_t status =
        lkm_bno086_probe_duplex(&g_device, &g_packet, &g_product);
    char buf[192];
    if (status != LKM_BNO086_OK) {
        g_state = LKM_BNO086_DIAG_PROBE_FAILED;
        (void)snprintf(buf, sizeof(buf),
                       "[LKM BNO086] Probe failed: %s; boot mask=0x%02X; rx=%lu\r\n",
                       lkm_bno086_status_text(status),
                       (unsigned)g_device.startup_mask,
                       (unsigned long)g_device.received_packets);
        log_line(buf);
        return;
    }

    g_product_verified = true;
    (void)snprintf(buf, sizeof(buf),
                   "[LKM BNO086] Product ID response OK: SW %u.%u.%u, part %lu, build %lu\r\n",
                   (unsigned)g_product.sw_major,
                   (unsigned)g_product.sw_minor,
                   (unsigned)g_product.sw_patch,
                   (unsigned long)g_product.sw_part_number,
                   (unsigned long)g_product.sw_build_number);
    log_line(buf);

    const lkm_link_result_t init_status = lkm_bno086_link_init(
        &g_link, &g_device, &g_stream, &g_packet, &config,
        g_product_verified);
    if (init_status != LKM_LINK_OK) {
        fault("link-init", init_status);
        return;
    }
    g_state = LKM_BNO086_DIAG_NEED_SET_FEATURE;
    g_retry_at_ms = HAL_GetTick();
    g_last_link = LKM_LINK_OK;
    /* Product ID consumed TX seq0. The new Link now owns channel2 seq1+.
     * No Set/Get commands are sent from this blocking init function. */
}

void LKM_BNO086_Diag_Service(void)
{
    ++g_service_calls;
    const uint32_t now = HAL_GetTick();
    switch (g_state) {
    case LKM_BNO086_DIAG_NEED_SET_FEATURE:
        if (!retry_is_due(now)) return;
        handle_tx_result(lkm_bno086_link_send_set(&g_link),
                         LKM_BNO086_DIAG_NEED_GET_FEATURE, "set-feature");
        return;

    case LKM_BNO086_DIAG_NEED_GET_FEATURE:
        if (!retry_is_due(now)) return;
        handle_tx_result(lkm_bno086_link_send_get(&g_link),
                         LKM_BNO086_DIAG_WAIT_FEATURE_CONFIRM, "get-feature");
        if (g_state == LKM_BNO086_DIAG_WAIT_FEATURE_CONFIRM &&
            g_stream.feature_confirmed) {
            g_state = LKM_BNO086_DIAG_STREAMING;
            log_line("[LKM BNO086] Feature confirmed; diagnostic stream enabled\r\n");
        }
        return;

    case LKM_BNO086_DIAG_WAIT_FEATURE_CONFIRM:
        if ((uint32_t)(now - g_confirm_since_ms) > LKM_DIAG_FEATURE_CONFIRM_MS) {
            fault("feature-confirm-timeout", LKM_LINK_STREAM_FAILURE);
            return;
        }
        g_last_link = lkm_bno086_link_poll(&g_link);
        if (g_last_link != LKM_LINK_OK && g_last_link != LKM_LINK_NO_PACKET) {
            fault("feature-confirm-rx", g_last_link);
            return;
        }
        if (g_stream.feature_confirmed) {
            g_state = LKM_BNO086_DIAG_STREAMING;
            log_line("[LKM BNO086] Feature confirmed; diagnostic stream enabled\r\n");
        }
        return;

    case LKM_BNO086_DIAG_STREAMING:
        g_last_link = lkm_bno086_link_poll(&g_link);
        if (g_last_link != LKM_LINK_OK && g_last_link != LKM_LINK_NO_PACKET)
            fault("stream-rx", g_last_link);
        return;

    case LKM_BNO086_DIAG_UNINITIALIZED:
    case LKM_BNO086_DIAG_PROBE_FAILED:
    case LKM_BNO086_DIAG_FAULT:
    default:
        /* No SPI/GPIO access after a failed probe or latched fault. */
        return;
    }
}

bool LKM_BNO086_Diag_ProductIdVerified(void)
{
    return g_product_verified;
}

bool LKM_BNO086_Diag_TryGetAttitude(lkm_bno086_attitude_t *out)
{
    if (out == NULL || g_state != LKM_BNO086_DIAG_STREAMING)
        return false;
    return lkm_bno086_link_get_attitude(&g_link, HAL_GetTick(), out) ==
           LKM_LINK_OK;
}


bool LKM_BNO086_Diag_TryGetTimedAttitude(lkm_bno086_timed_attitude_t *out)
{
    if (out == NULL) return false;
    /* A caller must check true. On failure no previous pose leaks out. */
    memset(out, 0, sizeof(*out));
    if (g_state != LKM_BNO086_DIAG_STREAMING ||
        !g_link.ready || g_link.fault_latched ||
        !g_stream.sample_valid || g_stream.valid_samples == 0u)
        return false;

    const uint32_t now_ms = HAL_GetTick();
    lkm_bno086_attitude_t pose = {0};
    if (lkm_bno086_link_get_attitude(&g_link, now_ms, &pose) != LKM_LINK_OK)
        return false;

    /* The single-owner Stream records this host timestamp at the moment it
     * accepts the SHTP sensor packet. Wrap-safe for bounded freshness TTL. */
    const uint32_t age_ms = (uint32_t)(now_ms - g_stream.last_receive_ms);
    if (age_ms > g_stream.config.stale_after_ms) return false;
    out->attitude = pose;
    out->host_received_ms = g_stream.last_receive_ms;
    out->host_age_ms = age_ms;
    out->accepted_packet_counter = g_stream.valid_samples;
    return true;
}

lkm_bno086_diag_state_t LKM_BNO086_Diag_GetState(void)
{
    return g_state;
}

lkm_link_result_t LKM_BNO086_Diag_LastLinkResult(void)
{
    return g_last_link;
}

uint32_t LKM_BNO086_Diag_ServiceCalls(void)
{
    return g_service_calls;
}
