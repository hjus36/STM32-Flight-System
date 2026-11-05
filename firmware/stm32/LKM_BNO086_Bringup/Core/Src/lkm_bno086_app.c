#include "lkm_bno086_app.h"
#include "lkm_bno086.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static lkm_bno086_t g_device;
static lkm_bno086_packet_t g_packet;
static lkm_bno086_product_id_t g_product;
static bool g_product_verified = false;

static void log_line(const char *text)
{
    if (text == NULL) return;
    size_t n = strlen(text);
    if (n > 0u && n <= 65535u) {
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)text, (uint16_t)n, 100u);
    }
}

void LKM_BNO086_App_Init(void)
{
    /* CubeMX labels must match BNO_CS, BNO_INTN, BNO_RST, BNO_PS0_WAKE. */
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

    lkm_bno086_bind(&g_device, &io);
    g_product_verified = false;
    log_line("\r\n[LKM BNO086] SHTP Product ID probe begin\r\n");

    const lkm_bno086_status_t status =
        lkm_bno086_probe(&g_device, &g_packet, &g_product);
    char buf[192];
    if (status == LKM_BNO086_OK) {
        g_product_verified = true;
        (void)snprintf(buf, sizeof(buf),
                       "[LKM BNO086] Product ID response OK: SW %u.%u.%u, part %lu, build %lu\r\n",
                       (unsigned)g_product.sw_major,
                       (unsigned)g_product.sw_minor,
                       (unsigned)g_product.sw_patch,
                       (unsigned long)g_product.sw_part_number,
                       (unsigned long)g_product.sw_build_number);
    } else {
        (void)snprintf(buf, sizeof(buf),
                       "[LKM BNO086] Probe failed: %s; boot mask=0x%02X; rx=%lu\r\n",
                       lkm_bno086_status_text(status),
                       (unsigned)g_device.startup_mask,
                       (unsigned long)g_device.received_packets);
    }
    log_line(buf);

    /* Never generate PWM or arm ESCs in the probe app. */
}

bool LKM_BNO086_App_ProductIdVerified(void)
{
    return g_product_verified;
}
