#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/rmt_rx.h"
#include "ws2835.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0

static const char *TAG = "ws2812";
#define STRIP_CHECK(a, str, gotoTag, retValue, ...)                               \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = retValue;                                                       \
            goto gotoTag;                                                         \
        }                                                                         \
    } while (0)

#define WS2812_T0H_NS (350)
#define WS2812_T0L_NS (800)
#define WS2812_T1H_NS (800)
#define WS2812_T1L_NS (600)
#define WS2812_RESET_US (300)

static jee_uint32_t ws2812T0hTicks = 0;
static jee_uint32_t ws2812T1hTicks = 0;
static jee_uint32_t ws2812T0lTicks = 0;
static jee_uint32_t ws2812T1lTicks = 0;

typedef struct
{
    ledStrip_t parent;
    rmt_channel_t rmt_channel;
    jee_uint32_t strip_len;
    jee_uint8_t buffer[0];
} ws2812_t;

static ledStrip_t *strip;

#define WS2812_DAT_IO (3)
#define LED_STRIP_NUM 25

/**
 * @brief Conver RGB data to RMT format.
 *
 * @note For WS2812, R,G,B each contains 256 different choices (i.e. uint8_t)
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] srcSize: size of source data
 * @param[in] wantedNum: number of RMT items that want to get
 * @param[out] translatedSize: number of source data that got converted
 * @param[out] itemNum: number of RMT items which are converted from source data
 */
static void IRAM_ATTR ws2812RmtAdapter(const void *src, rmt_item32_t *dest, jee_size_t srcSize,
                                       jee_size_t wantedNum, jee_size_t *translatedSize, jee_size_t *itemNum)
{
    if (src == NULL || dest == NULL)
    {
        *translatedSize = 0;
        *itemNum = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ws2812T0hTicks, 1, ws2812T0lTicks, 0}}}; // Logical 0
    const rmt_item32_t bit1 = {{{ws2812T1hTicks, 1, ws2812T1lTicks, 0}}}; // Logical 1
    jee_size_t size = 0;
    jee_size_t num = 0;
    jee_uint8_t *psrc = (jee_uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < srcSize && num < wantedNum)
    {
        for (int i = 0; i < 8; i++)
        {
            // MSB first
            if (*psrc & (1 << (7 - i)))
            {
                pdest->val = bit1.val;
            }
            else
            {
                pdest->val = bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translatedSize = size;
    *itemNum = num;
}

static esp_err_t ws2812SetPixelFunc(ledStrip_t *strip, jee_uint32_t index, jee_uint32_t red, jee_uint32_t green, jee_uint32_t blue)
{
    esp_err_t ret = ESP_OK;
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    STRIP_CHECK(index < ws2812->strip_len, "index out of the maximum number of leds", err, ESP_ERR_INVALID_ARG);
    jee_uint32_t start = index * 3;
    // In thr order of GRB
    ws2812->buffer[start + 0] = green & 0xFF;
    ws2812->buffer[start + 1] = red & 0xFF;
    ws2812->buffer[start + 2] = blue & 0xFF;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ws2812RefreshFunc(ledStrip_t *strip, jee_uint32_t timeoutMs)
{
    esp_err_t ret = ESP_OK;
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    STRIP_CHECK(rmt_write_sample(ws2812->rmt_channel, ws2812->buffer, ws2812->strip_len * 3, true) == ESP_OK,
                "transmit RMT samples failed", err, ESP_FAIL);
    return rmt_wait_tx_done(ws2812->rmt_channel, pdMS_TO_TICKS(timeoutMs));
err:
    return ret;
}

static esp_err_t ws2812ClearFunc(ledStrip_t *strip, jee_uint32_t timeoutMs)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    // Write zero to turn off all leds
    memset(ws2812->buffer, 0, ws2812->strip_len * 3);
    return ws2812RefreshFunc(strip, timeoutMs);
}

static esp_err_t ws2812DelFunc(ledStrip_t *strip)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    free(ws2812);
    return ESP_OK;
}

ledStrip_t *ledStripNewRmtWs2812(const LedStripConfig_t *config)
{
    ledStrip_t *ret = NULL;
    STRIP_CHECK(config, "configuration can't be null", err, NULL);

    // 24 bits per led
    jee_uint32_t ws2812Size = sizeof(ws2812_t) + config->max_leds * 3;
    ws2812_t *ws2812 = calloc(1, ws2812Size);
    STRIP_CHECK(ws2812, "request memory for ws2812 failed", err, NULL);

    uint32_t counterClkHz = 0;
    STRIP_CHECK(rmt_get_counter_clock((rmt_channel_t)config->dev, &counterClkHz) == ESP_OK,
                "get rmt counter clock failed", err, NULL);
    // ns -> ticks
    float ratio = (float)counterClkHz / 1e9;
    ws2812T0hTicks = (uint32_t)(ratio * WS2812_T0H_NS);
    ws2812T0lTicks = (uint32_t)(ratio * WS2812_T0L_NS);
    ws2812T1hTicks = (uint32_t)(ratio * WS2812_T1H_NS);
    ws2812T1lTicks = (uint32_t)(ratio * WS2812_T1L_NS);

    // set ws2812 to rmt adapter
    rmt_translator_init((rmt_channel_t)config->dev, ws2812RmtAdapter);

    ws2812->rmt_channel = (rmt_channel_t)config->dev;
    ws2812->strip_len = config->max_leds;

    ws2812->parent.set_pixel = ws2812SetPixelFunc;
    ws2812->parent.refresh = ws2812RefreshFunc;
    ws2812->parent.clear = ws2812ClearFunc;
    ws2812->parent.del = ws2812DelFunc;

    return &ws2812->parent;
err:
    return ret;
}

ledStrip_t *ledStripInit(jee_uint8_t channel, jee_uint8_t gpio, jee_uint16_t led_num)
{
    static ledStrip_t *pStrip;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio, channel);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    LedStripConfig_t strip_config = LED_STRIP_DEFAULT_CONFIG(led_num, (led_strip_dev_t)config.channel);

    pStrip = ledStripNewRmtWs2812(&strip_config);

    if (!pStrip)
    {
        ESP_LOGE(TAG, "install WS2812 driver failed");
        return NULL;
    }

    // Clear LED strip (turn off all LEDs)
    // ws2812Clear(100);

    return pStrip;
}

esp_err_t ledStripDenit(ledStrip_t *strip)
{
    ws2812_t *ws2812 = __containerof(strip, ws2812_t, parent);
    ESP_ERROR_CHECK(rmt_driver_uninstall(ws2812->rmt_channel));
    return strip->del(strip);
}

jee_int32_t ws2812SetPixel(jee_uint32_t index, jee_uint32_t red, jee_uint32_t green, jee_uint32_t blue)
{
    return ws2812SetPixelFunc(strip, index, red, green, blue);
}

jee_int32_t ws2812Refresh(jee_uint32_t timeout_ms)
{
    return ws2812RefreshFunc(strip, timeout_ms);
}

jee_int32_t ws2812Clear(jee_uint32_t timeout_ms)
{
    return ws2812ClearFunc(strip, timeout_ms);
}

jee_int32_t ws2812Del(void)
{
    ws2812DelFunc(strip);
    return ESP_OK;
}

void ws2812Init(void)
{

    strip = ledStripInit(RMT_CHANNEL_0, WS2812_DAT_IO, LED_STRIP_NUM);
    if (!strip)
    {
        ESP_LOGE(TAG, "Install WS2812 driver failed");
    }

}

