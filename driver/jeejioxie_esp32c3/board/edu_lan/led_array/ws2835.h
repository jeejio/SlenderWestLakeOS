#ifndef __WS2835_H__
#define __WS2835_H__

#include "stdint.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"

#include "jeedef.h"

#if 1
#define CODE_1 (0xFC)
#define CODE_0 (0xC0)

/**
 * @brief LED Strip Type
 *
 */
typedef struct ledStripS ledStrip_t;

/**
 * @brief LED Strip Device Type
 *
 */
typedef void *led_strip_dev_t;

/**
 * @brief Declare of LED Strip Type
 *
 */
struct ledStripS
{
    /**
     * @brief Set RGB for a specific pixel
     *
     * @param strip: LED strip
     * @param index: index of pixel to set
     * @param red: red part of color
     * @param green: green part of color
     * @param blue: blue part of color
     *
     * @return
     *      - ESP_OK: Set RGB for a specific pixel successfully
     *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
     *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
     */
    esp_err_t (*set_pixel)(ledStrip_t *strip, jee_uint32_t index, jee_uint32_t red, jee_uint32_t green, jee_uint32_t blue);

    /**
     * @brief Refresh memory colors to LEDs
     *
     * @param strip: LED strip
     * @param timeout_ms: timeout value for refreshing task
     *
     * @return
     *      - ESP_OK: Refresh successfully
     *      - ESP_ERR_TIMEOUT: Refresh failed because of timeout
     *      - ESP_FAIL: Refresh failed because some other error occurred
     *
     * @note:
     *      After updating the LED colors in the memory, a following invocation of this API is needed to flush colors to strip.
     */
    esp_err_t (*refresh)(ledStrip_t *strip, jee_uint32_t timeout_ms);

    /**
     * @brief Clear LED strip (turn off all LEDs)
     *
     * @param strip: LED strip
     * @param timeout_ms: timeout value for clearing task
     *
     * @return
     *      - ESP_OK: Clear LEDs successfully
     *      - ESP_ERR_TIMEOUT: Clear LEDs failed because of timeout
     *      - ESP_FAIL: Clear LEDs failed because some other error occurred
     */
    esp_err_t (*clear)(ledStrip_t *strip, jee_uint32_t timeout_ms);

    /**
     * @brief Free LED strip resources
     *
     * @param strip: LED strip
     *
     * @return
     *      - ESP_OK: Free resources successfully
     *      - ESP_FAIL: Free resources failed because error occurred
     */
    esp_err_t (*del)(ledStrip_t *strip);
};

/**
 * @brief LED Strip Configuration Type
 *
 */
typedef struct
{
    jee_uint32_t max_leds; /*!< Maximum LEDs in a single strip */
    led_strip_dev_t dev;   /*!< LED strip device (e.g. RMT channel, PWM channel, etc) */
} LedStripConfig_t;

/**
 * @brief Default configuration for LED strip
 *
 */
#define LED_STRIP_DEFAULT_CONFIG(number, dev_hdl) \
    {                                             \
        .max_leds = number,                       \
        .dev = dev_hdl,                           \
    }

/**
 * @brief Install a new ws2812 driver (based on RMT peripheral)
 *
 * @param config: LED strip configuration
 * @return
 *      LED strip instance or NULL
 */
ledStrip_t *ledStripNewRmtWs2812(const LedStripConfig_t *config);

/**
 * @brief Init the RMT peripheral and LED strip configuration.
 *
 * @param[in] channel: RMT peripheral channel number.
 * @param[in] gpio: GPIO number for the RMT data output.
 * @param[in] led_num: number of addressable LEDs.
 * @return
 *      LED strip instance or NULL
 */
ledStrip_t *led_strip_init(jee_uint8_t channel, jee_uint8_t gpio, jee_uint16_t led_num);

/**
 * @brief Denit the RMT peripheral.
 *
 * @param[in] strip: LED strip
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t ledStripDenit(ledStrip_t *strip);
typedef struct
{
    float hue;
    float saturation;
    float value;
} hal_hsv_t;

jee_int32_t ws2812SetPixel(jee_uint32_t index, jee_uint32_t red, jee_uint32_t green, jee_uint32_t blue);

jee_int32_t ws2812Refresh(jee_uint32_t timeout_ms);

jee_int32_t ws2812Clear(jee_uint32_t timeout_ms);

jee_int32_t ws2812Del(void);

void ws2812Init(void);
#endif /* __HAL_LED_ARRAY_H__ */

#endif
