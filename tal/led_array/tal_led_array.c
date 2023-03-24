#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_led_array.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#define LED_STRIP_NUM 25

#define LED_ARRAY_DEV "led_ws2835"
jee_device_t ledArrayDev = NULL;

static SemaphoreHandle_t ledRefreshSig = NULL;
static TimerHandle_t ledFlashTimer = NULL;

static int flashNumber = -1;
static int flashMode = ledArryFlashModeNull;

led_t led;
void vLedArrayRefreshTask(void *arg)
{
    jee_uint16_t i = 0;

    while (1)
    {
        xSemaphoreTake(ledRefreshSig, portMAX_DELAY);
        printf("r  g  b    %d   %d   %d  \r\n", led.color.red, led.color.green, led.color.blue);
        for (i = 0; i < LED_STRIP_NUM; i++)
        {
            jee_uint32_t setData[4] = {i, led.color.red, led.color.green, led.color.blue};
            jee_device_control(ledArrayDev, WS2835_PIXEL_SET_CMD, setData);
        }
        jee_uint32_t setData = 10;
        jee_device_control(ledArrayDev, WS2835_REFRESH_SET_CMD, &setData);
    }
}

void vLedArrayFlashTimerCb(TimerHandle_t timer)
{
    printf("timer cb count  %d\r\n", led.flashCount);

    // if ((led.flashCount == 0) || (led.flash_onoff == 0))
    if (led.flashCount == 0)
    {
        xTimerStop(ledFlashTimer, 0);
        return;
    }

    if (led.flashOnoff)
    {
        led.color.red = 0;
        led.color.green = 0;
        led.color.blue = 0;
    }
    else
    {
        led.color.red = led.colorTarget.red;
        led.color.green = led.colorTarget.green;
        led.color.blue = led.colorTarget.blue;
        led.flashCount--;
    }
    led.flashOnoff = !led.flashOnoff;
    xSemaphoreGive(ledRefreshSig);
}

jee_int32_t lTalLedArrayInit(void)
{
    // 查找设备
    ledArrayDev = jee_device_find(LED_ARRAY_DEV);
    if (ledArrayDev == JEE_NULL)
    {
        printf("can not find %s Model\n", LED_ARRAY_DEV);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(ledArrayDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", LED_ARRAY_DEV);
        return JEE_ERROR;
    }

    ledRefreshSig = xSemaphoreCreateBinary();
    xSemaphoreTake(ledRefreshSig, 0);
    ledFlashTimer = xTimerCreate("ledFlashTimer",
                                 pdMS_TO_TICKS(1000),
                                 pdTRUE,
                                 (void *)0,
                                 vLedArrayFlashTimerCb);
    led.power = 1;
    xTaskCreate(vLedArrayRefreshTask, "vLedArrayRefreshTask", 2048, NULL, 10, NULL);

    jee_uint32_t setData = 10;
    jee_device_control(ledArrayDev, WS2835_CLEAR_SET_CMD, &setData);

    return JEE_EOK;
}

void vTalLedArraySetOnOff(jee_uint8_t onOff)
{

    if (onOff == 0)
    {

        led.colorTarget.red = 0;
        led.colorTarget.green = 0;
        led.colorTarget.blue = 0;
        led.color.red = led.colorTarget.red;
        led.color.green = led.colorTarget.green;
        led.color.blue = led.colorTarget.blue;
        led.flashOnoff = 0;
    }
    else
    {
        if (led.color.red == 0 && led.color.green == 0 && led.color.blue == 0)
        {

            if (led.colorTarget.red == 0 && led.colorTarget.green == 0 && led.colorTarget.blue == 0)
            {
                led.colorTarget.red = 100;
                led.colorTarget.green = 100;
                led.colorTarget.blue = 100;
            }
            led.color.red = led.colorTarget.red;
            led.color.green = led.colorTarget.green;
            led.color.blue = led.colorTarget.blue;
        }
        led.flashOnoff = 1;
    }
    xTimerStop(ledFlashTimer, 0);
    jee_uint32_t setData = 10;
    jee_device_control(ledArrayDev, WS2835_CLEAR_SET_CMD, &setData);
    xSemaphoreGive(ledRefreshSig);
}

jee_uint8_t ucTalLedArrayGetStatus(void)
{
    return led.flashOnoff;
}

void vTalLedArraySetFlashNumber(jee_uint32_t flashNumber)
{
    jee_uint32_t setData = 10;
    jee_device_control(ledArrayDev, WS2835_CLEAR_SET_CMD, &setData);
    if (led.flashOnoff)
    {
        led.flashCount = flashNumber;
        xTimerStart(ledFlashTimer, 0);
    }
    else

        printf("hal_led_array_set_flash_num    %d\r\n", led.flashCount);
}

void vTalLedArraySetFlashMode(FlashMode speed)
{
    // strip->clear(strip, LED_STRIP_NUM);
    switch (speed)
    {
    case ledArryFlashModeFast:
    {
        xTimerChangePeriod(ledFlashTimer, pdMS_TO_TICKS(200), 0);
    }
    break;
    case ledArryFlashModeMiddle:
    {
        xTimerChangePeriod(ledFlashTimer, pdMS_TO_TICKS(500), 0);
    }
    break;
    case ledArryFlashModeSlow:
    {
        xTimerChangePeriod(ledFlashTimer, pdMS_TO_TICKS(1000), 0);
    }
    break;
    default:
        break;
    }
    flashMode = speed;
}

FlashMode xTalLedArrayGetFlashMode(void)
{
    return flashMode;
}

void vTalLedArraySetRgbColor(color_t color)
{

    led.colorTarget.red = led.color.red = color.red;
    led.colorTarget.green = led.color.green = color.green;
    led.colorTarget.blue = led.color.blue = color.blue;
    jee_uint32_t setData = 10;
    jee_device_control(ledArrayDev, WS2835_CLEAR_SET_CMD, &setData);
    xTimerStop(ledFlashTimer, 0);
    if (led.flashOnoff)
        xSemaphoreGive(ledRefreshSig);
}

color_t xTalLedArrayGetRgbColor(void)
{
    return led.colorTarget;
}
