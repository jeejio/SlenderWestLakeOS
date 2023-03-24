#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_getsture.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#define GETSTURE_DEV "gets_paj7620u2"
jee_device_t getstureDev = NULL;
jee_uint8_t getstureData = 0;
jee_uint8_t getstureUploadData = 0;
static TimerHandle_t GetstureTimer = NULL;

void vTalGetstureTimerCb(TimerHandle_t timer)
{
     jee_uint8_t data = 0;
     jee_device_read(getstureDev, NULL, &data, 1);
    if (data != 0)
    {
        getstureData = data;
        getstureUploadData = data;
    }
}

jee_int32_t lTalGetstureInit(void)
{
    // 查找设备
    getstureDev = jee_device_find(GETSTURE_DEV);
    if (getstureDev == JEE_NULL)
    {
        printf("can not find %s Model\n", GETSTURE_DEV);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(getstureDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", GETSTURE_DEV);
        return JEE_ERROR;
    }

    GetstureTimer = xTimerCreate("GetstureTimer",
                                 pdMS_TO_TICKS(500),
                                 pdTRUE,
                                 (void *)0,
                                 vTalGetstureTimerCb);

    xTimerStart(GetstureTimer, 0);

    return JEE_EOK;
   
}

GestureList ucTalGetGetstureStatus(void)
{

    jee_uint8_t data = 0;
    data = getstureUploadData;
    getstureUploadData = 0;
    return data;
}

GestureList xTalGetstureGetMovingDirection(void)
{
    if (getstureData >= UpGesture && getstureData <= BackwardGesture)
    {
        jee_uint8_t data = getstureData;
        getstureData =0;
        return data;
    }
    else
    {
        return NoGesture;
    }
}

GestureList xTalGetstureGetRotationDirection(void)
{
    if (getstureData >= ClockwiseGesture && getstureData <= AntiClockwiseGesture)
    {
       jee_uint8_t data = getstureData;
        getstureData =0;
        return data;
    }
    else
    {
        return NoGesture;
    }

}

GestureList xTalGetstureGetSwingStatus(void)
{
    if (getstureData == WaveGesture )
    {
        jee_uint8_t data = getstureData;
        getstureData =0;
        return data;
    }
    else
    {
        return NoGesture;
    }

}