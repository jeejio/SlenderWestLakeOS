#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h"     //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_pir.h"
#include "drv_sensor_sbf01.h"

#define PIR_DEV "pir_PIR"
jee_device_t sensor_pir_dev = NULL;
static unsigned char sensorPirValue = 0;

void vTalPirInit(void)
{
    // 查找设备
    sensor_pir_dev = jee_device_find(PIR_DEV);
    if (sensor_pir_dev == JEE_NULL)
    {
        LOGE("PIR", "can not find sensor Model\n");
        return;
    }
    else
        LOGI("PIR", "find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(sensor_pir_dev, JEE_DEVICE_FLAG_RDONLY);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return;
    }
    else
        printf("open senor device ok\n");

    // 初始化设备
    jee_device_control(sensor_pir_dev, SENSOR_PIR_INIT, NULL);
    
}

jee_base_t lTalGetIsExists(void)
{
    jee_device_read(sensor_pir_dev, 0, &sensorPirValue, 1);
    return sensorPirValue;
}