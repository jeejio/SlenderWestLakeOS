#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_magnetic.h"
#include "jeedef.h"

#define MAG_DEV "mag_cc6201st"
jee_device_t sensor_mag_dev = NULL;

jee_int32_t lTalMagneticInit(void)
{

    // 查找设备
    sensor_mag_dev = jee_device_find(MAG_DEV);
    if (sensor_mag_dev == JEE_NULL)
    {
        printf("can not find cc6201st Model\n");
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(sensor_mag_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pin device\n");
        return JEE_ERROR;
    }
    return JEE_EOK;
}

jee_uint8_t ucTalGetMagStatus(void)
{
    jee_uint8_t data = 0;
    jee_device_read(sensor_mag_dev, NULL, &data, 1);

    return data;
}

jee_uint8_t ucTalgetMagnetic(void)
{
    return ucTalGetMagStatus();
}

jee_uint8_t ucTalGetIsMagnetic(jee_uint8_t id)
{
    jee_uint8_t data = ucTalGetMagStatus();
    jee_uint8_t res = 0;
    switch (id)
    {
    case 1:
    {
        if (data & 0x01)
        {
            res = 1;
        }
        else
        {
            res = 0;
        }
    }
    break;
    case 2:
    {
        if (data & 0x02)
        {
            res = 1;
        }
        else
        {
            res = 0;
        }
    }
    break;
    case 3:
    {
        if (data & 0x04)
        {
            res = 1;
        }
        else
        {
            res = 0;
        }
    }
    break;
    default:

        break;
    }
    return res;
}
