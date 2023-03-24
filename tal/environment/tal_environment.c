#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_environment.h"


#define ambient_light_dev "li_bh1750fvi"
jee_device_t ambientLightDev = NULL;

#define tem_hum_dev "temp_DHT20"
jee_device_t temHumDev = NULL;

#define ioexpand_dev "ioexpand_pca9536"
jee_device_t ioexpandDev = NULL;

#define mq2_dev "smoke_mq2"
jee_device_t Mq2Dev = NULL;

#define mq7_dev "co_mq7"
jee_device_t Mq7Dev = NULL;

#define mq135_dev "iaq_mq135"
jee_device_t Mq135Dev = NULL;

jee_int32_t lTalAmbientLightInit(void)
{
    // 查找设备
    ambientLightDev = jee_device_find(ambient_light_dev);
    if (ambientLightDev == JEE_NULL)
    {
        printf("can not find %s Model\n",ambient_light_dev);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(ambientLightDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n",ambient_light_dev);
        return JEE_ERROR;
    }
    return JEE_EOK;
}

jee_uint16_t usTalGetAmbientLightData(void)
{
    jee_uint16_t data = 0;
    jee_device_read(ambientLightDev, NULL, &data, 2);
    return data;
}

jee_int32_t lTalTempHumInit(void)
{
    // 查找设备
    temHumDev = jee_device_find(tem_hum_dev);
    if (temHumDev == JEE_NULL)
    {
        printf("can not find %s Model\n",tem_hum_dev);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(temHumDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n",tem_hum_dev);
        return JEE_ERROR;
    }
    return JEE_EOK;
}

void vTalGetTemHumData(jee_int32_t *data)
{
    jee_device_read(temHumDev, NULL, data, 2);
}

jee_int32_t lTalMq2Init(void)
{
    Mq2Dev = jee_device_find(mq2_dev);
    if (Mq2Dev == JEE_NULL)
    {
        printf("can not find %s Model\n",mq2_dev);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(Mq2Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n",mq2_dev);
        return JEE_ERROR;
    }
    return JEE_EOK;
}

jee_uint8_t ucTalMq2GetSmokeStatus(void)
{
    jee_uint8_t data=0;
    jee_device_read(Mq2Dev, NULL, &data, 1);
    return data;
}

jee_int32_t lTalMq7Init(void)
{
    Mq7Dev = jee_device_find(mq7_dev);
    if (Mq7Dev == JEE_NULL)
    {
        printf("can not find %s Model\n",mq7_dev);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(Mq7Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n",mq7_dev);
        return JEE_ERROR;
    }
    return JEE_EOK;
}

jee_uint8_t ucTalMq7GetCoStatus(void)
{
    jee_uint8_t data=0;
    jee_device_read(Mq7Dev, NULL, &data, 1);
    return data;
}


jee_int32_t lTalMq135Init(void)
{
    Mq135Dev = jee_device_find(mq135_dev);
    if (Mq135Dev == JEE_NULL)
    {
        printf("can not find %s Model\n",mq135_dev);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(Mq135Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n",mq135_dev);
        return JEE_ERROR;
    }
    return JEE_EOK;
}

jee_uint8_t ucTalMq135GetAirStatus(void)
{
    jee_uint8_t data=0;
    jee_device_read(Mq135Dev, NULL, &data, 1);
    return data;
}
