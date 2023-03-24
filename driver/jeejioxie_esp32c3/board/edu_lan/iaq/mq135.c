#include "mq135.h"
#include <stdio.h>
#include "esp32c3/rom/ets_sys.h" //for ets_delay_us()
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "hal_i2c.h"
#include "hal_sensor.h"


#define ioexpand_dev "ioexpand_pca9536"
static jee_device_t ioexpandDev = NULL;

static jee_int32_t ioexpandInit(void)
{
    // 查找设备
    ioexpandDev = jee_device_find(ioexpand_dev);
    if (ioexpandDev == JEE_NULL)
    {
        printf("can not find %s Model\n",ioexpand_dev);
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(ioexpandDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n",ioexpand_dev);
        return JEE_ERROR;
    }
    return JEE_EOK;
}

static jee_int32_t ioexpandGetData(void)
{
    jee_int32_t data = 0;
    jee_device_read(ioexpandDev, NULL, &data, 1);
    return data;
}

jee_int32_t mq135_init(void)
{
    return ioexpandInit();
}


jee_int8_t mq135_get_data(void)
{
  return  ((ioexpandGetData() & 0x04)>>2)?0:1; 

}