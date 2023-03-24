#include "bh1750fvi.h"
#include <stdio.h>
#include "driver/ledc.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "driver/i2c.h"
#include "hal_i2c.h"

jee_device_t bh1750fviDev = NULL;
#define BH1750_ADDR (0x5C)


void bh1750fviInit(void)
{
     bh1750fviDev = jee_device_find("i2c0hard");
    if (bh1750fviDev == JEE_NULL)
    {
        printf("can not find %s Model\n", "i2c0hard");
        return ;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(bh1750fviDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", "i2c0hard");
        return ;
    }
    uint8_t write_data=0;
    write_data=0x01;
    jee_i2c_master_send(bh1750fviDev, BH1750_ADDR, JEE_I2C_STOP, &write_data, 1);
    write_data=0x10;
    jee_i2c_master_send(bh1750fviDev, BH1750_ADDR, JEE_I2C_STOP, &write_data, 1);
   
}
uint16_t bh1750fviGetData(void)
{
    uint8_t readData[3];
    jee_i2c_master_recv(bh1750fviDev, BH1750_ADDR, JEE_I2C_STOP, &readData, 3);
    uint16_t lightness;
    lightness = readData[0]*256+readData[1];
    return lightness;
}

