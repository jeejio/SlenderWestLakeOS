#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_waterpump.h"


#define WATER_PUMP_FAST_DUTY (100)
#define WATER_PUMP_MEDIAN_DUTY (70)
#define WATER_PUMP_LOW_DUTY (30)
#define WATER_PUMP_STOP_DUTY (0)

#define WATER_PUMP_DEV "wapump_lb1427b"
jee_device_t water_pump_dev = NULL;

uint8_t onoffFlag = 0;
static waterPumpSpeed_t speedLevel = WATER_PUMP_SPEED_STOP;

jee_int32_t lTalWaterPumpInit(void)
{

    // 查找设备
    water_pump_dev = jee_device_find(WATER_PUMP_DEV);
    if (water_pump_dev == JEE_NULL)
    {
        printf("can not find lb1427b Model\n");
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(water_pump_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pin device\n");
        return JEE_ERROR;
    }
    return JEE_EOK;
}

void vTalSetWaterPumpOnoff(jee_uint8_t OnOff)
{
    uint32_t duty = 0;
    onoffFlag = OnOff;
    if (OnOff == 1)
    {

        printf("water_pump_set_speed %d\n", speedLevel);
        if (speedLevel == WATER_PUMP_SPEED_LOW)
        {
            duty = WATER_PUMP_LOW_DUTY;
        }
        else if (speedLevel == WATER_PUMP_SPEED_MEDIAN)
        {
            duty = WATER_PUMP_MEDIAN_DUTY;
        }
        else if (speedLevel == WATER_PUMP_SPEED_FAST)
        {
            duty = WATER_PUMP_FAST_DUTY;
        }
        else if (speedLevel == WATER_PUMP_SPEED_STOP)
        {
            duty = WATER_PUMP_STOP_DUTY;
        }

        printf("setOnoff vel %d\n", OnOff);

        jee_device_control(water_pump_dev, WATER_PUMP_SET_CMD, (void *)&duty);
    }
    else
    {
        speedLevel = WATER_PUMP_SPEED_STOP;
        duty = WATER_PUMP_STOP_DUTY;
        jee_device_control(water_pump_dev, WATER_PUMP_SET_CMD, (void *)&duty);
    }
}

jee_uint8_t ucTalGetWaterPumpOnoff(void)
{
    jee_uint8_t read_data;
    jee_device_read(water_pump_dev, NULL, &read_data, 1);
    if (read_data == 0)
        return 0;
    else
        return 1;
}

void vTalSetWaterPumpMode(waterPumpSpeed_t speed)
{
    speedLevel = speed;
    uint32_t duty = 0;
    if (onoffFlag == 1)
    {

        printf("water_pump_set_speed %d\n", speed);
        if (speed == WATER_PUMP_SPEED_LOW)
        {
            duty = WATER_PUMP_LOW_DUTY;
        }
        else if (speed == WATER_PUMP_SPEED_MEDIAN)
        {
            duty = WATER_PUMP_MEDIAN_DUTY;
        }
        else if (speed == WATER_PUMP_SPEED_FAST)
        {
            duty = WATER_PUMP_FAST_DUTY;
        }
        else if (speed == WATER_PUMP_SPEED_STOP)
        {
            duty = WATER_PUMP_STOP_DUTY;
        }

        // printf("duty vel %d\n",duty);

        jee_device_control(water_pump_dev, WATER_PUMP_SET_CMD, (void *)&duty);
    }
}

waterPumpSpeed_t ucTalGetWaterPumpMode(void)
{
    return speedLevel;
}
