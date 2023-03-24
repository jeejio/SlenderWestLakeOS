#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "drv_sensor_dc3v_n20.h"
#include "tal_motor.h"
#include "string.h"
#define MOTOR_DEV "motor_DC3V"
jee_device_t sensor_motor_dev = NULL;
static int GetInfo[3];
int MotorSetInfo;
int *lTalMotorGetInfo(void)
{
    memset(GetInfo, 0, sizeof(GetInfo));
    jee_device_read(sensor_motor_dev, NULL, GetInfo, sizeof(GetInfo));
    return GetInfo;
}

int lTalMotorGetOnoff(void)
{
    lTalMotorGetInfo();
    return GetInfo[2];
}

int lTalMotorGetRotationDirection(void)
{
    lTalMotorGetInfo();
    return GetInfo[1];
}

int lTalMotorGetSpinVelocityLevel(void)
{
    lTalMotorGetInfo();
    return GetInfo[0];
}

void vTalMotorSetSpinVelocityLevel(MotorSpeed_t speed)
{
    MotorSetInfo = speed;
    sensor_motor_dev = jee_device_find(MOTOR_DEV);
    if (sensor_motor_dev == JEE_NULL)
    {
        LOGE("MOTOR", "can not find sensor Model\n");
    }
    else
    {
        LOGI("MOTOR", "find sensor Model ok\n");
        jee_device_control(sensor_motor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 1, &MotorSetInfo);
    }
}

void vTalMotorSetRotationDirection(MotorDirection_t direction)
{
    MotorSetInfo = direction;
    sensor_motor_dev = jee_device_find(MOTOR_DEV);
    if (sensor_motor_dev == JEE_NULL)
    {
        LOGE("MOTOR", "can not find sensor Model\n");
    }
    else
    {
        LOGI("MOTOR", "find sensor Model ok\n");
        jee_device_control(sensor_motor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 2, &MotorSetInfo);
    }
}
void vTalMotorSetOnoff(uint8_t onoff)
{
    MotorSetInfo = onoff;
    sensor_motor_dev = jee_device_find(MOTOR_DEV);
    if (sensor_motor_dev == JEE_NULL)
    {
        LOGE("MOTOR", "can not find sensor Model\n");
    }
    else
    {
        LOGI("MOTOR", "find sensor Model ok\n");
        jee_device_control(sensor_motor_dev, JEE_SENSOR_CTRL_USER_CMD_START + 3, &MotorSetInfo);
    }
}

int lTalMotorInit(void)
{
    // 查找设备
    sensor_motor_dev = jee_device_find(MOTOR_DEV);
    if (sensor_motor_dev == JEE_NULL)
    {
        LOGE("MOTOR", "can not find sensor Model\n");
        return -1;
    }
    else
        LOGI("MOTOR", "find sensor Model ok\n");

    // 打开设备
    jee_err_t result = JEE_EOK;

    result = jee_device_open(sensor_motor_dev, JEE_DEVICE_FLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open senor device\n");
        return -1;
    }
    else
        printf("open senor device ok\n");
    return 0;
}
