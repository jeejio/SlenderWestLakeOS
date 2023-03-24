/*
 * Copyright (c) 2023, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-01-31     huzhiqiang     the first version
 */

#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "dc3v_n20.h"   //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "hal_gpio.h"
#include "esp_log.h"
#include "drv_sensor_dc3v_n20.h"

#define SENSOR_MOTOR_NAME "motor_pwm"

jee_size_t sensorMotorReadStatus(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
    if (buf == NULL)
        return 0;

    motor_get_info((int *)buf);
    return (strlen(buf) - 1);
}

jee_err_t sensorMotorControl(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("cmd %d \n", cmd);
    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : motor_init \n");
            motor_init();
            printf(" end   : motor_init \n");
            return JEE_EOK;
        }
        break;
        case JEE_SENSOR_POWER_DOWN:
        {
        }
        break;
        default:
            break;
        }
    }
    break;
    case SENSOR_MOTOR_SET_SPEED:
    {
        motor_set_speed(*(int *)arg);
        return JEE_EOK;
    }
    break;
    case SENSOR_MOTOR_SET_DIRECTION:
    {
        motor_set_direction(*(int *)arg);
        return JEE_EOK;
    }
    break;
    case SENSOR_MOTOR_SET_ONOFF:
    {
        motor_set_switch(*(int *)arg);
        return JEE_EOK;
    }
    break;
    default:
        break;
    }
    return JEE_EOK;
}

static struct jee_sensor_ops sensor_ops =
    {
        sensorMotorReadStatus,
        sensorMotorControl};

int sensorMotorInit(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_motor = JEE_NULL;
    /* sensor register */
    sensor_motor = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_motor == JEE_NULL)
        return -1;
    memset(sensor_motor, 0, sizeof(struct jee_sensor_device));
    sensor_motor->info.type = JEE_SENSOR_CLASS_MOTOR;
    sensor_motor->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_motor->info.model = "DC3V";
    // sensor_motor->info.intf_type = JEE_SENSOR_INTF_I2C;

    if (cfg != JEE_NULL)
        memcpy(&sensor_motor->config, cfg, sizeof(struct jee_sensor_config));

    sensor_motor->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_motor, name, JEE_DEVICE_FLAG_RDWR, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_motor)
        vPortFree(sensor_motor);

    return -JEE_ERROR;
}

int jeeHwSensorMotorInit()
{
    //printf("begin to execute jeeHwSensorMotorInit\n");
    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = SENSOR_MOTOR_NAME;
    sensorMotorInit("DC3V", &cfg);
    //printf("finish to execute jeeHwSensorMotorInit\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jeeHwSensorMotorInit);
