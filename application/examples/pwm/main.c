/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023.2.3       zhengqian    first version
 */
 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
#include "hal_pwm.h"
#include "auto_init.h"
#include "driver/ledc.h"



#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095

jee_device_t pwm_dev = NULL;

// main
void app_main(void)
{
    printf("pwm ledc example Start...\n");
    rt_components_init();

    // 查找设备
    pwm_dev = jee_device_find("pwm");
    if (pwm_dev == JEE_NULL)
    {
        printf("can not find pwm Model\n");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(pwm_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pwm device\n");
        return;
    }

    // 初始化设备
    struct jee_pwm_configuration pwm_cfg = {
        .channel = LEDC_CHANNEL,
        .speed_mode = LEDC_MODE,
        .period = 1000*200,    // 单位ns,这里等价于5khz
        .timer_num = LEDC_TIMER,
        .pin_num = LEDC_OUTPUT_IO
        };

    jee_device_control(pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg);

    // 操控设备:Set duty
    pwm_cfg.duty = LEDC_DUTY;
    jee_device_control(pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg);
    jee_device_control(pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg);

    // 不使能设备：两种方法效果一样
    vTaskDelay(1000 / portTICK_PERIOD_MS);
#if  1
    jee_device_control(pwm_dev, PWM_CMD_DISABLE, NULL);

#else 
    jee_pwm_disable((struct jee_device_pwm *)pwm_dev, pwm_cfg.channel);
#endif

    // 使能设备:使用jee_pwm_enable时，需要重新初始化，并设置设备；通过jee_device_control时，可直接恢复到以前的状态。
    vTaskDelay(1000 / portTICK_PERIOD_MS);
#if  1
    jee_device_control(pwm_dev, PWM_CMD_ENABLE, (void *)&pwm_cfg);

#else
    jee_pwm_enable((struct jee_device_pwm *)pwm_dev, pwm_cfg.channel);
    jee_device_control(pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg);
    jee_device_control(pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg);
    jee_device_control(pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg);
#endif
}


