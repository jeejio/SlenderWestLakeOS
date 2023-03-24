/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 20222.12.13    zhengqian    first version
 */
 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
#include "hal_gpio.h"
#include "auto_init.h"

//本无需包含该头文件，这里暂时模拟系统启动初始化，自动加载jee_hw_gpio_init函数
//#include "driver/drv_gpio_IoModel.h"


//static const char *TAG = "gpio io_Model example"; // 用于log

static uint8_t s_led_state = 0;  // 初始灯灭

// 通过管脚19控制灯
struct jee_device_pin_mode gpiomode_19 = 
{
    .pin = 19,
    .mode = PIN_MODE_OUTPUT
};

struct jee_device_pin_status gpiostatus_19 = 
{
    .pin = 19,
    .status = 1    // 1/0
};

jee_device_t gpio_dev = NULL;

// 控制灯闪烁
void  blink_gpio_led()
{
    gpiostatus_19.status = s_led_state; // 1/0
    jee_device_write(gpio_dev, NULL, (void *)&gpiostatus_19, sizeof(struct jee_device_pin_status));
}

// main
void app_main(void)
{
    printf("gpio example Start...\n");

    rt_components_init();

    // 模拟启动注册，替代：INIT_BOARD_EXPORT(jee_hw_gpio_init);
    //jee_hw_gpio_init();
    //vTaskDelay(1*1000 / portTICK_PERIOD_MS);
    //printf("register the pin Model done\n");

    // 查找设备
    gpio_dev = jee_device_find("pin");
    if (gpio_dev == JEE_NULL)
    {
        printf("can not find pin Model\n");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(gpio_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pin device\n");
        return;
    }

    // 配置设备
    jee_device_control(gpio_dev, NULL, (void *)&gpiomode_19);

    // 读写设备：gpio控制灯闪烁
    while (1) 
    {
        printf("Turning the LED %s!\n", s_led_state == true ? "ON" : "OFF");
        blink_gpio_led();

        s_led_state = !s_led_state; // 状态翻转
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


