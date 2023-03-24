/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-12-07     zhengqian      the first version
*/

#ifndef __JEE_GPIO_H__
#define __JEE_GPIO_H__

#include "device.h"
#include "hal_gpio.h"           //gpio IO模型驱动框架头文件
#include "driver/gpio.h"        //厂家驱动头文件
#include "auto_init.h"


static void jee_gpio_write(struct jee_device *device, jee_base_t pin, jee_base_t value)
{
    gpio_set_level(pin, value);
    //printf("set %d level %d \n", (int)pin, (int)value);
}

static int jee_gpio_read(struct jee_device *device, jee_base_t pin)
{
    return gpio_get_level(pin);
}

static void jee_gpio_mode(struct jee_device *device, jee_base_t pin, jee_base_t mode)
{
    // typedef enum {
    //     GPIO_MODE_DISABLE = GPIO_MODE_DEF_DISABLE,                                                         /*!< GPIO mode : disable input and output             */
    //     GPIO_MODE_INPUT = GPIO_MODE_DEF_INPUT,                                                             /*!< GPIO mode : input only                           */
    //     GPIO_MODE_OUTPUT = GPIO_MODE_DEF_OUTPUT,                                                           /*!< GPIO mode : output only mode                     */
    //     GPIO_MODE_OUTPUT_OD = ((GPIO_MODE_DEF_OUTPUT) | (GPIO_MODE_DEF_OD)),                               /*!< GPIO mode : output only with open-drain mode     */
    //     GPIO_MODE_INPUT_OUTPUT_OD = ((GPIO_MODE_DEF_INPUT) | (GPIO_MODE_DEF_OUTPUT) | (GPIO_MODE_DEF_OD)), /*!< GPIO mode : output and input with open-drain mode*/
    //     GPIO_MODE_INPUT_OUTPUT = ((GPIO_MODE_DEF_INPUT) | (GPIO_MODE_DEF_OUTPUT)),                         /*!< GPIO mode : output and input mode                */
    // } gpio_mode_t;

    // #define PIN_MODE_OUTPUT         0x00
    // #define PIN_MODE_INPUT          0x01
    // #define PIN_MODE_INPUT_PULLUP   0x02
    // #define PIN_MODE_INPUT_PULLDOWN 0x03
    // #define PIN_MODE_OUTPUT_OD      0x04

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    //
    if (mode == PIN_MODE_OUTPUT)
    {
        io_conf.mode = GPIO_MODE_OUTPUT;
        //gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        printf("set mode  PIN_MODE_OUTPUT \n");
    }
    else if(mode == PIN_MODE_INPUT)
    {
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        //gpio_set_direction(pin, GPIO_MODE_INPUT);
    }
    else if(mode == PIN_MODE_OUTPUT_OD)
    {
        io_conf.mode = GPIO_MODE_OUTPUT_OD;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        //gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    }
    else if(mode == PIN_MODE_INPUT_PULLUP)
    {
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        io_conf.mode = GPIO_MODE_INPUT;
    }
    else if(mode == PIN_MODE_INPUT_PULLDOWN)
    {
        io_conf.pull_down_en = 1;
        io_conf.pull_up_en = 0;
        io_conf.mode = GPIO_MODE_INPUT;
    }
    gpio_config(&io_conf);
}

static jee_err_t jee_gpio_attach_irq(struct jee_device *device, jee_int32_t pin,
                                  jee_uint32_t mode, void (*hdr)(void *args), void *args)
{
    if (pin < 0)
    {
        return JEE_ENOSYS;
    }

    portDISABLE_INTERRUPTS();

    /*irq mode set*/
    switch (mode)
    {
    case PIN_IRQ_MODE_RISING:
        gpio_set_intr_type(pin, GPIO_INTR_POSEDGE); 
        break;
    case PIN_IRQ_MODE_FALLING:
        gpio_set_intr_type(pin, GPIO_INTR_NEGEDGE); 
        break;
    case PIN_IRQ_MODE_RISING_FALLING:
        gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE); 
        break;
    case PIN_IRQ_MODE_HIGH_LEVEL:
        gpio_set_intr_type(pin, GPIO_INTR_HIGH_LEVEL); 
        break;
    case PIN_IRQ_MODE_LOW_LEVEL:
        gpio_set_intr_type(pin, GPIO_INTR_LOW_LEVEL); 
        break;
    default:
        portENABLE_INTERRUPTS();
        return JEE_ENOSYS;
    }

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);//设置中断优先级最低
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(pin, hdr, args);//注册中断处理程序

    portENABLE_INTERRUPTS();
    return JEE_EOK;
}

static jee_err_t jee_gpio_detach_irq(struct jee_device *device, jee_int32_t pin)
{
    if (pin < 0)
    {
        return JEE_ENOSYS;
    }
    portDISABLE_INTERRUPTS();

    gpio_isr_handler_remove(pin);//删除中断处理服务

    portENABLE_INTERRUPTS();
    return JEE_EOK;
}

static jee_err_t jee_gpio_irq_enable(struct jee_device *device, jee_base_t pin, jee_uint32_t enabled)
{
    if (pin < 0)
    {
        return JEE_ENOSYS;
    }
    portDISABLE_INTERRUPTS();

    if (enabled == PIN_IRQ_ENABLE)
    {
        //tls_clr_gpio_irq_status((enum tls_io_name)gpio_pin);
        gpio_intr_enable(pin);

        portENABLE_INTERRUPTS();
        return JEE_EOK;
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        gpio_intr_disable(pin);
        portENABLE_INTERRUPTS();
        return JEE_EOK;
    }
    else
    {
        portENABLE_INTERRUPTS();
        return JEE_ENOSYS;
    }
}


struct jee_pin_ops _jee_gpio_ops =
{
    jee_gpio_mode,
    jee_gpio_write,
    jee_gpio_read,
    jee_gpio_attach_irq,
    jee_gpio_detach_irq,
    jee_gpio_irq_enable,
    NULL
};

int jee_hw_gpio_init(void)
{
    //printf("begin to jee_hw_gpio_init\n");
    int ret = -1;
    ret = jee_device_pin_register("pin", &_jee_gpio_ops, JEE_NULL);

    //printf("finish to jee_hw_gpio_init\n");
    return ret;
}

INIT_BOARD_EXPORT(jee_hw_gpio_init);

#endif