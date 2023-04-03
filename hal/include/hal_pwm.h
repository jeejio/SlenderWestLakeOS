/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-2-2       zhengqian    first version
 */

#ifndef __PWM_H_INCLUDE__
#define __PWM_H_INCLUDE__

#include "freertos/device.h"
#include "freertos/FreeRTOS.h"


#define PWM_CMD_ENABLE      (JEE_DEVICE_CTRL_BASE(PWM) + 0)
#define PWM_CMD_DISABLE     (JEE_DEVICE_CTRL_BASE(PWM) + 1)
#define PWM_CMD_SET         (JEE_DEVICE_CTRL_BASE(PWM) + 2)
#define PWM_CMD_GET         (JEE_DEVICE_CTRL_BASE(PWM) + 3)
#define PWMN_CMD_ENABLE     (JEE_DEVICE_CTRL_BASE(PWM) + 4)
#define PWMN_CMD_DISABLE    (JEE_DEVICE_CTRL_BASE(PWM) + 5)
#define PWM_CMD_SET_PERIOD  (JEE_DEVICE_CTRL_BASE(PWM) + 6)
#define PWM_CMD_SET_PULSE   (JEE_DEVICE_CTRL_BASE(PWM) + 7)
#define PWM_CMD_INIT        (JEE_DEVICE_CTRL_BASE(PWM) + 8)
#define PWM_CMD_SET_DUTY    (JEE_DEVICE_CTRL_BASE(PWM) + 9)
#define PWM_CMD_UPDATE_DUTY (JEE_DEVICE_CTRL_BASE(PWM) + 10)


struct jee_pwm_configuration
{
    jee_uint32_t channel; /* 1-n or 0-n, which depends on specific MCU requirements */
    jee_uint32_t period;  /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
    jee_uint32_t pulse;   /* unit:ns (pulse<=period) */

    jee_uint32_t timer_num;
    jee_uint32_t speed_mode;
    jee_uint32_t pin_num;
    jee_uint32_t duty;
    uint8_t bit_resolution; /*duty_resolution:1bit 2bit 3bit 4bit eg... */

    /*
     * JEE_TRUE  : The channel of pwm is complememtary.
     * JEE_FALSE : The channel of pwm is nomal.
    */
    jee_bool_t  complementary;
};

struct jee_device_pwm;
struct jee_pwm_ops
{
    jee_err_t (*control)(struct jee_device_pwm *device, int cmd, void *arg);
};

struct jee_device_pwm
{
    struct jee_device parent;
    const struct jee_pwm_ops *ops;
};

jee_err_t jee_device_pwm_register(struct jee_device_pwm *device, const char *name, const struct jee_pwm_ops *ops, const void *user_data);

jee_err_t jee_pwm_enable(struct jee_device_pwm *device, int channel);
jee_err_t jee_pwm_disable(struct jee_device_pwm *device, int channel);
jee_err_t jee_pwm_set(struct jee_device_pwm *device, int channel, jee_uint32_t period, jee_uint32_t pulse);
jee_err_t jee_pwm_set_period(struct jee_device_pwm *device, int channel, jee_uint32_t period);
jee_err_t jee_pwm_set_pulse(struct jee_device_pwm *device, int channel, jee_uint32_t pulse);

#endif /* __PWM_H_INCLUDE__ */
