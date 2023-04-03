/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-2-3       zhengqian      the first version
*/

#include "freertos/FreeRTOS.h"
#include "freertos/device.h"
#include <driver/ledc.h>
#include "hal_pwm.h"
#include "auto_init.h"
#include "string.h"
#include "esp_log.h"


static void pwm_config(struct jee_pwm_configuration *cfg)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = cfg->speed_mode,
        .timer_num        = cfg->timer_num,
        .duty_resolution  = (ledc_timer_bit_t)(cfg->bit_resolution),//LEDC_TIMER_13_BIT,
        .freq_hz          = (1e9)/(cfg->period),
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = cfg->speed_mode,
        .channel        = cfg->channel,
        .timer_sel      = cfg->timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = cfg->pin_num,
        .duty           = cfg->duty,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static jee_err_t jee_pwm_control(struct jee_device_pwm *device, int cmd, void *arg)
{
    static struct jee_pwm_configuration pwm_cfg = {0};

    if(arg != NULL)
        memcpy((void *)&pwm_cfg, arg, sizeof(struct jee_pwm_configuration));

    if (pwm_cfg.channel > 5) // esp c3 最多只支持0-5通道
        return JEE_EINVAL;

    switch (cmd)
    {
    case PWM_CMD_INIT:
        pwm_cfg.duty = 0;
        pwm_config(&pwm_cfg);
        printf("PWM_CMD_INIT return ok\n");
        return JEE_EOK;
    case PWM_CMD_SET_DUTY:
        ESP_ERROR_CHECK(ledc_set_duty(pwm_cfg.speed_mode, pwm_cfg.channel, pwm_cfg.duty));
        //printf("PWM_CMD_SET_DUTY return ok\n");
        return JEE_EOK;
     case PWM_CMD_UPDATE_DUTY:
        ESP_ERROR_CHECK(ledc_update_duty(pwm_cfg.speed_mode, pwm_cfg.channel));
        //printf("PWM_CMD_UPDATE_DUTY return ok\n");
        return JEE_EOK;
    case PWM_CMD_ENABLE:
        ESP_ERROR_CHECK(ledc_set_duty(pwm_cfg.speed_mode, pwm_cfg.channel, pwm_cfg.duty)); // 恢复上一次的duty
        ESP_ERROR_CHECK(ledc_update_duty(pwm_cfg.speed_mode, pwm_cfg.channel));
        printf("PWM_CMD_ENABLE return ok\n");
        return JEE_EOK;
    case PWM_CMD_DISABLE: //todo
        ESP_ERROR_CHECK(ledc_set_duty(pwm_cfg.speed_mode, pwm_cfg.channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(pwm_cfg.speed_mode, pwm_cfg.channel));
        printf("PWM_CMD_DISABLE return ok\n");
        return JEE_EOK;
     case PWM_CMD_GET:
        memcpy(arg, (void *)&pwm_cfg, sizeof(struct jee_pwm_configuration));
        printf("PWM_CMD_GET return ok\n");
        return JEE_EOK;
    case PWM_CMD_SET: //todo
        pwm_config(&pwm_cfg);
        printf("PWM_CMD_SET return ok\n");
        return JEE_EOK;
    default:
        return JEE_EINVAL;
    }
}

static struct jee_pwm_ops drv_ops =
{
    jee_pwm_control
};
static struct jee_device_pwm jee_pwm;

int jee_hw_pwm_init(void)
{
    jee_device_pwm_register(&jee_pwm, "pwm", &drv_ops, 0);
    return JEE_EOK;
}

INIT_PREV_EXPORT(jee_hw_pwm_init);
