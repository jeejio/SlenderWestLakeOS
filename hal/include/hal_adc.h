/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-07     aozima       the first version
 * 2018-11-16     Ernest Chen  add finsh command and update adc function
 * 2022-05-11     Stanley Lwin add finsh voltage conversion command
 */

#ifndef __ADC_H__
#define __ADC_H__
#include <jeedef.h>

struct jee_adc_device;
struct jee_adc_ops
{
    jee_err_t (*enabled)(struct jee_adc_device *device, jee_uint32_t channel, jee_bool_t enabled);
    jee_err_t (*convert)(struct jee_adc_device *device, jee_uint32_t channel, int *value);
    jee_uint8_t (*get_resolution)(struct jee_adc_device *device);
    jee_int16_t (*get_vref) (struct jee_adc_device *device);
};

struct jee_adc_device
{
    struct jee_device parent;
    const struct jee_adc_ops *ops;
};
typedef struct jee_adc_device *jee_adc_device_t;

typedef enum
{
    RT_ADC_CMD_ENABLE = JEE_DEVICE_CTRL_BASE(ADC) + 1,
    RT_ADC_CMD_DISABLE = JEE_DEVICE_CTRL_BASE(ADC) + 2,
    RT_ADC_CMD_GET_RESOLUTION = JEE_DEVICE_CTRL_BASE(ADC) + 3, /* get the resolution in bits */
    RT_ADC_CMD_GET_VREF = JEE_DEVICE_CTRL_BASE(ADC) + 4, /* get reference voltage */
} jee_adc_cmd_t;

jee_err_t jee_hw_adc_register(jee_adc_device_t adc,const char *name, const struct jee_adc_ops *ops, const void *user_data);

jee_uint32_t jee_adc_read(jee_adc_device_t dev, jee_uint32_t channel);
jee_err_t jee_adc_enable(jee_adc_device_t dev, jee_uint32_t channel);
jee_err_t jee_adc_disable(jee_adc_device_t dev, jee_uint32_t channel);
jee_int16_t jee_adc_voltage(jee_adc_device_t dev, jee_uint32_t channel);
int jee_hw_adc_init(void);
#endif /* __ADC_H__ */
