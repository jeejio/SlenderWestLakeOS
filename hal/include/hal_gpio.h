/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-30     zhengqian    the first version
 */

#ifndef PIN_H__
#define PIN_H__

#include "freertos/device.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* pin device and operations*/
struct jee_device_pin
{
    struct jee_device parent;
    const struct jee_pin_ops *ops;
};

#define PIN_LOW                 0x00
#define PIN_HIGH                0x01

#define PIN_MODE_OUTPUT         0x00
#define PIN_MODE_INPUT          0x01
#define PIN_MODE_INPUT_PULLUP   0x02
#define PIN_MODE_INPUT_PULLDOWN 0x03
#define PIN_MODE_OUTPUT_OD      0x04

#define PIN_IRQ_MODE_RISING             0x00
#define PIN_IRQ_MODE_FALLING            0x01
#define PIN_IRQ_MODE_RISING_FALLING     0x02
#define PIN_IRQ_MODE_HIGH_LEVEL         0x03
#define PIN_IRQ_MODE_LOW_LEVEL          0x04

#define PIN_IRQ_DISABLE                 0x00
#define PIN_IRQ_ENABLE                  0x01

#define PIN_IRQ_PIN_NONE                -1

struct jee_device_pin_mode
{
    jee_uint16_t pin;
    jee_uint16_t mode;
};
struct jee_device_pin_status
{
    jee_uint16_t pin;
    jee_uint16_t status;
};
struct jee_pin_irq_hdr
{
    jee_int16_t        pin;
    jee_uint16_t       mode;
    void (*hdr)(void *args);
    void             *args;
};
struct jee_pin_ops
{
    void (*pin_mode)(struct jee_device *device, jee_base_t pin, jee_base_t mode);
    void (*pin_write)(struct jee_device *device, jee_base_t pin, jee_base_t value);
    int (*pin_read)(struct jee_device *device, jee_base_t pin);
    jee_err_t (*pin_attach_irq)(struct jee_device *device, jee_int32_t pin,
                      jee_uint32_t mode, void (*hdr)(void *args), void *args);
    jee_err_t (*pin_detach_irq)(struct jee_device *device, jee_int32_t pin);
    jee_err_t (*pin_irq_enable)(struct jee_device *device, jee_base_t pin, jee_uint32_t enabled);
    jee_base_t (*pin_get)(const char *name);
};

int jee_device_pin_register(const char *name, const struct jee_pin_ops *ops, void *user_data);

void jee_pin_mode(jee_base_t pin, jee_base_t mode);
void jee_pin_write(jee_base_t pin, jee_base_t value);
int  jee_pin_read(jee_base_t pin);
jee_err_t jee_pin_attach_irq(jee_int32_t pin, jee_uint32_t mode,
                             void (*hdr)(void *args), void  *args);
jee_err_t jee_pin_detach_irq(jee_int32_t pin);
jee_err_t jee_pin_irq_enable(jee_base_t pin, jee_uint32_t enabled);
/* Get pin number by name,such as PA.0,P0.12 */
jee_base_t jee_pin_get(const char *name);

#ifdef __cplusplus
}
#endif

#endif
