#include "lb1427b.h"
#include <stdio.h>
#include "driver/ledc.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "hal/gpio_types.h"
#define WATER_PUMP_OUTPUT_GPIO 2

#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE /*!< LEDC high speed speed_mode */
#define PWM_OUTPUT_IO (3)            // Define the output GPIO
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_DUTY (4095)    // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define PWM_FREQUENCY (50) // Frequency in Hertz. Set frequency at 5 kHz

#define PWM_DUTY_MAX (8190)

jee_device_t pwmDev = NULL;
struct jee_pwm_configuration cfg;
jee_uint8_t pwmDuty = 0;

void lb1427Init(void)
{
    pwmDev = jee_device_find("pwm");
    if (pwmDev == JEE_NULL)
    {
        printf("can not find pwm Model\n");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(pwmDev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pin device\n");
        return;
    }

    cfg.channel = PWM_CHANNEL;
    cfg.timer_num = PWM_TIMER;
    cfg.duty = 0;
    cfg.period = (1e9) / PWM_FREQUENCY;
    cfg.pin_num = PWM_OUTPUT_IO;
    cfg.speed_mode = PWM_MODE;
    // 配置设备
    jee_device_control(pwmDev, PWM_CMD_INIT, (void *)&cfg);

#if 1
    jee_pin_mode(WATER_PUMP_OUTPUT_GPIO, PIN_MODE_OUTPUT);
    jee_pin_write(WATER_PUMP_OUTPUT_GPIO, 0);
#endif

    pwmDuty = 0;
}

void lb1427SetPwmDuty(jee_uint8_t value)
{
    if (value > 100)
        return;
    pwmDuty = value;
    cfg.duty = (jee_uint32_t)(((float)(PWM_DUTY_MAX * value)) * 0.01);

    jee_device_control(pwmDev, PWM_CMD_SET_DUTY, (void *)&cfg);
    jee_device_control(pwmDev, PWM_CMD_UPDATE_DUTY, (void *)&cfg);
}

jee_uint8_t lb1427GetPwmDuty(void)
{
    return pwmDuty;
}
