/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-2-7       zhengqian      the first version
*/

#include "freertos/FreeRTOS.h"
#include "freertos/device.h"
#include "hal_rtc.h"
#include "driver/gptimer.h"
#include <sys/time.h>
#include "auto_init.h"
#include "string.h"
#include "esp_log.h"

#if defined(JEE_USING_SOFT_RTC)
#error "Please CANCEL the JEE_USING_SOFT_RTC option. Make sure there is only one RTC device on the system."
#endif

static struct jee_rtc_device rtc_device;


static jee_err_t jee_rtc_init(void)
{

    return JEE_EOK;
}


jee_err_t jee_get_secs(time_t *sec)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    *sec = (time_t)tv.tv_sec + (time_t)(tv.tv_usec/(1e6));

    return JEE_EOK;
}

jee_err_t jee_set_secs(time_t *sec)
{
    struct timeval tv = {0};
    tv.tv_sec = *sec;
    settimeofday(&tv, NULL);    

    return JEE_EOK;
}

jee_err_t jee_get_timeval(struct timeval *tv)
{
    gettimeofday(tv, NULL);

    return JEE_EOK;
}

jee_err_t jee_set_timeval(struct timeval *tv)
{
    settimeofday(tv, NULL);

    return JEE_EOK;
}

struct jee_rtc_ops rtc_ops = {
    jee_rtc_init,
    jee_get_secs,
    jee_set_secs,
    jee_get_timeval,
    jee_set_timeval
};

int jee_hw_rtc_init(void)
{
    rtc_device.ops = &rtc_ops;
    jee_hw_rtc_register(&rtc_device, "rtc", JEE_DEVICE_FLAG_RDWR, NULL);

    return 0;
}
INIT_DEVICE_EXPORT(jee_hw_rtc_init);
