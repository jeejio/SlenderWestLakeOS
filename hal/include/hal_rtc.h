/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-1-11      zhengqian    first version
 */

#ifndef __RTC_H__
#define __RTC_H__

#include <freertos/device.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JEE_DEVICE_CTRL_RTC_GET_TIME     (JEE_DEVICE_CTRL_BASE(RTC) + 0x01)              /**< get second time */
#define JEE_DEVICE_CTRL_RTC_SET_TIME     (JEE_DEVICE_CTRL_BASE(RTC) + 0x02)              /**< set second time */
#define JEE_DEVICE_CTRL_RTC_GET_TIMEVAL  (JEE_DEVICE_CTRL_BASE(RTC) + 0x03)              /**< get timeval for gettimeofday */
#define JEE_DEVICE_CTRL_RTC_SET_TIMEVAL  (JEE_DEVICE_CTRL_BASE(RTC) + 0x04)              /**< set timeval for gettimeofday */


struct jee_rtc_ops
{
    jee_err_t (*init)(void);
    jee_err_t (*get_secs)(time_t *sec);
    jee_err_t (*set_secs)(time_t *sec);
    jee_err_t (*get_timeval)(struct timeval *tv);
    jee_err_t (*set_timeval)(struct timeval *tv);
};

typedef struct jee_rtc_device
{
    struct jee_device parent;
    const struct jee_rtc_ops *ops;
} jee_rtc_dev_t;

jee_err_t jee_hw_rtc_register(jee_rtc_dev_t  *rtc,
                            const char    *name,
                            jee_uint32_t    flag,
                            void          *data);

jee_err_t set_date(jee_uint32_t year, jee_uint32_t month, jee_uint32_t day);
jee_err_t set_time(jee_uint32_t hour, jee_uint32_t minute, jee_uint32_t second);
jee_err_t set_timestamp(time_t timestamp);
jee_err_t get_timestamp(time_t *timestamp);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */
