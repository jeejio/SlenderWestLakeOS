/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023.2.6       zhengqian    first version
 */
 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
#include "hal_rtc.h"
#include "auto_init.h"


// main
void app_main(void)
{
    printf("rtc example Start...\n");
    rt_components_init();

    printf("[RTC Test]Set RTC 2023-02-06 11:30:36\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    jee_err_t ret = JEE_EOK;
    ret = set_date(2023, 2, 6);
    if(ret != JEE_EOK)
    {
        printf("[RTC Test]Set RTC Date failed\n");
        return;
    }

    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    ret = set_time(11, 30, 36);
    if(ret != JEE_EOK)
    {
        printf("[RTC Test]Set RTC Time failed\n");
        return;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    uint8_t i;
    time_t now;
    for(i = 0; i < 10; i++)
    {
        //printf("[RTC Test]Read RTC Date and Time: ");
        now = time(JEE_NULL);
        printf("[RTC Test]Read RTC Date and Time: %s\n", ctime(&now));

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    return;
}

