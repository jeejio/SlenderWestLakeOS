#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal_gpio.h"
#include "auto_init.h"
#include "single_switch.h"

#define SINGLE_SWITCH_IO (9)

#define SWITCH_CHECK_CLICK_TIME 25      // 50*20MS =1000MS
#define SWITCH_CHECK_LONG_PRESS_TIME 50 // 100*20MS =2000MS

#define SWITCH_CLICK 1
#define SWITCH_DOUBLE_CLICK 2
#define SWITCH_LONG_PRESS 3

static const char *TAG = "SINGLE_SWITCH";
static uint32_t switch_count = 0;
static uint32_t first_switch_count = 0; /**/
static uint8_t check_count = 0;
static uint8_t check_down = SWITCH_CHECK_CLICK_TIME;
static uint8_t check_key_status = 0;
static uint8_t switch_status=1;
static uint8_t read_key(void)
{
    int ret = jee_pin_read(SINGLE_SWITCH_IO) ? 1 : 0;
    return ret;
}

static void single_switch_gpio_mode(void)
{
    jee_pin_mode(SINGLE_SWITCH_IO, PIN_MODE_INPUT_PULLUP);
}

static int switch_key_scan(void)
{
    check_count++;

    if (check_count == 1)
    {
        first_switch_count = switch_count;
        return 0;
    }

    if (check_count == 2)
    {
        uint32_t times;
        times = switch_count - first_switch_count;
        if (times > SWITCH_CHECK_LONG_PRESS_TIME)
        {
            check_count = 0;
            printf("\n long click \n");
            return SWITCH_LONG_PRESS;
        }
        if (times > SWITCH_CHECK_CLICK_TIME)
        {
            check_count = 0;
            printf("\n click \n");
            return SWITCH_CLICK;
        }
    }

    if (check_count == 3)
    {
        check_down = SWITCH_CHECK_CLICK_TIME;
        return 0;
    }
    if (check_count == 4)
    {
        check_count = 0;
        printf("\n double click \n");
        return SWITCH_DOUBLE_CLICK;
    }
    return 0;
}

static void single_switch_task(void *arg)
{
    while (1)
    {
        if ((read_key() != switch_status))
        {
            check_key_status = switch_key_scan();
        }
        if (check_count == 2)
        {
            check_down--;
            if (check_down == 0)
            {
                check_down = SWITCH_CHECK_CLICK_TIME;
                check_count--;
                check_key_status = switch_key_scan();
            }
        }
        switch_status = read_key();
        switch_count++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void single_switch_clean_mode(void)
{
    /*清除按键(单击 双击 长按的状态)*/
    check_key_status=0;
}

void single_switch_get_info(int *buf)
{
    buf[0]=check_key_status;
    buf[1]=1-switch_status;
}


void single_switch_init(void)
{
    single_switch_gpio_mode();
    xTaskCreate(single_switch_task, "single_switch_task", 1024, NULL, 5, NULL);
}
