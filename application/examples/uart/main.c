/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 *
 */




/*延时1000ms*/
#if 0
void delay(void) //误差0us
{
    unsigned char a,b,c;
    for(c=167;c>0;c--)
    for(b=171;b>0;b--)
    for(a=16;a>0;a--);
    //_nop_;
    //if Keil,require use intrins.h
}
#endif

//UART Demo
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/drv_uart_iomodel.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "device.h"
#include "auto_init.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD  4  //(CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD  5  //(CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      1
#define ECHO_UART_BAUD_RATE     115200//(CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    8192

#define UART0_NAME "uart0"
#define UART1_NAME "uart1"

static const char *TAG = "UART TEST";

#define BUF_SIZE (128)

struct uart_plat_data {
    jee_uint8_t  *xBuf;
    jee_uint32_t bufSize;
    jee_int32_t  real_size;
};

static void echo_task(void *arg)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    struct uart_plat_data xUartData = {
        .xBuf = data,
        .bufSize = BUF_SIZE - 1,
        .real_size = 0,
    };

    jee_device_t uart_dev = jee_device_find(UART0_NAME);

    if (uart_dev == JEE_NULL)
    {
        //LogError(TAG, "can not find uart device: %s\n", UART0_NAME);
        printf("can not find uart device: %s\n", UART0_NAME);
    }
    else
    {
        if (jee_device_open(uart_dev, JEE_DEVICE_OFLAG_RDWR) != JEE_EOK)
        {
            //LogError(TAG, "open uart device failed!\n");
            printf("open uart device failed!\n");
        }
    }

    while (1) {
        memset((void *)data, 0, BUF_SIZE);
        // Read data from the UART
        jee_err_t ret = jee_device_control(uart_dev, JEE_DEVICE_CTRL_UART_READ, (void *)&xUartData);
        if (JEE_EOK == ret) {
            data[xUartData.real_size] = '\0';
            //LogInfo(TAG, "%s: Recv str: %s\n", __func__, (char *) data);
            printf("%s: Recv str: %s\n", __func__, (char *) data);
        } else {
            //LogError(TAG, "%s: Recv error\n", __func__);
            printf("%s: Recv error\n", __func__);
        }

        // Write data back to the UART

        jee_device_control(uart_dev, JEE_DEVICE_CTRL_UART_WRITE, (void *)&xUartData);
        if (JEE_EOK == ret) {
            data[xUartData.real_size] = '\0';
            vTaskDelay(pdMS_TO_TICKS(100));
            //LogInfo(TAG, "%s: Send str: %s\n", __func__, (char *) data);
            printf("%s: Send str: %s\n", __func__, (char *) data);
        } else {
            //LogError(TAG, "%s: Send error\n", __func__);
            printf("%s: Send error\n", __func__);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    int i = 0xff;

#ifdef configUSING_COMPONENTS_INIT
    // Onboard components initialization.
    //rt_components_board_init();
    // components initialization.
    rt_components_init();
#endif
    //jee_device_mutex_init();
    //jee_device_sem_init();
    //jee_hw_uart_init();
    LogInfo(TAG, "Recv str: %d\n", i);
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
