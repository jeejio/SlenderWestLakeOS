/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-04                  first version
 *
 */

#include "device.h"
#include "hal_serial.h"
#include "driver/uart.h"
#include "driver/drv_uart_iomodel.h"
#include "hal/uart_hal.h"
#include "sdkconfig.h"
#include "auto_init.h"
#include "soc/uart_periph.h"
#include "hal/uart_hal.h"
#include "esp_log.h"

#ifdef BSP_USING_UART0
#define UART0_NAME "uart0"
#define JEE_BSP_UART0_PORT    0
#define JEE_BSP_UART0_TX_PIN  21
#define JEE_BSP_UART0_RX_PIN  20
#define JEE_BSP_UART0_RTS_PIN (UART_PIN_NO_CHANGE)
#define JEE_BSP_UART0_CTS_PIN (UART_PIN_NO_CHANGE)
#endif

#ifdef BSP_USING_UART1
#define UART1_NAME "uart1"
#define JEE_BSP_UART1_PORT    1
#define JEE_BSP_UART1_TX_PIN  3
#define JEE_BSP_UART1_RX_PIN  2
#define JEE_BSP_UART1_RTS_PIN (UART_PIN_NO_CHANGE)
#define JEE_BSP_UART1_CTS_PIN (UART_PIN_NO_CHANGE)
#endif

#define UART_RX_BUF_SIZE      256
#define UART_TX_BUF_SIZE      0
#define UART_EVENT_QUEUE_SIZE 0

#define UART_EMPTY_THRESH_DEFAULT       (10)
#define UART_FULL_THRESH_DEFAULT        (120)
#define UART_TOUT_THRESH_DEFAULT        (10)
#define UART_CLKDIV_FRAG_BIT_WIDTH      (3)
#define UART_TX_IDLE_NUM_DEFAULT        (0)
#define UART_PATTERN_DET_QLEN_DEFAULT   (10)
#define UART_MIN_WAKEUP_THRESH          (UART_LL_MIN_WAKEUP_THRESH)

#ifdef BSP_USING_UART0
static struct jee_serial_device serial0;
#endif

#ifdef BSP_USING_UART0
static struct jee_serial_device serial1;
#endif

static __attribute__((unused)) const char *TAG = "DRV_UART";


static jee_err_t mcu_uart_configure(struct jee_serial_device *serial, struct serial_configure *cfg);

static int mcu_uart_putc(struct jee_serial_device *serial, char c);
static int mcu_uart_getc(struct jee_serial_device *serial);
static jee_size_t mcu_uart_transmit(struct jee_serial_device *serial, jee_uint8_t *buf, jee_size_t size, jee_uint32_t devFlag);

struct uart_plat_data {
    jee_uint8_t  *xBuf;
    jee_uint32_t bufSize;
    jee_int32_t  real_size;
};

/**
 * @brief This function will perform a configuration of the uart.
 *
 * @param serial is the pointer of serial device driver structure.
 *
 * @param serial_configure is the parameters of the uart device.
 *
 * @param arg is the argument of command.
 *
 * @return the result, -JEE_ENOSYS for failed.
 */
static jee_err_t mcu_uart_configure(struct jee_serial_device *serial, struct serial_configure *cfg)
{
    uart_port_t port = (uart_port_t)serial->parent.user_data;
    int intr_alloc_flags = 0;

    LogInfo(TAG, "%s", __func__);

    configASSERT(cfg != NULL);

    uart_config_t uart_config = {
        .baud_rate = cfg->baud_rate,
        .data_bits = cfg->data_bits - 5,
        .stop_bits = cfg->stop_bits,
        .source_clk = UART_SCLK_APB,
    };

    if (cfg->parity == PARITY_ODD) {
        uart_config.parity = UART_PARITY_ODD;
    }
    if (cfg->flowcontrol == JEE_SERIAL_FLOWCONTROL_CTSRTS) {
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
    }

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    ESP_ERROR_CHECK(uart_driver_install(port, UART_RX_BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, cfg->txPinNo, cfg->rxPinNo, cfg->rtsPinNo, cfg->ctsPinNo));
    return JEE_EOK;
}

static jee_err_t mcu_uart_control(struct jee_serial_device *serial, int cmd, void *arg)
{
    uart_port_t port = (uart_port_t)serial->parent.user_data;
    struct uart_plat_data *pUartData = (struct uart_plat_data *)arg;

    switch (cmd) {
    case JEE_DEVICE_CTRL_CLOSE:
        /* Uninstall the uart driver */
        uart_driver_delete(port);
        break;

    case JEE_DEVICE_CTRL_CLR_INT:
        /* Disable the UART Interrupt */
        uart_disable_rx_intr(port);
        uart_disable_tx_intr(port);
        break;

    case JEE_DEVICE_CTRL_SET_INT:
        /* Enable the UART Interrupt */
        uart_enable_rx_intr(port);
        uart_enable_tx_intr(port, 1, UART_EMPTY_THRESH_DEFAULT);
        break;

    case JEE_DEVICE_CTRL_UART_READ:
        pUartData->real_size = mcu_uart_transmit(serial, pUartData->xBuf,
                                              pUartData->bufSize, UART_READ);
        if (pUartData->real_size < 0) {
            return JEE_ERROR;
        }
        break;

    case JEE_DEVICE_CTRL_UART_WRITE:
        pUartData->real_size = mcu_uart_transmit(serial, pUartData->xBuf,
                                              pUartData->bufSize, UART_WRITE);
        if (pUartData->real_size <= 0) {
            return JEE_ERROR;
        }
        break;

    default:
        LogError(TAG, "%s: This command is unrecognized!\n", __func__);
        break;
    }

    return JEE_EOK;
}

static int mcu_uart_putc(struct jee_serial_device *serial, char c)
{
    return 0;
}

static int mcu_uart_getc(struct jee_serial_device *serial)
{
    return 1;
}

static jee_size_t mcu_uart_transmit(struct jee_serial_device *serial, jee_uint8_t *buf, jee_size_t size, jee_uint32_t devFlag)
{
    uart_port_t port = (uart_port_t)serial->parent.user_data;

    if (devFlag == UART_READ) {
        return uart_read_bytes(port, buf, size, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
    } else if (devFlag == UART_WRITE) {
        return uart_write_bytes(port, buf, size);
    }

    return 0;
}

static const struct jee_uart_ops _uart_ops =
{
    mcu_uart_configure,
    mcu_uart_control,
    mcu_uart_putc,
    mcu_uart_getc,
    mcu_uart_transmit,
};

int jee_hw_uart_init(void)
{
    static struct jee_serial_device *_serial;
#ifdef BSP_USING_UART0
    struct serial_configure xUart0Config = JEE_SERIAL_CONFIG_DEFAULT;
#endif
#ifdef BSP_USING_UART1
    struct serial_configure xUart1Config = JEE_SERIAL_CONFIG_DEFAULT;
#endif
    LogInfo(TAG, "%s\n", __func__);

#ifdef BSP_USING_UART0
    _serial  = &serial0;
    xUart0Config.rxPinNo = JEE_BSP_UART0_RX_PIN;
    xUart0Config.txPinNo = JEE_BSP_UART0_TX_PIN;
    xUart0Config.rtsPinNo = JEE_BSP_UART0_RTS_PIN;
    xUart0Config.ctsPinNo = JEE_BSP_UART0_CTS_PIN;
    _serial->config = xUart0Config;
    _serial->ops = &_uart_ops;

    jee_hw_serial_register(_serial, UART0_NAME, JEE_DEVICE_FLAG_RDWR | JEE_DEVICE_FLAG_INT_RX, (void *)JEE_BSP_UART0_PORT);
#endif
#ifdef BSP_USING_UART1
    _serial  = &serial1;
    xUart1Config.baud_rate= BAUD_RATE_9600;
    xUart1Config.rxPinNo = JEE_BSP_UART1_RX_PIN;
    xUart1Config.txPinNo = JEE_BSP_UART1_TX_PIN;
    xUart1Config.rtsPinNo = JEE_BSP_UART1_RTS_PIN;
    xUart1Config.ctsPinNo = JEE_BSP_UART1_CTS_PIN;
    _serial->config = xUart1Config;
    _serial->ops = &_uart_ops;

    jee_hw_serial_register(_serial, UART1_NAME, JEE_DEVICE_FLAG_RDWR | JEE_DEVICE_FLAG_INT_RX, (void *)JEE_BSP_UART1_PORT);
#endif

    return 0;
}
INIT_PREV_EXPORT(jee_hw_uart_init);


