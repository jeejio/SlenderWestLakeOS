/*
 * Copyright (c) 2022, Jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author   Notes
 * 2022-12-02     Monk     First version
 */

#ifndef __HAL_SERIAL_H__
#define __HAL_SERIAL_H__

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal_ringbuffer.h"
#include "jeedef.h"


#define BAUD_RATE_2400                  2400
#define BAUD_RATE_4800                  4800
#define BAUD_RATE_9600                  9600
#define BAUD_RATE_19200                 19200
#define BAUD_RATE_38400                 38400
#define BAUD_RATE_57600                 57600
#define BAUD_RATE_115200                115200
#define BAUD_RATE_230400                230400
#define BAUD_RATE_460800                460800
#define BAUD_RATE_921600                921600
#define BAUD_RATE_2000000               2000000
#define BAUD_RATE_2500000               2500000
#define BAUD_RATE_3000000               3000000

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     8
#define DATA_BITS_9                     9

#define STOP_BITS_1                     1
#define STOP_BITS_1_5                   2
#define STOP_BITS_2                     3

#ifdef _WIN32
#include <windows.h>
#else
#define PARITY_NONE                     0
#define PARITY_ODD                      1
#define PARITY_EVEN                     2
#endif

#define BIT_ORDER_LSB                   0
#define BIT_ORDER_MSB                   1

#define NRZ_NORMAL                      0       /* Non Return to Zero : normal mode */
#define NRZ_INVERTED                    1       /* Non Return to Zero : inverted mode */

#ifdef configUSE_JEE_UART_MECHANISM
#define JEE_DEVICE_FLAG_RX_BLOCKING      0x1000
#define JEE_DEVICE_FLAG_RX_NON_BLOCKING  0x2000

#define JEE_DEVICE_FLAG_TX_BLOCKING      0x4000
#define JEE_DEVICE_FLAG_TX_NON_BLOCKING  0x8000

#define JEE_SERIAL_RX_BLOCKING           JEE_DEVICE_FLAG_RX_BLOCKING
#define JEE_SERIAL_RX_NON_BLOCKING       JEE_DEVICE_FLAG_RX_NON_BLOCKING
#define JEE_SERIAL_TX_BLOCKING           JEE_DEVICE_FLAG_TX_BLOCKING
#define JEE_SERIAL_TX_NON_BLOCKING       JEE_DEVICE_FLAG_TX_NON_BLOCKING

#define JEE_DEVICE_CHECK_OPTMODE         0x20

#define JEE_SERIAL_EVENT_RX_IND          0x01    /* Rx indication */
#define JEE_SERIAL_EVENT_TX_DONE         0x02    /* Tx complete   */
#define JEE_SERIAL_EVENT_RX_DMADONE      0x03    /* Rx DMA transfer done */
#define JEE_SERIAL_EVENT_TX_DMADONE      0x04    /* Tx DMA transfer done */
#define JEE_SERIAL_EVENT_RX_TIMEOUT      0x05    /* Rx timeout    */
#else /* configUSE_JEE_UART_MECHANISM */
#define JEE_SERIAL_DEV_FLAG_OPENED       0x1000
#endif /* configUSE_JEE_UART_MECHANISM */

#define JEE_SERIAL_ERR_OVERRUN           0x01
#define JEE_SERIAL_ERR_FRAMING           0x02
#define JEE_SERIAL_ERR_PARITY            0x03

#define JEE_SERIAL_TX_DATAQUEUE_SIZE     2048
#define JEE_SERIAL_TX_DATAQUEUE_LWM      30

#define JEE_SERIAL_RX_MINBUFSZ 64
#define JEE_SERIAL_TX_MINBUFSZ 64

#define JEE_SERIAL_TX_BLOCKING_BUFFER    1
#define JEE_SERIAL_TX_BLOCKING_NO_BUFFER 0

#define JEE_SERIAL_FLOWCONTROL_CTSRTS    1
#define JEE_SERIAL_FLOWCONTROL_NONE      0

/* Default config for serial_configure structure */
#ifdef configUSE_JEE_UART_MECHANISM
#define JEE_SERIAL_CONFIG_DEFAULT                      \
{                                                      \
    BAUD_RATE_115200,            /* 115200 bits/s */   \
    DATA_BITS_8,                 /* 8 databits */      \
    STOP_BITS_1,                 /* 1 stopbit */       \
    PARITY_NONE,                 /* No parity  */      \
    BIT_ORDER_LSB,               /* LSB first sent */  \
    NRZ_NORMAL,                  /* Normal mode */     \
    JEE_SERIAL_RX_MINBUFSZ,      /* rxBuf size */      \
    JEE_SERIAL_TX_MINBUFSZ,      /* txBuf size */      \
    JEE_SERIAL_FLOWCONTROL_NONE, /* Off flowcontrol */ \
    0                                                 \
}
#else
#define JEE_SERIAL_CONFIG_DEFAULT                      \
{                                                      \
    BAUD_RATE_115200,            /* 115200 bits/s */   \
    DATA_BITS_8,                 /* 8 databits */      \
    STOP_BITS_1,                 /* 1 stopbit */       \
    PARITY_NONE,                 /* No parity  */      \
    BIT_ORDER_LSB,               /* LSB first sent */  \
    NRZ_NORMAL,                  /* Normal mode */     \
    JEE_SERIAL_FLOWCONTROL_NONE, /* Off flowcontrol */ \
    0,                                                 \
    -1,                          /* Rx pin number */   \
    -1,                          /* Tx pin number */   \
    -1,                          /* RTS pin number */  \
    -1                           /* cts pin number */  \
}
#endif

struct serial_configure
{
    jee_uint32_t baud_rate;

    jee_uint32_t data_bits               :4;
    jee_uint32_t stop_bits               :2;
    jee_uint32_t parity                  :2;
    jee_uint32_t bit_order               :1;
    jee_uint32_t invert                  :1;
#ifdef configUSE_JEE_UART_MECHANISM
    jee_uint32_t rx_bufsz                :16;
    jee_uint32_t tx_bufsz                :16;
#endif
    jee_uint32_t flowcontrol             :1;
    jee_uint32_t reserved                :5;

    jee_int32_t rxPinNo                  :8;
    jee_int32_t txPinNo                  :8;
    jee_int32_t rtsPinNo                 :8;
    jee_int32_t ctsPinNo                 :8;
};

#ifdef configUSE_JEE_UART_MECHANISM
/*
 * Serial Receive FIFO mode
 */
struct jee_serial_rx_fifo
{
    struct jee_ringbuffer rb;

    SemaphoreHandle_t rxSem;

    jee_uint16_t rx_cpt_index;

    /* software fifo */
    jee_uint8_t buffer[];
};

/*
 * Serial Transmit FIFO mode
 */
struct jee_serial_tx_fifo
{
    struct jee_ringbuffer rb;

    jee_size_t put_size;

    jee_bool_t activated;

    SemaphoreHandle_t txSem;

    /* software fifo */
    jee_uint8_t buffer[];
};
#endif

struct jee_serial_device
{
    struct jee_device          parent;

    const struct jee_uart_ops *ops;
    struct serial_configure   config;

#ifdef configUSE_JEE_UART_MECHANISM
    void *serial_rx;
    void *serial_tx;
#endif
};

/**
 * uart operators
 */
struct jee_uart_ops
{
    jee_err_t (*configure)(struct jee_serial_device       *serial,
                          struct serial_configure       *cfg);

    jee_err_t (*control)(struct jee_serial_device         *serial,
                                            int          cmd,
                                            void        *arg);

    int (*putc)(struct jee_serial_device *serial, char c);
    int (*getc)(struct jee_serial_device *serial);

    jee_size_t (*transmit)(struct jee_serial_device       *serial,
                                 jee_uint8_t             *buf,
                                 jee_size_t               size,
                                 jee_uint32_t             tx_flag);
};

#ifdef configUSE_JEE_UART_MECHANISM
void jee_hw_serial_isr(struct jee_serial_device *serial, int event);
#endif

jee_err_t jee_hw_serial_register(struct jee_serial_device      *serial,
                               const  char                  *name,
                                      jee_uint32_t            flag,
                                      void                  *data);

#endif
