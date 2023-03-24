/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-12-07                    the first version
*/

#ifndef __DRV_UART_IOMODEL_H__
#define __DRV_UART_IOMODEL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define JEE_DEVICE_CTRL_UART_READ  0xF1
#define JEE_DEVICE_CTRL_UART_WRITE 0xF2

#define UART_READ    0x1
#define UART_WRITE   0x2

#define UART_READ_TIMEOUT_MS  20

#define BSP_USING_UART0
#define BSP_USING_UART1

int jee_hw_uart_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_IOMODEL_H__ */
