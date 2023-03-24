/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-12-13     zhengqian      the first version
 */

#ifndef __JEE_SENSOR_WS2835_H__
#define __JEE_SENSOR_WS2835_H__

#include "hal_sensor.h" //sensor IO模型驱动框架头文件

#define WS2835_PIXEL_SET_CMD    (JEE_SENSOR_CTRL_USER_CMD_START + 1)
#define WS2835_REFRESH_SET_CMD  (JEE_SENSOR_CTRL_USER_CMD_START + 2)
#define WS2835_CLEAR_SET_CMD    (JEE_SENSOR_CTRL_USER_CMD_START + 3)
#define WS2835_DEL_SET_CMD      (JEE_SENSOR_CTRL_USER_CMD_START + 4)

int jeeHwSensorWs2835Init();

#endif