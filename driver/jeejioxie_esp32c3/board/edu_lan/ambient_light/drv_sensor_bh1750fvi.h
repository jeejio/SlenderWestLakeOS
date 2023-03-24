/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-12-13     zhengqian      the first version
 */

#ifndef __JEE_SENSOR_BH1750FVI_H__
#define __JEE_SENSOR_BH1750FVI_H__

#include "hal_sensor.h" //sensor IO模型驱动框架头文件

#define WATER_PUMP_SET_CMD  (JEE_SENSOR_CTRL_USER_CMD_START + 1)    //水泵设置占空比命令

int jeeHwSensorLb1427bInit();

#endif