#ifndef __PAJ7620U2_H
#define __PAJ7620U2_H

#include "stdint.h"
#include "stdbool.h"

typedef enum
{
  MOVING_UP = 1,
  MOVING_DOWN,
  MOVING_LEFT,
  MOVING_RIGHT,
  MOVING_FRONT,
  MOVING_BACK,
} moving_dir_t;

typedef enum
{
  CLOCKWISE = 7,
  COUNTERCLOCKWISE,
} rotating_dir_t;

/**
 * @brief       手势传感器初始化
 *
 *   NOTE:      IIC初始化，GPIO初始化，配置传感器参数
 *
 */
void gesture_sensor_init(void);

/**
  * @brief       获取手势传感器检测见过
  *
  * @retuen      手势检测结果
  *               NoGesture            = 0,
  *               UpGesture            = 1,
  *               DownGesture          = 2,
  *               LeftGesture          = 3,
  *               RightGesture         = 4,
  *               ForwardGesture       = 5,
  *               BackwardGesture      = 6,
  *               ClockwiseGesture     = 7,
  *               AntiClockwiseGesture = 8,
  *               WaveGesture          = 9,
  *
  */
uint8_t get_getsture_status(void);

#endif
