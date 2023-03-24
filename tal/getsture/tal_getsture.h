#ifndef __TAL_GETSTURE_H__
#define __TAL_GETSTURE_H__

typedef enum
{
    NoGesture = 0,
    UpGesture = 1,
    DownGesture = 2,
    LeftGesture = 3,
    RightGesture = 4,
    ForwardGesture = 5,
    BackwardGesture = 6,
    ClockwiseGesture = 7,
    AntiClockwiseGesture = 8,
    WaveGesture = 9,
} GestureList;

/**
 * @brief       手势传感器初始化
 *
 * NOTE:        TAL层调用驱动框架查找设备，并对手势传感器硬件进行初始化
 *
 */
jee_int32_t lTalGetstureInit(void);

/**
 * @brief       获取手势传感器检测结果
 *
 * NOTE:        调用后结果自动清零
 *
 * @return      手势类型（GestureList）
 *
 */
GestureList ucTalGetGetstureStatus(void);

/**
 * @brief       获取手势的移动方向.
 *
 * @return      方向值 （GestureList ）
 *
 *   NOTE:
 *              UpGesture            = 1,   上
 *              DownGesture          = 2,   下
 *              LeftGesture          = 3,   左
 *              RightGesture         = 4,   右
 *              ForwardGesture       = 5,   前
 *              BackwardGesture      = 6,   后
 *              NoGesture            = 0,   不属于以上六种
 *
 */
GestureList xTalGetstureGetMovingDirection(void);

/**
 * @brief       获取手势的旋转方向.
 *
 * @return      旋转方向值 （GestureList ）
 *
 *   NOTE:
 *              ClockwiseGesture      = 1,   顺时针
 *              AntiClockwiseGesture  = 2,   逆时针
 *              NoGesture             = 0,   不属于以上2种
 *
 */
GestureList xTalGetstureGetRotationDirection(void);

/**
 * @brief       获取手势的摇摆状态.
 *
 * @return      摇摆手势值 （GestureList ）
 *
 *   NOTE:
 *              WaveGesture = 9,,  摇摆
 *              NoGesture   = 0,   非摇摆
 *
 */
GestureList xTalGetstureGetSwingStatus(void);

#endif
