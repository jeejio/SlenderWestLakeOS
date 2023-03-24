/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2023-02-07     xuyuhu         the first version
 */

#ifndef __TAL_SIX_AXIS_H__
#define __TAL_SIX_AXIS_H__

#include "string.h"
typedef struct
{
   float yawAngle; /* */
   float pithAngle;
   float rollAngle;
   float accelerationX;
   float accelerationY;
   float accelerationZ;
   float angularVelocityX;
   float angularVelocityY;
   float angularVelocityZ;

} SixAxisOnReceive_t;

typedef struct
{
   float accelerationX; // X轴加速度
   float accelerationY; // Y轴加速度
   float accelerationZ; // Z轴加速度
} SixAxisAccelerometer_t;

typedef struct
{
   float angularVelocityX; // X轴角速度
   float angularVelocityY; // Y轴角速度
   float angularVelocityZ; // Z轴角速度
} SixAxisGyroscope_t;

typedef struct
{
   float yawAngle;  // 偏航角度
   float pithAngle; // 俯仰角度
   float rollAngle; // 翻滚角度
} SixAxisEulerAngle_t;

/**
 * @brief       初始化六轴传感器                .
 *
 *   NOTE:      NULL
 */
void vTalSixAxisInit(void);

/**
 * @brief       获取陀螺仪数据.
 *
 *   NOTE:      数据包括：陀螺仪 X轴 Y轴 Z轴
 *
 * return   陀螺仪数据.
 *
 */
SixAxisGyroscope_t *vTalSixAxisGetGyroscope(void);

/**
 * @brief       获取欧拉角数据.
 *
 *   NOTE:      数据包括： 偏航角度 俯仰角度 翻滚角度
 *
 * return   欧拉角数据.
 *
 */
SixAxisEulerAngle_t *vTalSixAxisGetEulerAngle(void);

/**
 * @brief       获取加速度数据.
 *
 *   NOTE:      数据包括： 加速度 X轴 Y轴 Z轴
 *
 * return   加速度数据.
 *
 */
SixAxisAccelerometer_t *vTalSixAxisGetAccelerometer(void);
/*******以下接口与 【中科物栖小应用开发文档】 保持函数名称对应**********/

/**
 * @brief       获取陀螺仪偏航角度.
 *
 *   NOTE:      用四元素计算偏航角，偏航角是表示绕y轴旋转的角度
 *
 * return   陀螺仪偏航角度.
 *
 */
float lTalSixAxisGetYawAngle(void);

/**
 * @brief       获取陀螺仪俯仰角度.
 *
 *   NOTE:      用四元素计算俯仰角，俯仰角是表示绕x轴旋转的角度
 *
 * return   陀螺仪俯仰角度.
 *
 */
float lTalSixAxisGetPithAngle(void);

/**
 * @brief       获取陀螺仪翻滚角度.
 *
 *   NOTE:      用四元素计算翻滚脚，滚转脚是表示绕z轴旋转的角度
 *
 * return   陀螺仪翻滚角度.
 *
 */
float lTalSixAxisGetRollAngle(void);

/**
 * @brief       获取加速度X轴.
 *
 *   NOTE:      从传感器中直接读取加速度X轴的高低字节，得到raw_acc_xyz[0]，
 *              公式：
 *              ONE_G = 9.807f
 *             {
 *              加速度参数2g       g_imu.ssvt_a =  (1 << 14);
 *              加速度参数4g       g_imu.ssvt_a =  (1 << 13);
 *              加速度参数8g       g_imu.ssvt_a =  (1 << 12);
 *              加速度参数16g      g_imu.ssvt_a =  (1 << 11);
 *              }
 *              acc[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
 *
 * @return      X轴值-acc[0]
 */
float lTalSixAxisGetAccelerationX(void);

/**
 * @brief       获取加速度Y轴.
 *
 *   NOTE:      从传感器中直接读取加速度Y轴的高低字节，得到raw_acc_xyz[1]，
 *              公式：
 *              ONE_G = 9.807f
 *             {
 *              加速度参数2g       g_imu.ssvt_a =  (1 << 14);
 *              加速度参数4g       g_imu.ssvt_a =  (1 << 13);
 *              加速度参数8g       g_imu.ssvt_a =  (1 << 12);
 *              加速度参数16g      g_imu.ssvt_a =  (1 << 11);
 *              }
 *              acc[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
 *
 * @return      Y轴值-acc[1]
 */
float lTalSixAxisGetAccelerationY(void);

/**
 * @brief       获取加速度Z轴.
 *
 *   NOTE:      从传感器中直接读取加速度Z轴的高低字节，得到raw_acc_xyz[2]，
 *              公式：
 *              ONE_G = 9.807f
 *             {
 *              加速度参数2g       g_imu.ssvt_a =  (1 << 14);
 *              加速度参数4g       g_imu.ssvt_a =  (1 << 13);
 *              加速度参数8g       g_imu.ssvt_a =  (1 << 12);
 *              加速度参数16g      g_imu.ssvt_a =  (1 << 11);
 *              }
 *              acc[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;
 *
 * @return      Z轴值-acc[2]
 */
float lTalSixAxisGetAccelerationZ(void);

/**
 * @brief       获取陀螺仪X轴.
 *
 *   NOTE:      从传感器中直接读取陀螺仪X轴的高低字节，得到raw_gyro_xyz[0]，
 *              公式：
 *             {
 *              陀螺仪参数16dps         g_imu.ssvt_g =  2048;
 *              陀螺仪参数32dps         g_imu.ssvt_g =  1024;
 *              陀螺仪参数64dps         g_imu.ssvt_g =  512;
 *              陀螺仪参数128dps        g_imu.ssvt_g =  256;
 *              陀螺仪参数256dps        g_imu.ssvt_g =  128;
 *              陀螺仪参数512dps        g_imu.ssvt_g =  64;
 *              陀螺仪参数1024dps       g_imu.ssvt_g =  32;
 *              陀螺仪参数2048dps       g_imu.ssvt_g =  16;
 *              }
 *              gyro[0] = (float)(raw_gyro_xyz[0] * 0.01745f) / g_imu.ssvt_g;
 *
 * @return      X轴值-gyro[0]
 */
float lTalSixAxisGetAngularVelocityX(void);

/**
 * @brief       获取陀螺仪Y轴.
 *
 *   NOTE:      从传感器中直接读取陀螺仪Y轴的高低字节，得到raw_gyro_xyz[1]，
 *              公式：
 *             {
 *              陀螺仪参数16dps         g_imu.ssvt_g =  2048;
 *              陀螺仪参数32dps         g_imu.ssvt_g =  1024;
 *              陀螺仪参数64dps         g_imu.ssvt_g =  512;
 *              陀螺仪参数128dps        g_imu.ssvt_g =  256;
 *              陀螺仪参数256dps        g_imu.ssvt_g =  128;
 *              陀螺仪参数512dps        g_imu.ssvt_g =  64;
 *              陀螺仪参数1024dps       g_imu.ssvt_g =  32;
 *              陀螺仪参数2048dps       g_imu.ssvt_g =  16;
 *              }
 *              gyro[1] = (float)(raw_gyro_xyz[1] * 0.01745f) / g_imu.ssvt_g;
 *
 * @return      Y轴值-gyro[1]
 */
float lTalSixAxisGetAngularVelocityY(void);

/**
 * @brief       获取陀螺仪Z轴.
 *
 *   NOTE:      从传感器中直接读取陀螺仪Z轴的高低字节，得到raw_gyro_xyz[2]，
 *              公式：
 *             {
 *              陀螺仪参数16dps         g_imu.ssvt_g =  2048;
 *              陀螺仪参数32dps         g_imu.ssvt_g =  1024;
 *              陀螺仪参数64dps         g_imu.ssvt_g =  512;
 *              陀螺仪参数128dps        g_imu.ssvt_g =  256;
 *              陀螺仪参数256dps        g_imu.ssvt_g =  128;
 *              陀螺仪参数512dps        g_imu.ssvt_g =  64;
 *              陀螺仪参数1024dps       g_imu.ssvt_g =  32;
 *              陀螺仪参数2048dps       g_imu.ssvt_g =  16;
 *              }
 *              gyro[2] = (float)(raw_gyro_xyz[2] * 0.01745f) / g_imu.ssvt_g;
 *
 * @return      Z轴值-gyro[2]
 */
float lTalSixAxisGetAngularVelocityZ(void);

/**
 * @brief       上报所有传感器数据.
 *
 *   NOTE:      数据包括：欧拉角，加速度，陀螺仪
 *
 * @return      指向传感器数据的指针
 *
 *   NOTE:
 *           typedef struct
 *             {
 *                 float yawAngle;           //偏航角度
 *                 float pithAngle;          //俯仰角度
 *                 float rollAngle;          //翻滚角度
 *                 float accelerationX;      //X轴加速度
 *                 float accelerationY;      //Y轴加速度
 *                 float accelerationZ;      //Z轴加速度
 *                 float angularVelocityX;   //X轴角速度
 *                 float angularVelocityY;   //Y轴角速度
 *                 float angularVelocityZ;   //Z轴角速度
 *             } SixAxisOnReceive_t;
 *
 */
SixAxisOnReceive_t *lTalSixAxisOnReceive(void);
#endif