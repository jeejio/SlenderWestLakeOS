#ifndef _H_EDU_LAN_APP_
#define _H_EDU_LAN_APP_ (1)

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "JeeErrno.h"
#include "JeeRPC.h"
#include "JeeMQTTConnection.h"
#include "JeeMQTTTransaction.h"
#include "cJSON.h"
#include "freertos/device.h"
#include "JeeKeyManager.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define CHECK_API_OBJS(obj, result, resultLen)            \
    {                                                     \
        if (!obj || !result || !resultLen)                \
        {                                                 \
            printf("[%s] args error!\r\n", __FUNCTION__); \
            return -1;                                    \
        }                                                 \
    }

#define CHECK_API_ARGS(result, resultLen)                 \
    {                                                     \
        if (!result || !resultLen)                        \
        {                                                 \
            printf("[%s] args error!\r\n", __FUNCTION__); \
            return -1;                                    \
        }                                                 \
    }

    typedef enum devType_en
    {
        SENSOR_MIN_ID = 1,
        BUTTON_PANEL_ID = 1,
        ENV_SENSOR_ID,
        CODE_SCANNER_ID,
        ENCODER_ID,
        GESTURE_SENSOR_ID,
        HALL_SENSOR_ID,
        IMU_SENSOR_ID,
        JOYSTICK_ID,
        LCD_ID,
        LED_ARRAY_ID,
        MOTOR_ID,
        PIR_SENSOR_ID,
        RAIN_SENSOR_ID,
        RFID_ID,
        STEERING_GEAR_ID,
        WATER_PUMP_ID,
        BLOOD_OXYGEN_ID,
        SENSOR_MAX_ID,
    } DevType_t;

#define ISEXISTS 2
#define PRECIPITATIONLEVEL 4

    extern DevType_t devType;
    extern int64_t org_device_id;

    extern int32_t lGetDeviceType(void);
    extern int lSetDeviceType(int type);

    extern void vRegisterDeviceType(void);

    typedef int32_t (*lDevApiProcess_t)(void *args, char *result, int32_t resultLen);

    typedef int32_t (*lDevObjectInit_t)(void *args);
    typedef struct devObject_st
    {
        lDevObjectInit_t lDevObjectInit; // 设备初始化函数，包括传感器初始化和上报任务onRecive初始化
        PJeeDevApi_t devApi;             // RPC 链表
    } DevObject_t;

    // 所有设备的集合
    extern DevObject_t *DevObjectGroup[];

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _H_EDU_LAN_APP_
