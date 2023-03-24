#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_six_axis.h"

static void __vImuGyroscopeAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    SixAxisOnReceive_t *OnReceive = NULL;
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {

            printf("test_t \n");
            OnReceive = lTalSixAxisOnReceive();
            printf("3acc-x:%d,acc-y:%d,acc-z:%d ", (short)OnReceive->accelerationX, (short)OnReceive->accelerationY, (short)OnReceive->accelerationZ);
            printf("3gyr-x:%d,gyr-y:%d,gyr-z:%d ", (short)OnReceive->angularVelocityX, (short)OnReceive->angularVelocityY, (short)OnReceive->angularVelocityZ);
            printf("3pith:%d,roll:%d,yaw:%d\n ", (short)OnReceive->pithAngle, (short)OnReceive->rollAngle, (short)OnReceive->yawAngle);
            char *obuff = calloc(512, 1);
            sprintf(obuff, "{\"code\":%d,\"yawAngle\":%f,\"pithAngle\":%f,\"rollAngle\":%f,\"accelerationX\":%f,\"accelerationY\":%f,\"accelerationZ\":%f,\"angularVelocityX\":%f,\"angularVelocityY\":%f,\"angularVelocityZ\":%f,\"msg\":\"\"}",
                    JERRNO_RPC_SUCCESS, OnReceive->yawAngle, OnReceive->pithAngle, OnReceive->rollAngle, OnReceive->accelerationX, OnReceive->accelerationY, OnReceive->accelerationZ, OnReceive->angularVelocityX, OnReceive->angularVelocityY, OnReceive->angularVelocityZ);
            JeeRPC_SendEvent(conn, NULL, obuff);
            free(obuff);
            obuff = NULL;
            printf("test--------------------\n");
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

int32_t IMUSensor_Object_Init(void *args)
{
    printf("six axis init\r\n");
    vTalSixAxisInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vImuGyroscopeAppTask, /* The function that implements the task. */
                "ImuGyroscopeAppTask",  /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,     /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,           /* The task parameter, not used in this case. */
                uxTaskPriorityReq,      /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);                  /* The task handle is not used. */
    return 0;
}

// 获取陀螺仪偏航角度
int32_t lRotaryEncoderGetYawAngleProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetYawAngle();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪俯仰角度
int32_t lRotaryEncoderGetPithAngleProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetPithAngle();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪翻滚角度。
int32_t lRotaryEncoderGetRollAngleProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetRollAngle();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪X轴加速度
int32_t lRotaryEncoderGetAccelerationXProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetAccelerationX();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪Y轴加速度（单位m/s2）
int32_t lRotaryEncoderGetAccelerationYProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetAccelerationY();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪Z轴加速度（单位m/s2）
int32_t lRotaryEncoderGetAccelerationZProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetAccelerationZ();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪X轴角速度
int32_t lRotaryEncoderGetAngularVelocityXProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetAngularVelocityX();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪Y轴角速度
int32_t lRotaryEncoderGetAngularVelocityYProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetAngularVelocityY();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}
// 获取陀螺仪Z轴角速度
int32_t lRotaryEncoderGetAngularVelocityZProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float ret = lTalSixAxisGetAngularVelocityZ();
    sprintf(result, "{\"code\":%d,\"value\":%f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}

/************IMU_SENSOR_API*************/
JeeDevApi_t ImuSensor_API[] =
    {
        REGISTER_API("getYawAngle", lRotaryEncoderGetYawAngleProcess, NONE_OBJ),
        REGISTER_API("getPithAngle", lRotaryEncoderGetPithAngleProcess, NONE_OBJ),
        REGISTER_API("getRollAngle", lRotaryEncoderGetRollAngleProcess, NONE_OBJ),
        REGISTER_API("getAccelerationX", lRotaryEncoderGetAccelerationXProcess, NONE_OBJ),
        REGISTER_API("getAccelerationY", lRotaryEncoderGetAccelerationYProcess, NONE_OBJ),
        REGISTER_API("getAccelerationZ", lRotaryEncoderGetAccelerationZProcess, NONE_OBJ),
        REGISTER_API("getAngularVelocityX", lRotaryEncoderGetAngularVelocityXProcess, NONE_OBJ),
        REGISTER_API("getAngularVelocityY", lRotaryEncoderGetAngularVelocityYProcess, NONE_OBJ),
        REGISTER_API("getAngularVelocityZ", lRotaryEncoderGetAngularVelocityZProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

DevObject_t ImuSensorObject =
    {
        .lDevObjectInit = IMUSensor_Object_Init,
        .devApi = ImuSensor_API,
};
