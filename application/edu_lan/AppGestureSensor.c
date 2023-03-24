#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_getsture.h"

////////////////////////////////////////////start of GestureSensor :手势识别

// 返回值:
//   *              UpGesture            = 1,   上
//   *              DownGesture          = 2,   下
//   *              LeftGesture          = 3,   左
//   *              RightGesture         = 4,   右
//   *              ForwardGesture       = 5,   前
//   *              BackwardGesture      = 6,   后
//   *              NoGesture            = 0,   不属于以上六种
int32_t lGetMovingDirectionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    char *pcGesture[7] = {"NoGesture", "Up", "Down", "Left", "Right", "Front", "Rear"};
    GestureList ret;
    int counter = 0;
    do
    {
        ret = xTalGetstureGetMovingDirection();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (counter++ > 30)
        {
            break;
        }
    } while (0 == ret);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)((ret >= 1 && ret <= 6) ? pcGesture[ret] : "NoGesture"), "");
    return 0;
}

//
//  返回值:
//  *              ClockwiseGesture      = 7,   顺时针
//  *              AntiClockwiseGesture  = 8,   逆时针
//  *              NoGesture             = 0,   不属于以上2种
//
int32_t lGestureGetRotationDirectionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    char *pcGesture[9] = {"NoGesture", "Up", "Down", "Left", "Right", "Front", "Rear", "Clockwise", "Counterclockwise"};
    GestureList ret;
    int counter = 0;
    do
    {
        ret = xTalGetstureGetRotationDirection();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (counter++ > 30)
        {
            break;
        }
    } while (0 == ret);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)((ret >= 7 && ret <= 8) ? pcGesture[ret] : "NoGesture"), "");
    return 0;
}

//
//  返回值:
//  *              WaveGesture = 9,   摇摆
//  *              NoGesture   = 0,   非摇摆
//
int32_t lGetSwingStatusProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    GestureList ret;
    int counter = 0;
    do
    {
        ret = xTalGetstureGetSwingStatus();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (counter++ > 30)
        {
            break;
        }
    } while (0 == ret);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)((ret == 9) ? "True" : "False"), "");
    return 0;
}

////////////////////////////////////////////end of GestureSensor :手势识别

/************GESTURE_SENSOR 手势*************/
JeeDevApi_t GestureSensor_API[] =
    {
        REGISTER_API("getMovingDirection", lGetMovingDirectionProcess, NONE_OBJ),
        REGISTER_API("getRotationDirection", lGestureGetRotationDirectionProcess, NONE_OBJ),
        REGISTER_API("getSwingStatus", lGetSwingStatusProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vGestureSensorAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    GestureList retMoving, retRotation, retSwing;
    char *pGesture[9] = {"NoGesture", "Up", "Down", "Left", "Right", "Front", "Rear", "Clockwise", "Counterclockwise"};
    char arrayBuff[512] = {0};
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {

            retMoving = xTalGetstureGetMovingDirection();
            retRotation = xTalGetstureGetRotationDirection();
            retSwing = xTalGetstureGetSwingStatus();
            if ((retSwing != 0) || (retRotation != 0) || retMoving != 0)
            {
                sprintf(arrayBuff, "{\"code\":%d,\"movingDirection\":\"%s\",\"rotationDirection\":\"%s\",\"swingStatus\":\"%s\",\"message\":\"%s\"}",
                        JERRNO_RPC_SUCCESS, pGesture[retMoving], retRotation ? pGesture[retRotation] : "NoGesture", (retSwing == 9) ? "True" : "False", "");
                JeeRPC_SendEvent(conn, NULL, arrayBuff);
                memset(arrayBuff, 0, 512);
            }
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS); // 周期上报，周期为3s
    }
}

int32_t GestureSensor_Object_Init(void *args)
{
    printf("GestureSensor_Object_Init\r\n");
    lTalGetstureInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vGestureSensorAppTask,   /* The function that implements the task. */
                "__vGestureSensorAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,        /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,              /* The task parameter, not used in this case. */
                uxTaskPriorityReq,         /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

DevObject_t GestureSensorObject =
    {
        .lDevObjectInit = GestureSensor_Object_Init,
        .devApi = GestureSensor_API,
};
