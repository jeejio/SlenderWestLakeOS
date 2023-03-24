#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_joystick.h"

////////////////////////////////////////////start of Joystick :游戏操纵杆

//
//
//
int32_t lJoystickGetStatusProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int ret = 0;
    ret = lTalJoystickGetStatus();

    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return 0;
}

//
// int 型，取值范围为坐标轴【-100,100】
//
int32_t lJoystickGetPositionXProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int ret = 0;
    ret = lTalJoystickGetPositioninformationX();

    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}

//
// int 型，取值范围为坐标轴【-100,100】
//
int32_t lJoystickGetPositionYProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int ret = 0;
    ret = lTalJoystickGetPositioninformationY();

    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}

////////////////////////////////////////////end of Joystick :游戏操纵杆

/************JOYSTICK_API操纵杆*************/
JeeDevApi_t Joystick_API[] =
    {
        REGISTER_API("getStatus", lJoystickGetStatusProcess, NONE_OBJ),
        REGISTER_API("getPositioninformationX", lJoystickGetPositionXProcess, NONE_OBJ),
        REGISTER_API("getPositioninformationY", lJoystickGetPositionYProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vJoystickAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    TalJoystickPositionInfo_t *pstInfo = NULL;
    char arrayBuff[128] = {0};
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            pstInfo = lTalJoystickPositionReceive();
            if (pstInfo == NULL) // 返回值不合法
            {
                continue;
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            printf("positionX:%d, positionY:%d\n", pstInfo->getPositioninformationX, pstInfo->getPositioninformationY);
            sprintf(arrayBuff, "{\"code\":%d,\"positionX\":%d,\"positionY\":%d,\"message\":\"%s\",\"firmwareCode\":%d,\"modeType\":\"%s\"}",
                    JERRNO_RPC_SUCCESS, pstInfo->getPositioninformationX, pstInfo->getPositioninformationY, "", devType, "positionReceive");
            JeeRPC_SendEvent(conn, NULL, arrayBuff);
            memset(arrayBuff, 0, 128);
            pstInfo = NULL;
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // 周期上报，周期为100ms
    }
}

static void __vJoystickModeTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    int mode = 0, oldMode = 0;
    char *modeStr = "NULL";
    char arrayBuff[128] = {0};
    int connectFlag = 0;

    while (1)
    {

        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            mode = lTalJoystickModeReceive();
            printf("joystick mode :%d\r\n", mode);
            if (mode <= 0)
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }
            else if (1 == mode)
            {
                modeStr = "Click";
            }
            else if (2 == mode)
            {
                modeStr = "DoubleClick";
            }
            else if (3 == mode)
            {
                modeStr = "LongPress";
            }
            else if (4 == mode)
            {
                modeStr = "CalibrationStrat";
            }
            else if (5 == mode)
            {
                modeStr = "CalibrationEnd";
            }
            oldMode = mode;

            sprintf(arrayBuff, "{\"code\":%d,\"mode\":\"%s\",\"message\":\"%s\",\"firmwareCode\":%d,\"modeType\":\"%s\"}",
                    JERRNO_RPC_SUCCESS, modeStr, "", devType, "modeReceive");
            JeeRPC_SendEvent(conn, NULL, arrayBuff);
            memset(arrayBuff, 0, 128);
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // 周期上报，周期为3s
    }
}

int32_t Joystick_Object_Init(void *args)
{
    printf("Joystick_Object_Init\r\n");
    vTalJoystickInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vJoystickAppTask,   /* The function that implements the task. */
                "__vJoystickAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,   /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,         /* The task parameter, not used in this case. */
                uxTaskPriorityReq,    /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    xTaskCreate(__vJoystickModeTask,   /* The function that implements the task. */
                "__vJoystickModeTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,    /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,          /* The task parameter, not used in this case. */
                uxTaskPriorityReq,     /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

DevObject_t JoystickObject =
    {
        .lDevObjectInit = Joystick_Object_Init,
        .devApi = Joystick_API,
};