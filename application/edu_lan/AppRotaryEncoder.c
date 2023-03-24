#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_rotary_encoder.h"

/************BUTTON_PANEL_API  END*************/
int32_t lRotaryEncoderGetRotationDirectionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    bool ret = cTalRotaryEncoderGetRotationDirection();

    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "Clockwise" : "Counterclockwise"), "");

    return (int32_t)ret;
}
int32_t lRotaryEncoderGetPositionProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = lTalRotaryEncoderGetPosition();

    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return ret;
}
// 按键状态（ 按下：1、抬起：0）
int32_t lRotaryEncoderGetIsPressedProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = cTalRotaryEncoderGetIsPressed();

    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return ret;
}

/************旋转编码*************/
JeeDevApi_t RotaryEncoder_API[] =
    {
        REGISTER_API("getRotationDirection", lRotaryEncoderGetRotationDirectionProcess, NONE_OBJ),
        REGISTER_API("getPosition", lRotaryEncoderGetPositionProcess, NONE_OBJ),
        REGISTER_API("getIsPressed", lRotaryEncoderGetIsPressedProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vRotaryEncoderAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    char isPressed[8] = {0};
    char rotationDirection[24] = {0};
    RotaryEncoderStatus_t *GetStatus;
    int connectFlag = 0;
    int oldPressValue = 0, oldRotationDirection = 0, oldRelativePos = 0;
    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            GetStatus = xTalRotaryEncoderOnReceive();
            printf("GetStatus->IsPressed:%d,GetStatus->RelativePos:%d,GetStatus->RotationDirection:%d \n", GetStatus->IsPressed, GetStatus->RelativePos, GetStatus->RotationDirection);
            if (1 == GetStatus->IsPressed)
                strcpy(isPressed, "True");
            else
                strcpy(isPressed, "False");
            if (1 == GetStatus->RotationDirection)
                strcpy(rotationDirection, "Clockwise");
            else
                strcpy(rotationDirection, "CounterClockwise");

            if (GetStatus->IsPressed == oldPressValue && GetStatus->RotationDirection == oldRotationDirection && GetStatus->RelativePos == oldRelativePos)
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }
            else
            {
                oldPressValue = GetStatus->IsPressed;
                oldRotationDirection = GetStatus->RotationDirection;
                oldRelativePos = GetStatus->RelativePos;

                char *obuff = malloc(128);
                memset(obuff, 0, 128);
                sprintf(obuff, "{\"code\":%d,\"isPressed\":\"%s\",\"rotationDirection\":\"%s\",\"position\":%d,\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isPressed, rotationDirection, GetStatus->RelativePos);
                JeeRPC_SendEvent(conn, NULL, obuff);
                free(obuff);
                obuff = NULL;

                printf("test--------------------\n");
            }
        }
        else
        {
            printf("mqtt dis connect\r\n");
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
    }
}

int32_t RotaryEncoder_Object_Init(void *args)
{
    printf("rotary_encoder init\r\n");
    vTalRotaryEncoderInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vRotaryEncoderAppTask, /* The function that implements the task. */
                "RotaryEncoderAppTask",  /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,      /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,            /* The task parameter, not used in this case. */
                uxTaskPriorityReq,       /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);                   /* The task handle is not used. */
    return 0;
}

DevObject_t RotaryEncoderObject =
    {
        .lDevObjectInit = RotaryEncoder_Object_Init,
        .devApi = RotaryEncoder_API,
};
