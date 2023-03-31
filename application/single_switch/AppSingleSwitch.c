#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_single_switch.h"

////////////////////////////////////////////start of ButtonPanel: 多功能按键
static bool OnOff = 0; // 单键开关：开关状态

//
//* @return      False-关闭，True-开启  默认False
//
int32_t lGetOnOffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    char isOnOff[8] = {0};

    if (OnOff)
    {
        strcpy(isOnOff, "True");
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, OnOff, isOnOff);
    }
    else
    {
        strcpy(isOnOff, "False");
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, OnOff, isOnOff);
    }
    return 0;
}

//
//* @return     CLICK-单击, DOUBLECLICK-双击, PRESS-长按
//
int32_t lGetTriggerModalityProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int ret;
    ret = lTalSingleSwitchModeReceive();
    switch (ret)
    {
    case 1:
    {
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, ret, "CLICK");
    }
    break;
    case 2:
    {
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, ret, "DOUBLECLICK");
    }
    break;
    case 3:
    {
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, ret, "PRESS");
    }
    break;
    default:
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_FAILED, ret, "NULL");
        break;
    }
    return 0;
}

/************SINGLE_SWITCH_API*************/
JeeDevApi_t SingleSwitch_API[] =
    {
        REGISTER_API("getOnOff", lGetOnOffProcess, NONE_OBJ),
        REGISTER_API("getTriggerModality", lGetTriggerModalityProcess, NONE_OBJ),
        {
            .method = NULL,
        }};

static void __vSingleSwitchAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    TalSwitchInfo_t *pstInfo = NULL;
    char arrayBuff[256] = {0};
    int connectFlag = 0;
    char isOnOff[8] = {0};
    char isMode[16] = {0};
    bool old_onoff_status = 0;
    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            int onOff_status = lTalSingleSwitchGetStatus();
            if (old_onoff_status != onOff_status)
            {
                old_onoff_status = onOff_status; // 更新状态
                if (onOff_status == 1)           // 上升沿触发
                {
                    OnOff = 1 - OnOff; // 每次上升沿都 切换一次开关状态
                    if (0 == OnOff)
                        strcpy(isOnOff, "False");
                    else
                        strcpy(isOnOff, "True");
                    sprintf(arrayBuff, "{\"code\":%d,\"onOff\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isOnOff);
                    JeeRPC_SendEvent(conn, NULL, arrayBuff);
                    memset(isOnOff, 0, 8);
                    memset(arrayBuff, 0, 256);
                    pstInfo = NULL;
                }
                continue;
            }
            int mode = lTalSingleSwitchModeReceive();
            if (mode > 0)
            {
                switch (mode)
                {
                case 1:
                {
                    strcpy(isMode, "CLICK");
                    sprintf(arrayBuff, "{\"code\":%d,\"triggerModality\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isMode);
                    JeeRPC_SendEvent(conn, NULL, arrayBuff);
                    memset(isMode, 0, 16);
                    memset(arrayBuff, 0, 256);
                    pstInfo = NULL;
                }
                break;
                case 2:
                {
                    strcpy(isMode, "DOUBLECLICK");
                    sprintf(arrayBuff, "{\"code\":%d,\"triggerModality\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isMode);
                    JeeRPC_SendEvent(conn, NULL, arrayBuff);
                    memset(isMode, 0, 16);
                    memset(arrayBuff, 0, 256);
                    pstInfo = NULL;
                }
                break;
                case 3:
                {
                    strcpy(isMode, "PRESS");
                    sprintf(arrayBuff, "{\"code\":%d,\"triggerModality\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isMode);
                    JeeRPC_SendEvent(conn, NULL, arrayBuff);
                    memset(isMode, 0, 16);
                    memset(arrayBuff, 0, 256);
                    pstInfo = NULL;
                }
                break;
                default:
                    sprintf(arrayBuff, "{\"code\":%d,\"triggerModality\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isMode);
                    JeeRPC_SendEvent(conn, NULL, arrayBuff);
                    memset(isMode, 0, 16);
                    memset(arrayBuff, 0, 256);
                    pstInfo = NULL;
                    break;
                }
            }
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int32_t SingleSwitch_Object_Init(void *args)
{
    printf("SingleSwitch_Object_Init\r\n");
    vTalSingleSwitchInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vSingleSwitchAppTask,   /* The function that implements the task. */
                "__vSingleSwitchAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,       /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,             /* The task parameter, not used in this case. */
                uxTaskPriorityReq,        /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

/************SingleSwitch_API*************/
DevObject_t SingleSwitchObject =
    {
        .lDevObjectInit = SingleSwitch_Object_Init,
        .devApi = SingleSwitch_API,
};
