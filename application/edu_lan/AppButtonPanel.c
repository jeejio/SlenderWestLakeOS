#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_button_panel.h"

////////////////////////////////////////////start of ButtonPanel: 多功能按键

//
//* @return      0： 没有按键按下，1-9：按下的按键序号
//
int32_t lGetButtonCodeProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int ret = 0;
    ret = cTalButtonPanelGetButtonCodes();

    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return ret;
}

//
//    * @return     按键按下模式
//    *             1 - 单击 @ref 500ms 秒内完成按键动作
//    *             2 - 双击 @ref 连续两次完成单击动作
//    *             3 - 多击 @ref 连续三次及以上完成单击动作
//    *             5 - 短按 @ref 500ms-5s 完成单次按下动作
//    *             7 - 长按 @ref 5s-9s 内完成单次按下动作
//    *             9 - 长按保持 @ref 超过9s 完成单次按下动作
//
int32_t lGetButtonModeProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int ret = 0;
    ret = cTalButtonPanelGetButtonMode();

    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return ret;
}

////////////////////////////////////////////end of ButtonPanel: 多功能按键

/************BUTTON_PANEL_API*************/
JeeDevApi_t ButtonPanel_API[] =
    {
        REGISTER_API("buttonCodes", lGetButtonCodeProcess, NONE_OBJ),
        REGISTER_API("getButtonMode", lGetButtonModeProcess, NONE_OBJ),
        {
            .method = NULL,
        }};

static void __vButtonPanelAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    ButtonPanelInfo_t *pstInfo = NULL;
    char arrayBuff[128] = {0};
    int connectFlag = 0;

    while (1)
    {

        pstInfo = cTalButtonPanelOnReceive();
        if (pstInfo == NULL || pstInfo->value == 0) // 返回值不合法
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            printf("value:%d, buttonMode:%d\n", pstInfo->value, pstInfo->button_mode);
            sprintf(arrayBuff, "{\"code\":%d,\"value\":%d,\"buttonMode\":%d}",
                    JERRNO_RPC_SUCCESS, pstInfo->value, pstInfo->button_mode);
            JeeRPC_SendEvent(conn, NULL, arrayBuff);
            memset(arrayBuff, 0, 128);
            pstInfo = NULL;
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }
        // vTaskDelay(3000 / portTICK_PERIOD_MS); // 周期上报，周期为3s
    }
}

int32_t ButtonPanel_Object_Init(void *args)
{
    printf("ButtonPanel_Object_Init\r\n");
    vTalButtonPanelInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vButtonPanelAppTask,   /* The function that implements the task. */
                "__vButtonPanelAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,      /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,            /* The task parameter, not used in this case. */
                uxTaskPriorityReq,       /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

/************BUTTON_PANEL_API*************/
DevObject_t ButtonPanelObject =
    {
        .lDevObjectInit = ButtonPanel_Object_Init,
        .devApi = ButtonPanel_API,
};
