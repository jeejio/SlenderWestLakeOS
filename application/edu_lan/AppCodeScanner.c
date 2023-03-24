#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_scanner.h"

static void __vOnTransRev_Test(const char *result)
{
    printf("[NeedCloudToDevideRsp!!!\n]");
}

/************BUTTON_PANEL_API  START*************/
int32_t lCodeScannerGetCodeProcess(void *obj, char *result, int32_t resultLen)
{
    int32_t ret = 0;
    CHECK_API_ARGS(result, resultLen);
    uint8_t data[180] = {0};
    jee_uint8_t data_len = 0;
    int counter = 0;
    do
    {
        data_len = ucTalScannerGetCode(data);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (counter++ > 50)
        {
            strcpy((char *)data, "None");
            break;
        }
    } while (0 == data_len);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, data, "");

    return ret;
}

/************CODE_SCANNER*************/
JeeDevApi_t CodeScanner_API[] =
    {
        REGISTER_API("getCode", lCodeScannerGetCodeProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vCodeScannerAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    uint8_t data[180];
    uint8_t len = 0;
    int connectFlag = 0;

    while (1)
    {
        printf("test_t \n");
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {

            memset(data, 0, sizeof(data));
            len = ucTalScannerGetCode(data);
            printf("len %d\n", len);
            if (0 != len)
            {
                len -= len;
                printf("scan %s\n", data);
                char *obuff = calloc(512, 1);
                sprintf(obuff, "{\"code\":%d,\"scanCode\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, data);
                JeeRPC_SendEvent(conn, NULL, obuff);
                free(obuff);
                obuff = NULL;
                printf("test--------------------\n");
            }
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int32_t CodeScanner_Object_Init(void *args)
{
    printf("CodeScanner init\r\n");

    lTalScannerInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vCodeScannerAppTask,   /* The function that implements the task. */
                "__vCodeScannerAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,      /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,            /* The task parameter, not used in this case. */
                uxTaskPriorityReq,       /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);
    return 0;
}

/************CODE_SCANNER*************/
DevObject_t CodeScannerObject =
    {
        .lDevObjectInit = CodeScanner_Object_Init,
        .devApi = CodeScanner_API,
};
