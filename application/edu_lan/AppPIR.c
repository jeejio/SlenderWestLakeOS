#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_pir.h"

/***************PIR Sensor**************/
// 0：未检测到人体
// 1：检测到人体
int32_t lGetIsExistsProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = lTalGetIsExists();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");

    return ret;
}

/************PIR_SENSOR_API*************/
JeeDevApi_t PIR_API[] =
    {
        REGISTER_API("getIsExists", lGetIsExistsProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vPIR_AppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    char isExists[8] = {0};
    jee_base_t state = ISEXISTS;
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            printf("test_t \n");
            jee_base_t ret = lTalGetIsExists();
            printf("getIsExists ret = %d\r\n", ret);
            if (0 == ret)
                strcpy(isExists, "False");
            else
                strcpy(isExists, "True");
            if (ret != state)
            {
                state = ret;
                char *obuff = malloc(128);
                memset(obuff, 0, 128);
                sprintf(obuff, "{\"code\":%d,\"isExists\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isExists);
                JeeRPC_SendEvent(conn, NULL, obuff);
                free(obuff);
                obuff = NULL;
                printf("test--------------------\n");
            }
#if 0
        hal_water_pump_set_speed(i++);
        if(i>2) i=0;
        printf("speed %d \n",hal_water_pump_get_speed());
#endif
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

int32_t PIR_Object_Init(void *args)
{
    printf("PIR init\r\n");
    vTalPirInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vPIR_AppTask,     /* The function that implements the task. */
                "PIR_AppTask",      /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq, /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,       /* The task parameter, not used in this case. */
                uxTaskPriorityReq,  /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);
    return 0;
}

DevObject_t PIR_Object =
    {
        .lDevObjectInit = PIR_Object_Init,
        .devApi = PIR_API,
};
