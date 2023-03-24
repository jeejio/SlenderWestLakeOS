#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_hyetometer.h"

/************rain sensor  start*************/
int32_t lGetIsRainingProcess(void *obj, char *result, int32_t resultLen)
{
    int32_t ret = 0;
    CHECK_API_ARGS(result, resultLen);

    ret = cTalHyetometerGetIsRaining();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");

    return ret;
}
int32_t lGetPrecipitationLevelgProcess(void *obj, char *result, int32_t resultLen)
{
    int32_t ret = 0;
    char *str = NULL;
    CHECK_API_ARGS(result, resultLen);

    ret = cTalHyetometerGetPecipitationLevel();
    if (ret == 0)
    {
        str = "None";
    }
    else if (ret == 1)
    {
        str = "Light";
    }
    else if (ret == 2)
    {
        str = "Middle";
    }
    else if (ret == 3)
    {
        str = "Heavy";
    }
    else
    {
        ret = -1;
        goto exit;
    }
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, str, "");
exit:
    return ret;
}

/************rain sensor  end*************/

/************RAIN_SENSOR_API*************/
JeeDevApi_t RainSensor_API[] =
    {
        REGISTER_API("getIsRaining", lGetIsRainingProcess, NONE_OBJ),
        REGISTER_API("getPrecipitationLevel", lGetPrecipitationLevelgProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vHyetometer_AppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    char isRaining[8] = {0};
    char precipitationLevel[7] = {0};
    jee_base_t state = PRECIPITATIONLEVEL;
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);

        printf("test_t \n");
        if (connectFlag)
        {
            HyetomterInfo_t *GetStatus = xTalHyetometerOnReceive();
            printf("isRaining:%d  precipitationLevel %d \n", GetStatus->isRaining, GetStatus->precipitationLevel);
            if (0 == GetStatus->isRaining)
                strcpy(isRaining, "False");
            else
                strcpy(isRaining, "True");
            if (0 == GetStatus->precipitationLevel)
                strcpy(precipitationLevel, "None");
            if (1 == GetStatus->precipitationLevel)
                strcpy(precipitationLevel, "Light");
            if (2 == GetStatus->precipitationLevel)
                strcpy(precipitationLevel, "Middle");
            if (3 == GetStatus->precipitationLevel)
                strcpy(precipitationLevel, "Heavy");

            if (GetStatus->precipitationLevel != state)
            {
                state = GetStatus->precipitationLevel;
                char *obuff = calloc(512, 1);
                sprintf(obuff, "{\"code\":%d,\"isRaining\":\"%s\",\"precipitationLevel\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isRaining, precipitationLevel);
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

int32_t Hyetometer_Object_Init(void *args)
{
    printf("Hyetometer init\r\n");
    vTalHyetometerInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vHyetometer_AppTask, /* The function that implements the task. */
                "Hyetometer_AppTask",  /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,    /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,          /* The task parameter, not used in this case. */
                uxTaskPriorityReq,     /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);
    return 0;
}

DevObject_t RainSensorObject =
    {
        .lDevObjectInit = Hyetometer_Object_Init,
        .devApi = RainSensor_API,
};
