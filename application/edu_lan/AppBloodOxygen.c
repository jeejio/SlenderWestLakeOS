#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_spo2.h"

/************HeartRateMonitor 心率血氧检测仪  start*************/

MSON_DEFOBJ(HeartRateOnOff,
            char *onOff;);

int32_t lgetOnOffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    jee_uint8_t onOffFlag = ucTalSpo2GetOnOff();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(onOffFlag ? "Ture" : "False"), "");
    return 0;
}

int32_t lgetHeartRateProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    jee_uint8_t onOffFlag = ucTalSpo2GetOnOff();
    if (1 == onOffFlag)
    {
        jee_uint32_t heartRate = lTalSpo2GetHeartRate();
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, heartRate, "");
    }
    else
    {
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, 0, "");
    }
    return 0;
}

int32_t lgetBloodOxygenProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    jee_uint8_t onOffFlag = ucTalSpo2GetOnOff();
    if (1 == onOffFlag)
    {
        jee_uint32_t Spo2Data = lTalSpo2GetBloodOxygen();
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, Spo2Data, "");
    }
    else
    {
        sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
                JERRNO_RPC_SUCCESS, 0, "");
    }
    return 0;
}

int32_t lsetOnOffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    int32_t ret = 0;
    uint8_t onoff = 0;
    HeartRateOnOff *heartRateOnOff = (HeartRateOnOff *)obj;

    if (!strcmp(heartRateOnOff->onOff, "True"))
    {
        onoff = 1;
    }
    else if (!strcmp(heartRateOnOff->onOff, "False"))
    {
        onoff = 0;
    }
    else
    {
        printf("Set HeartRateMonitor onOff value error\r\n");
        ret = -1;
        goto exit;
    }
    printf("Set HeartRateMonitor on off:%s->%d\r\n", heartRateOnOff->onOff, onoff);
    vTalSp02SetOnOff(onoff);
    sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}
/************HeartRateMonitor 心率血氧检测仪  end*************/

/************BLOOD_OXYGEN_API*************/
JeeDevApi_t BloodOxygen_API[] =
    {
        REGISTER_API("getOnOff", lgetOnOffProcess, NONE_OBJ),
        REGISTER_API("getHeartRate", lgetHeartRateProcess, NONE_OBJ),
        REGISTER_API("getBloodOxygen", lgetBloodOxygenProcess, NONE_OBJ),
        REGISTER_API("setOnOff", lsetOnOffProcess, HeartRateOnOff),
        {
            .method = NULL,
        },
};

static void __vBloodOxygen_AppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    char onoff[8] = {0};
    int state = ISEXISTS;
    int connectFlag = 0;
    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {

            jee_uint8_t onOff = ucTalSpo2GetOnOff();
            jee_uint32_t heartRate = lTalSpo2GetHeartRate();
            jee_uint32_t bloodOxygen = lTalSpo2GetBloodOxygen();
            if (1 == onOff)
                strcpy(onoff, "True");
            else
                strcpy(onoff, "Flase");
            if ((onOff != state) || (1 == onOff))
            {
                state = onOff;
                char *obuff = malloc(512);
                memset(obuff, 0, 512);
                if (1 == onOff)
                    sprintf(obuff, "{\"code\":%d,\"onOff\":\"%s\",\"heartRate\":%d,\"bloodOxygen\":%d,\"msg\":\"\"}",
                            JERRNO_RPC_SUCCESS, onoff, heartRate, bloodOxygen);
                else
                    sprintf(obuff, "{\"code\":%d,\"onOff\":\"%s\",\"heartRate\":%d,\"bloodOxygen\":%d,\"msg\":\"\"}", JERRNO_RPC_SUCCESS, onoff, 0, 0);
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

int32_t BloodOxygen_Object_Init(void *args)
{
    printf("BloodOxygen init\r\n");
    lTalSpo2Init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vBloodOxygen_AppTask, /* The function that implements the task. */
                "BloodOxygen_AppTask",  /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,     /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,           /* The task parameter, not used in this case. */
                uxTaskPriorityReq,      /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);
    return 0;
}

DevObject_t BloodOxygenObject =
    {
        .lDevObjectInit = BloodOxygen_Object_Init,
        .devApi = BloodOxygen_API,
};
