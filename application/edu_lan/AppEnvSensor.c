#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_environment.h"

////////////////////////////////////////////start of EnvironmentSensor :环境传感器
static SemaphoreHandle_t environmentMutex = NULL;
//
//
//
int32_t lGetTemperatureProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    jee_int32_t pdata[2] = {0};
    xSemaphoreTake(environmentMutex, portMAX_DELAY);
    vTalGetTemHumData(pdata);
    xSemaphoreGive(environmentMutex);
    sprintf(result, "{\"code\":%d,\"value\":%.1f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, pdata[0] / 10.0f, "");
    return 0;
}

//
//
//
int32_t lGetHumidityProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    jee_int32_t pdata[2] = {0};
    xSemaphoreTake(environmentMutex, portMAX_DELAY);
    vTalGetTemHumData(pdata);
    xSemaphoreGive(environmentMutex);
    sprintf(result, "{\"code\":%d,\"value\":%.2f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, pdata[1] / 100.0f, "");
    return 0;
}

//
//
//
int32_t lGetLightIntensityProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    xSemaphoreTake(environmentMutex, portMAX_DELAY);
    jee_uint16_t ret = usTalGetAmbientLightData();
    xSemaphoreGive(environmentMutex);
    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ret, "");
    return 0;
}

//
//
//
int32_t lGetCoStatusProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    xSemaphoreTake(environmentMutex, portMAX_DELAY);
    jee_uint8_t ret = ucTalMq7GetCoStatus();
    xSemaphoreGive(environmentMutex);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return 0;
}

//
//
//
int32_t lGetSmokeStatusProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    xSemaphoreTake(environmentMutex, portMAX_DELAY);
    jee_uint8_t ret = ucTalMq2GetSmokeStatus();
    xSemaphoreGive(environmentMutex);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return 0;
}

//
//
//
int32_t lGetAirStatusProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    xSemaphoreTake(environmentMutex, portMAX_DELAY);
    jee_uint8_t ret = ucTalMq135GetAirStatus();
    xSemaphoreGive(environmentMutex);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return 0;
}

////////////////////////////////////////////end of EnvironmentSensor :环境传感器

/************ENV_SENSOR*************/
JeeDevApi_t EnvSensor_API[] =
    {
        REGISTER_API("getTemperature", lGetTemperatureProcess, NONE_OBJ),
        REGISTER_API("getHumidity", lGetHumidityProcess, NONE_OBJ),
        REGISTER_API("getLightIntensity", lGetLightIntensityProcess, NONE_OBJ),
        REGISTER_API("getCoStatus", lGetCoStatusProcess, NONE_OBJ),
        REGISTER_API("getSmokeStatus", lGetSmokeStatusProcess, NONE_OBJ),
        REGISTER_API("getAirStatus", lGetAirStatusProcess, NONE_OBJ),
        {
            .method = NULL,
        },
};

static void __vEnvSensorAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    jee_int32_t pTemHumdata[2] = {0};
    jee_uint16_t retLight = 0;
    jee_uint8_t retCo = 0;
    jee_uint8_t retSmoke = 0;
    jee_uint8_t retAir = 0;
    char arrayBuff[256] = {0};
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            xSemaphoreTake(environmentMutex, portMAX_DELAY);
            vTalGetTemHumData(pTemHumdata);
            retLight = usTalGetAmbientLightData();
            retCo = ucTalMq7GetCoStatus();
            retSmoke = ucTalMq2GetSmokeStatus();
            retAir = ucTalMq135GetAirStatus();
            xSemaphoreGive(environmentMutex);

            printf("temperature:%.1f, humidity:%.2f, lightIntensity:%d, coStatus:%s, smokeStatus:%s, airStatus:%s,\n",
                   pTemHumdata[0] / 10.0f, pTemHumdata[1] / 100.0f, retLight, (retCo ? "True" : "False"), (retSmoke ? "True" : "False"), (retAir ? "True" : "False"));
            sprintf(arrayBuff, "{\"code\":%d,\"temperature\":%.1f,\"humidity\":%.2f,\"lightIntensity\":%d,\"coStatus\":\"%s\",\"smokeStatus\":\"%s\",\"airStatus\":\"%s\",\"message\":\"%s\"}",
                    JERRNO_RPC_SUCCESS, pTemHumdata[0] / 10.0f, pTemHumdata[1] / 100.0f, retLight, (retCo ? "True" : "False"), (retSmoke ? "True" : "False"), (retAir ? "True" : "False"), "");
            JeeRPC_SendEvent(conn, NULL, arrayBuff);
            memset(arrayBuff, 0, 256);
            memset(pTemHumdata, 0, 8); 
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS); // 周期上报，周期为3s
    }
}

int32_t EnvSensor_Object_Init(void *args)
{
    printf("EnvSensor_Object_Init\r\n");
    lTalAmbientLightInit();
    lTalTempHumInit();
    lTalMq2Init();
    lTalMq7Init();
    lTalMq135Init();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    environmentMutex = xSemaphoreCreateMutex();
    if (!environmentMutex)
    {
        printf("create mutex error\n");
    }
    xSemaphoreGive(environmentMutex);
    
    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vEnvSensorAppTask,   /* The function that implements the task. */
                "__vEnvSensorAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,    /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,          /* The task parameter, not used in this case. */
                uxTaskPriorityReq,     /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

/************ENV_SENSOR*************/
DevObject_t EnvSensorObject =
    {
        .lDevObjectInit = EnvSensor_Object_Init,
        .devApi = EnvSensor_API,
};
