#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_rfid.h"

////////////////////////////////////////////start of RFID :读写器

MSON_DEFOBJ(RFIDData,
            char *data;);

//
//
//
int32_t lGetRfidIDProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    uint8_t ucID[4] = {0};

    vTalRfidGetID(ucID);

    sprintf(result, "{\"code\":%d,\"value\":\"%02X%02X%02X%02X\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, ucID[0], ucID[1], ucID[2], ucID[3], "");
    return 0;
}

//
//
//
int32_t lGetRfidDataProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    uint8_t ucID[16] = {0};

    vTalRfidGetData(ucID);

    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)ucID, "");
    return 0;
}

//
//
//
int32_t lSetRfidDataProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    RFIDData *rfidData = (RFIDData *)obj;

    if (!rfidData->data)
    {
        printf("Parse json \"data\" error\r\n");
        goto exit;
    }
    vTalRfidSetData((uint8_t *)rfidData->data);

    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");
exit:
    return 0;
}

////////////////////////////////////////////end of RFID :读写器

/************RFID_API*************/
JeeDevApi_t RFID_API[] =
    {
        REGISTER_API("getID", lGetRfidIDProcess, NONE_OBJ),
        REGISTER_API("getData", lGetRfidDataProcess, NONE_OBJ),
        REGISTER_API("setData", lSetRfidDataProcess, RFIDData),
        {
            .method = NULL,
        },
};

static void __vRFIDAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    uint8_t ucID[4] = {0};
    uint8_t ucData[16] = {0};
    char arrayBuff[128] = {0};
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            vTalRfidOnReceive(ucID, ucData);
            printf("ID:%s, data:%s\n", (char *)ucID, (char *)ucData);
            sprintf(arrayBuff, "{\"code\":%d,\"id\":\"%02X%02X%02X%02X\",\"data\":\"%s\",\"message\":\"%s\"}",
                    JERRNO_RPC_SUCCESS, ucID[0], ucID[1], ucID[2], ucID[3], (char *)ucData, "");
            JeeRPC_SendEvent(conn, NULL, arrayBuff);
            memset(arrayBuff, 0, 128);
            memset(ucID, 0, 4);
            memset(ucData, 0, 16);
        }
        else
        {
            printf("mqtt dis connect\r\n");
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS); // 周期上报，周期为3s
    }
}

int32_t RFID_Object_Init(void *args)
{
    printf("RFID_Object_Init\r\n");
    vTalRfidInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vRFIDAppTask,     /* The function that implements the task. */
                "__vRFIDAppTask",   /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq, /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,       /* The task parameter, not used in this case. */
                uxTaskPriorityReq,  /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

DevObject_t RFID_Object =
    {
        .lDevObjectInit = RFID_Object_Init,
        .devApi = RFID_API,
};
