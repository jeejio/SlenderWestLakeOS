#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_magnetic.h"

MSON_DEFOBJ(MagneticFieldIsMagnetic, int IsMagnetic;);

// 0:True
// 1:False
int32_t lgetMagneticProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    printf("[%s] is running\r\n", __FUNCTION__);

    jee_uint8_t ret = ucTalGetMagStatus();
    printf("mag status:%d\r\n", ret);
    ret &= 0x07;

    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");

    return ret;
}

// 1:True
// 0:False
int32_t lgetIsMagneticProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    int32_t ret = 0;
    MagneticFieldIsMagnetic *magneticFieldIsMagnetic = (MagneticFieldIsMagnetic *)obj;

    printf("Magnetic chanle is:%d\r\n", magneticFieldIsMagnetic->IsMagnetic);
    if (magneticFieldIsMagnetic->IsMagnetic < 1 || magneticFieldIsMagnetic->IsMagnetic > 3)
    {
        printf("Magnetic chanle error\r\n");
        ret = -1;
        goto exit;
    }
    ret = ucTalGetIsMagnetic(magneticFieldIsMagnetic->IsMagnetic);
    sprintf(result, "{\"code\":%d, \"ID\":%d, \"isMagnetic\":\"%s\", \"message\":\"%s\"}", JERRNO_RPC_SUCCESS,
            magneticFieldIsMagnetic->IsMagnetic, (char *)(ret ? "True" : "False"), "");

exit:
    return ret;
}

/************HALL_SENSOR_API*************/
JeeDevApi_t MagneticField_API[] =
    {
        REGISTER_API("getMagnetic", lgetMagneticProcess, NONE_OBJ),
        REGISTER_API("getIsMagnetic", lgetIsMagneticProcess, MagneticFieldIsMagnetic),
        {
            .method = NULL,
        },
};

static void __vMagneticAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;
    jee_uint8_t data = 0, dataOld = 0;
    char magnetic[8] = {0};
    char isMagnetic1[8] = {0};
    char isMagnetic2[8] = {0};
    char isMagnetic3[8] = {0};
    int connectFlag = 0;

    while (1)
    {
        connectFlag = lMqtt_is_connected(conn);
        if (connectFlag)
        {
            data = ucTalGetMagStatus();
            if (dataOld != data)
            {
                if(data & 0x07)
                {
                    strcpy(magnetic, "True");
                }
                else
                {
                    strcpy(magnetic, "False");
                }

                if ((data & 0x01) != (dataOld & 0x01))
                {
                    if ((data & 0x01) != 0)
                    {
                        strcpy(isMagnetic1, "True");
                        printf("mag 1 open\n");
                    }
                    else
                    {
                        strcpy(isMagnetic1, "False");
                        printf("mag 1 close\n");
                    }
                }
                else
                {
                    if ((data & 0x01) != 0)
                    {
                        strcpy(isMagnetic1, "True");
                        printf("mag 1 open\n");
                    }
                    else
                    {
                        strcpy(isMagnetic1, "False");
                        printf("mag 1 close\n");
                    }
                }

                if ((data & 0x02) != (dataOld & 0x02))
                {
                    if ((data & 0x02) != 0)
                    {
                        strcpy(isMagnetic2, "True");
                        printf("mag 2 open\n");
                    }
                    else
                    {
                        strcpy(isMagnetic2, "False");
                        printf("mag 2 close\n");
                    }
                }
                else
                {
                    if ((data & 0x02) != 0)
                    {
                        strcpy(isMagnetic2, "True");
                        printf("mag 2 open\n");
                    }
                    else
                    {
                        strcpy(isMagnetic2, "False");
                        printf("mag 2 close\n");
                    }
                }

                if ((data & 0x04) != (dataOld & 0x04))
                {
                    if ((data & 0x04) != 0)
                    {
                        strcpy(isMagnetic3, "True");
                        printf("mag 3 open\n");
                    }
                    else
                    {
                        strcpy(isMagnetic3, "False");
                        printf("mag 3 close\n");
                    }
                }
                else
                {
                    if ((data & 0x04) != 0)
                    {
                        strcpy(isMagnetic3, "True");
                        printf("mag 3 open\n");
                    }
                    else
                    {
                        strcpy(isMagnetic3, "False");
                        printf("mag 3 close\n");
                    }
                }
                dataOld = data;

                char *obuff = malloc(128);
                memset(obuff, 0, 128);
                sprintf(obuff, "{\"code\":%d,\"isMagnetic1\":\"%s\",\"isMagnetic2\":\"%s\",\"isMagnetic3\":\"%s\",\"magnetic\":\"%s\",\"msg\":\"\"}", JERRNO_RPC_SUCCESS, isMagnetic1, isMagnetic2, isMagnetic3,magnetic);
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

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

int32_t Magnetic_Object_Init(void *args)
{
    printf("Hall sensor init\r\n");
    lTalMagneticInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 4;
    xTaskCreate(__vMagneticAppTask, /* The function that implements the task. */
                "MagneticAppTask",  /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq, /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,       /* The task parameter, not used in this case. */
                uxTaskPriorityReq,  /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);
    return 0;
}

DevObject_t MagneticFieldObject =
    {
        .lDevObjectInit = Magnetic_Object_Init,
        .devApi = MagneticField_API,
};
