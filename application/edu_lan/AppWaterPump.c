#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_waterpump.h"

MSON_DEFOBJ(WaterPumpMode,
            char *pumpMode;)

MSON_DEFOBJ(WaterPumpOnoff,
            char *Onoff;)

/************WaterPump 水泵  start*************/
int32_t lGetPumpModeProcess(void *obj, char *result, int32_t resultLen)
{
    char *mode = NULL;

    CHECK_API_ARGS(result, resultLen);

    waterPumpSpeed_t ret = ucTalGetWaterPumpMode();
    if (ret == WATER_PUMP_SPEED_FAST)
    {
        mode = "Fast";
    }
    else if (ret == WATER_PUMP_SPEED_MEDIAN)
    {
        mode = "Middle";
    }
    else if (ret == WATER_PUMP_SPEED_LOW)
    {
        mode = "Slow";
    }
    else if (ret == WATER_PUMP_SPEED_STOP)
    {
        mode = "Stop";
    }

    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, mode, "");
    return (int32_t)ret;
}

int32_t lGetPumpOnoffProcess(void *obj, char *result, int32_t resultLen)
{
    int32_t ret = 0;
    CHECK_API_ARGS(result, resultLen);
    ret = ucTalGetWaterPumpOnoff();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, (char *)(ret ? "True" : "False"), "");
    return ret;
}

int32_t lSetPumpModeProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    int32_t ret = 0;
    waterPumpSpeed_t speed;
    WaterPumpMode *waterPumpMode = (WaterPumpMode *)obj;

    if (!waterPumpMode->pumpMode)
    {
        printf("Parse json pumpMode error\r\n");
        goto exit;
    }

    if (strcmp(waterPumpMode->pumpMode, "Fast") == 0)
    {
        speed = WATER_PUMP_SPEED_FAST;
    }
    else if (strcmp(waterPumpMode->pumpMode, "Middle") == 0)
    {
        speed = WATER_PUMP_SPEED_MEDIAN;
    }
    else if (strcmp(waterPumpMode->pumpMode, "Slow") == 0)
    {
        speed = WATER_PUMP_SPEED_LOW;
    }
    else
    {
        printf("[%s]  json str err\r\n", __FUNCTION__);
        ret = -1;
        goto exit;
    }
    printf("%s pumpMode:%s\n", __FUNCTION__, waterPumpMode->pumpMode);
    vTalSetWaterPumpMode(speed);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

int32_t lSetPumpOnoffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    int32_t ret = 0;
    jee_uint8_t status;
    WaterPumpOnoff *waterPumpOnoff = (WaterPumpOnoff *)obj;

    if (!waterPumpOnoff->Onoff)
    {
        printf("Parse json waterPumpOnoff error\r\n");
        goto exit;
    }

    if (strcmp(waterPumpOnoff->Onoff, "True") == 0)
    {
        status = 1;
    }
    else if (strcmp(waterPumpOnoff->Onoff, "False") == 0)
    {
        status = 0;
    }
    else
    {
        printf("[%s]  json str err\r\n", __FUNCTION__);
        ret = -1;
        goto exit;
    }
    printf("%s Onoff:%s\n", __FUNCTION__, waterPumpOnoff->Onoff);
    vTalSetWaterPumpOnoff(status);

    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, "");

exit:
    return ret;
}

/************WaterPump 水泵  end*************/

/************WATER_PUMP_API*************/
JeeDevApi_t WaterPump_API[] =
    {
        REGISTER_API("getPumpMode", lGetPumpModeProcess, NONE_OBJ),
        REGISTER_API("getPumpOnoff", lGetPumpOnoffProcess, NONE_OBJ),
        REGISTER_API("setPumpMode", lSetPumpModeProcess, WaterPumpMode),
        REGISTER_API("setPumpOnoff", lSetPumpOnoffProcess, WaterPumpOnoff),
        {
            .method = NULL,
        },
};

int32_t WaterPump_Object_Init(void *args)
{
    printf("WATER_PUMP_ID init\r\n");
    lTalWaterPumpInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return 0;
}

DevObject_t WaterPumpObject =
    {
        .lDevObjectInit = WaterPump_Object_Init,
        .devApi = WaterPump_API,
};
