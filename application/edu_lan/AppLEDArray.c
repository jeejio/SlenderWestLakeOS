#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_led_array.h"

/*****************LEDArray*******************/
MSON_DEFOBJ(RGBColor,
            jee_uint8_t red;
            jee_uint8_t green;
            jee_uint8_t blue;);

MSON_DEFOBJ(RGBOnoff,
            char *onOff;);

MSON_DEFOBJ(RGBFlashNumber,
            jee_uint32_t flashNumber;);

MSON_DEFOBJ(RGBFlashMode,
            char *flashMode;);

int32_t lLEDArrayGetRgbColorProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    color_t color = xTalLedArrayGetRgbColor();
    sprintf(result, "{\"code\":%d,\"red\":%d,\"green\":%d,\"blue\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, color.red, color.green, color.blue, "");
    return 0;
}

int32_t lLEDArrayGetStatusProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    jee_uint8_t status = ucTalLedArrayGetStatus();
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, status ? "True" : "False", "");
    return 0;
}

int32_t lLEDArrayGetFlashModeProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    FlashMode mode = xTalLedArrayGetFlashMode();
    char *modeStr = "Fast";
    if (ledArryFlashModeFast == mode)
    {
        modeStr = "Fast";
    }
    else if (ledArryFlashModeMiddle == mode)
    {
        modeStr = "Middle";
    }
    else if (ledArryFlashModeSlow == mode)
    {
        modeStr = "Slow";
    }
    else
    {
        modeStr = "None";
    }
    printf("mode:%d, modeStr:%s\r\n", mode, modeStr);
    sprintf(result, "{\"code\":%d,\"value\":\"%s\",\"message\":\"%s\"}",
            strlen(modeStr) ? JERRNO_RPC_SUCCESS : JERRNO_RPC_FAILED, modeStr, "");
    return 0;
}

int32_t lLEDArraySetRgbColorProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    color_t color = *(color_t *)obj;

    vTalLedArraySetRgbColor(color);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

    return 0;
}

int32_t lLEDArraySetOnOffProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    RGBOnoff *rgbOnoff = (RGBOnoff *)obj;
    jee_uint8_t onoff = 0;

    if (!rgbOnoff->onOff)
    {
        printf("Parse json LED \"OnOff\" error\r\n");
        goto exit;
    }
    else
    {
        if (!strcmp(rgbOnoff->onOff, "True"))
        {
            onoff = 1;
        }
        else if (!strcmp(rgbOnoff->onOff, "False"))
        {
            onoff = 0;
        }
        else
        {
            printf("Set onoff value error:%s\r\n", rgbOnoff->onOff);
            goto exit;
        }
    }
    printf("Set LED onoff:%d\r\n", onoff);
    vTalLedArraySetOnOff(onoff);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return 0;
}

int32_t lLEDArraySetFlashNumberProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    RGBFlashNumber *rgbFlashNumber = (RGBFlashNumber *)obj;
    jee_uint32_t flashNumber = rgbFlashNumber->flashNumber;

    printf("Set LED flashNumber:%d\r\n", flashNumber);
    vTalLedArraySetFlashNumber(flashNumber);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

    return 0;
}

int32_t lLEDArraySetFlashModeProcess(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    RGBFlashMode *rgbFlashMode = (RGBFlashMode *)obj;
    FlashMode speed = ledArryFlashModeNull;

    if (!rgbFlashMode->flashMode)
    {
        printf("Parse json LED flashMode error\r\n");
        goto exit;
    }
    else
    {
        // // speed = sub->valueint;
        printf("Set LED flashMode:%s\r\n", rgbFlashMode->flashMode);
        if (strcmp("Fast", rgbFlashMode->flashMode) == 0)
        {
            speed = ledArryFlashModeFast;
        }
        else if (strcmp("Middle", rgbFlashMode->flashMode) == 0)
        {
            /* code */
            speed = ledArryFlashModeMiddle;
        }
        else if (strcmp("Slow", rgbFlashMode->flashMode) == 0)
        {
            speed = ledArryFlashModeSlow;
        }
        else
        {
            sprintf(result, "{\"code\":%d,\"message\":\"error\"}", JERRNO_RPC_FAILED, "");
            goto exit;
        }
    }

    printf("Set LED flashMode:%s\r\n", rgbFlashMode->flashMode);
    vTalLedArraySetFlashMode(speed);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

exit:
    return 0;
}

/************LED_ARRAY_API*************/
JeeDevApi_t LEDArray_API[] =
    {
        REGISTER_API("getRgbColor", lLEDArrayGetRgbColorProcess, NONE_OBJ),
        REGISTER_API("getOnOff", lLEDArrayGetStatusProcess, NONE_OBJ),
        REGISTER_API("getFlashMode", lLEDArrayGetFlashModeProcess, NONE_OBJ),
        REGISTER_API("setRgbColor", lLEDArraySetRgbColorProcess, RGBColor),
        REGISTER_API("setOnOff", lLEDArraySetOnOffProcess, RGBOnoff),
        REGISTER_API("setFlashNumber", lLEDArraySetFlashNumberProcess, RGBFlashNumber),
        REGISTER_API("setFlashMode", lLEDArraySetFlashModeProcess, RGBFlashMode),
        {
            .method = NULL,
        },
};

int32_t LEDArray_Object_Init(void *args)
{
    printf("LED array init\r\n");
    lTalLedArrayInit();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    return 0;
}

DevObject_t LEDArrayObject =
    {
        .lDevObjectInit = LEDArray_Object_Init,
        .devApi = LEDArray_API,
};
