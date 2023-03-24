#include "JeeRPC.h"
#include "edu_lan_app.h"
#include "tal_display.h"

////////////////////////////////////////////start of ScreenSensor :屏幕传感器
MSON_DEFOBJ(LCDText, char *text;);
MSON_DEFOBJ(LCDTextSize, int textSize;);
MSON_DEFOBJ(LCDTextColor, char *textColor;);
MSON_DEFOBJ(LCDWordSpacing, float wordSpacing;);
MSON_DEFOBJ(LCDLineSpacing, float lineSpacing;);
MSON_DEFOBJ(
    LCDStartPosition,
    struct {
        int x;
        int y;
    } startPosition;

);

int32_t lGetTextSize(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    jee_int32_t size = 0;
    size = xTalDisplayGetTextSize();

    sprintf(result, "{\"code\":%d,\"value\":%d,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, size, "");
    return 0;
}

int32_t lGetWordSpacing(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);
    float value = 0;
    value = fTalDisplayGetWordSpacing();
    sprintf(result, "{\"code\":%d,\"value\":%.2f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, value, "");
    return 0;
}

int32_t lGetLineSpacing(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_ARGS(result, resultLen);

    float value = 0;
    value = fTalDisplayGetLineSpacing();
    sprintf(result, "{\"code\":%d,\"value\":%.2f,\"message\":\"%s\"}",
            JERRNO_RPC_SUCCESS, value, "");
    return 0;
}

int32_t lSetText(void *args, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(args, result, resultLen);

    cJSON *item = NULL;
    if (args == NULL)
    {
        printf("file:%s, func:%s, line:%d,  error:args == NULL \r\n", __FILE__, __FUNCTION__, __LINE__);
        return 0;
    }

    cJSON *root = cJSON_Parse(args);
    if (!root)
    {
        printf("Parse json root error\r\n");
        goto exit;
    }
    cJSON *sub = cJSON_GetObjectItem(root, "text");
    if (!sub)
    {
        printf("Parse json \"data\" error\r\n");
        goto del_json;
    }
    else
    {
        vTalDisplayClear(BLACK);
        vTalDisplaySetTextPosition(0, 0);
        if (cJSON_IsArray(sub) == true)
        {
            // 分多行显示
            int arr_size = cJSON_GetArraySize(sub);
            for (int i = 0; i < arr_size; i++)
            {
                item = cJSON_GetArrayItem(sub, i);
                if (item != NULL)
                {
                    if (cJSON_IsString(item) == true)
                    {
                        vTalDisplayShowString(lcd.start_x, lcd.start_y, 240, 320, xTalDisplayGetTextSize(),
                                              item->valuestring);
                        if (lcd.start_y >= 320 - lcd.text_size)
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        else if (cJSON_IsString(sub) == true)
        {
            vTalDisplayShowString(0, 0, 240, 320, xTalDisplayGetTextSize(), sub->valuestring);
        }
        else
        {
            sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_FAILED, "");
            goto del_json;
        }
        sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");
    }
del_json:
    cJSON_Delete(root);

exit:
    return 0;
}

int32_t lSetTextSize(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);

    LCDTextSize *lcdTextSize = (LCDTextSize *)obj;

    int size = lcdTextSize->textSize;
    if ((size == 12) || (size == 16) || (size == 24) || (size == 32))
    {
        vTalDisplaySetTextSize(size);
        sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");
    }
    else
    {
        sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_FAILED, "");
        goto exit;
    }

exit:
    return 0;
}

/**
 * @brief  hex转bcd
 * @param[in] {Value_H} 高字节
 * @param[in] {Value_L} 低字节
 * @return 转换完成后数据
 */
unsigned char cHexToBcd(unsigned char Value_H, unsigned char Value_L)
{
    unsigned char bcd_value;

    if ((Value_H >= '0') && (Value_H <= '9'))
        Value_H -= '0';
    else if ((Value_H >= 'A') && (Value_H <= 'F'))
        Value_H = Value_H - 'A' + 10;
    else if ((Value_H >= 'a') && (Value_H <= 'f'))
        Value_H = Value_H - 'a' + 10;

    bcd_value = Value_H & 0x0f;

    bcd_value <<= 4;
    if ((Value_L >= '0') && (Value_L <= '9'))
        Value_L -= '0';
    else if ((Value_L >= 'A') && (Value_L <= 'F'))
        Value_L = Value_L - 'a' + 10;
    else if ((Value_L >= 'a') && (Value_L <= 'f'))
        Value_L = Value_L - 'a' + 10;

    bcd_value |= Value_L & 0x0f;

    return bcd_value;
}

int32_t lSetTextColor(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    LCDTextColor *lcdTextColor = (LCDTextColor *)obj;

    if (!lcdTextColor->textColor)
    {
        printf("Parse json \"data\" error\r\n");
        goto exit;
    }
    else
    {
        int len = strlen(lcdTextColor->textColor);
        if (len != 4)
        {
            sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_FAILED, "");
            goto exit;
        }
        uint16_t color = (uint16_t)cHexToBcd(lcdTextColor->textColor[0], lcdTextColor->textColor[1]) * 256 +
                         cHexToBcd(lcdTextColor->textColor[2], lcdTextColor->textColor[3]);
        vTalDisplaySetTextColor1(color);
        sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");
    }

exit:
    return 0;
}

int32_t lSetWordSpacing(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    LCDWordSpacing *lcdWordSpacing = (LCDWordSpacing *)obj;

    float space = lcdWordSpacing->wordSpacing;
    vTalDisplaySetWordSpacing(space);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

    return 0;
}

int32_t lSetLineSpacing(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    LCDLineSpacing *lcdLineSpacing = (LCDLineSpacing *)obj;

    float space = lcdLineSpacing->lineSpacing;
    vTalDisplaySetLineSpacing(space);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

    return 0;
}

int32_t lSetStartPosition(void *obj, char *result, int32_t resultLen)
{
    CHECK_API_OBJS(obj, result, resultLen);
    LCDStartPosition *lcdStartPosition = (LCDStartPosition *)obj;
    int x = 0, y = 0;
    x = lcdStartPosition->startPosition.x;
    y = lcdStartPosition->startPosition.y;
    vTalDisplaySetTextPosition(x, y);
    sprintf(result, "{\"code\":%d,\"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");
    char buff[128];
    sprintf(buff, "pos:%d,%d", x, y);
    vTalDisplayShowString(0, 0, 240, 320, 16, buff);

    return 0;
}
////////////////////////////////////////////end of ScreenSensor :屏幕传感器

/************LCD_API*************/
JeeDevApi_t LCD_API[] =
    {
        REGISTER_API("getTextSize", lGetTextSize, NONE_OBJ),
        REGISTER_API("getWordSpacing", lGetWordSpacing, NONE_OBJ),
        REGISTER_API("getLineSpacing", lGetLineSpacing, NONE_OBJ),
        {.method = "setText",
         .lJeeDevApiProcess = lSetText},
        REGISTER_API("setTextSize", lSetTextSize, LCDTextSize),                //
        REGISTER_API("setTextColor", lSetTextColor, LCDTextColor),             //?
        REGISTER_API("setWordSpacing", lSetWordSpacing, LCDWordSpacing),       // deceng
        REGISTER_API("setLineSpacing", lSetLineSpacing, LCDLineSpacing),       // diceng
        REGISTER_API("setStartPosition", lSetStartPosition, LCDStartPosition), // diceng
        {
            .method = NULL,
        },

};

static void __vDisplayAppTask(void *pvParameters)
{
    PJeeMQTTConnection_t conn = (PJeeMQTTConnection_t)pvParameters;

    while (1)
    {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

int32_t Display_Object_Init(void *args)
{
    printf("Display_Object_Init\r\n");
    lTalDisplayInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    uint16_t usTaskStackSizeReq = 4096 * 4;
    UBaseType_t uxTaskPriorityReq = 6;
    xTaskCreate(__vDisplayAppTask,   /* The function that implements the task. */
                "__vDisplayAppTask", /* Just a text name for the task to aid debugging. */
                usTaskStackSizeReq,  /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)conn,        /* The task parameter, not used in this case. */
                uxTaskPriorityReq,   /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);

    return 0;
}

DevObject_t LCD_Object =
    {
        .lDevObjectInit = Display_Object_Init,
        .devApi = LCD_API,
};
