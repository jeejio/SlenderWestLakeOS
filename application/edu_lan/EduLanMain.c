#include <stddef.h>
#include <stdio.h>

#include "device.h"
#include "auto_init.h"
#include "esp_log.h"
#include "edu_lan_app.h"
#include "JeeRPC.h"

#include "freertos/device.h"
#include "freertos/timers.h"

DevType_t devType = 0;
int64_t org_device_id = 0;

extern DevObject_t ButtonPanelObject;
extern DevObject_t EnvSensorObject;
extern DevObject_t CodeScannerObject;
extern DevObject_t RotaryEncoderObject;
extern DevObject_t GestureSensorObject;
extern DevObject_t MagneticFieldObject;
extern DevObject_t ImuSensorObject;
extern DevObject_t JoystickObject;
extern DevObject_t LCD_Object;
extern DevObject_t LEDArrayObject;
extern DevObject_t MotorObject;
extern DevObject_t PIR_Object;
extern DevObject_t RainSensorObject;
extern DevObject_t RFID_Object;
extern DevObject_t SteeringGearObject;
extern DevObject_t WaterPumpObject;
extern DevObject_t BloodOxygenObject;
extern DevObject_t SingleSwitchObject;

DevObject_t *DevObjectGroup[] =
    {
        NULL, // 设备码从1开始
        &ButtonPanelObject,
        &EnvSensorObject,
        &CodeScannerObject,
        &RotaryEncoderObject,
        &GestureSensorObject,
        &MagneticFieldObject,
        &ImuSensorObject,
        &JoystickObject,
        &LCD_Object,
        &LEDArrayObject,
        &MotorObject,
        &PIR_Object,
        &RainSensorObject,
        &RFID_Object,
        &SteeringGearObject,
        &WaterPumpObject,
        &BloodOxygenObject,
        &SingleSwitchObject,
};

/****************devType read and write********************/

/**
 * @brief       从文件系统中读取设备ID
 *
 * @return      -1:不存在数据     other:设备ID
 *
 */
static int lDeviceTypeReadFlash(void)
{
    FILE *f;
    f = fopen("/data/dev_type.bin", "rb");
    if (f == NULL)
    {
        printf("Failed to open file for reading\n");
        return -1;
    }
    int data = -1;
    fread(&data, 4, 1, f);
    fclose(f);
    printf("read devType : %d \n", data);
    return data;
}

/**
 * @brief       将devType 写入文件系统中
 *
 *    NOTE:     如果文件不存在，则自动创建文件并写入
 *
 * @param[in]   需要写入的devType值
 *
 * @return      -1:写入失败     other:写入成功
 *
 */
static int lDeviceTypeWriteFlash(int devType)
{
    FILE *f;
    f = fopen("/data/dev_type.bin", "wb");
    if (f == NULL)
    {
        printf("Failed to open file for writing\n");
        return -1;
    }
    int data = devType;
    fwrite(&data, 4, 1, f);
    fclose(f);
    printf("write devType : %d \n", data);
    return 0;
}
#if 0 /*测试接口:每次上电读写devtype*/
#include "auto_init.h"
static void test_flash_devtype(void)
{
    int data =lDeviceTypeReadFlash();
    if(data == -1)
    {
        data =1;
        lDeviceTypeWriteFlash(data);
    }else{
        data ++;
        lDeviceTypeWriteFlash(data);
    }
}
INIT_APP_EXPORT(test_flash_devtype);
#endif

int32_t lGetDeviceType(void)
{
    return lDeviceTypeReadFlash();
}
int lSetDeviceType(int type)
{
    return lDeviceTypeWriteFlash(type);
}

void vRegisterDeviceType(void)
{
    if (!DevObjectGroup[devType])
    {
        printf("[Warn] Not set device type.\r\n");
        return;
    }
    if (!DevObjectGroup[devType]->lDevObjectInit)
    {
        printf("Device[%d] no initialization logic\r\n");
    }
    else
    {
        DevObjectGroup[devType]->lDevObjectInit(NULL);
    }
    uint8_t i;
    for (i = 0; DevObjectGroup[devType]->devApi[i].method; i++)
    {
        JeeRPC_Register(&DevObjectGroup[devType]->devApi[i]);
    }
}

static TimerHandle_t reboot_timer = NULL;
void reboot_timer_cb(TimerHandle_t timer)
{
    printf("reboot_timer_cb  +++++\r\n");
    esp_restart();
}

/************DeviceCode*************/
// deviceCode 1-17
int32_t lSetDeviceCodeProcess(void *args, char *result, int32_t resultLen)
{
    printf("[%s] is running\r\n", __FUNCTION__);
    CHECK_API_ARGS(result, resultLen);

    int32_t ret = 0;
    cJSON *root = cJSON_Parse(args);
    cJSON *sub = cJSON_GetObjectItem(root, "value1");
    if (!sub)
    {
        printf("Parse json value error\r\n");
        ret = -1;
        goto exit;
    }
    else
    {
        printf("Set device code:%d\r\n", sub->valueint);
        if (sub->valueint < SENSOR_MIN_ID || sub->valueint >= SENSOR_MAX_ID)
        {
            printf("Set device code error\r\n");
            ret = -1;
            goto e_delet_root;
        }
        printf("sub->valueint:%d, devType\r\n", sub->valueint, devType);
        if (sub->valueint != devType)
        {
            lDeviceTypeWriteFlash(sub->valueint);
            sprintf(result, "{\"code\":%d, \"message\":\"%s\"}", JERRNO_RPC_SUCCESS, "");

            reboot_timer = xTimerCreate("reboot_timer",
                                        pdMS_TO_TICKS(2000),
                                        pdTRUE,
                                        (void *)0,
                                        reboot_timer_cb);
            xTimerStart((TimerHandle_t)reboot_timer, 0);
        }
    }

e_delet_root:
    cJSON_Delete(root);

exit:
    return ret;
}

int vAppInit(void)
{
    // TODO 切换固件在这里实现
    JeeDevApi_t SetDeviceCode_API =
        {
            .method = "setDeviceCode",
            .lJeeDevApiProcess = lSetDeviceCodeProcess,
        };
    JeeRPC_Register(&SetDeviceCode_API);
    LOGI("app", "-------->app start");
    return 0;
}

INIT_APP_EXPORT(vAppInit);
