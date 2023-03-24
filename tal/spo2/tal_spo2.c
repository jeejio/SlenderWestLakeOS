#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_spo2.h"

#define SPO2_DEV "spo2_max30102"
jee_device_t spo2Dev = NULL;
static TimerHandle_t Spo2Timer = NULL;

jee_uint8_t onOffFlag=0;
jee_uint8_t dataOkFlag=0;
jee_int32_t Spo2Data=0;
jee_int32_t heartRate=0;
void vTalSpo2TimerCb(TimerHandle_t timer)
{
    jee_int32_t p[4];
    jee_device_read(spo2Dev, NULL, p,4);
    onOffFlag=p[0];
    dataOkFlag=p[1];
    Spo2Data=p[2];
    heartRate=p[3];
}
jee_int32_t lTalSpo2Init(void)
{

    // 查找设备
    spo2Dev = jee_device_find(SPO2_DEV);
    if (spo2Dev == JEE_NULL)
    {
        printf("can not find lb1427b Model\n");
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(spo2Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pin device\n");
        return JEE_ERROR;
    }

    Spo2Timer = xTimerCreate("Spo2Timer",
                                 pdMS_TO_TICKS(1000),
                                 pdTRUE,
                                 (void *)0,
                                 vTalSpo2TimerCb);

    xTimerStart(Spo2Timer, 0);

    return JEE_EOK;
}

void vTalSp02SetOnOff(jee_uint8_t OnOff)
{
    jee_device_control(spo2Dev, MAX30102_SET_ONOFFCMD, (void *)&OnOff);
}

jee_uint8_t ucTalSpo2GetOnOff(void)
{
    return onOffFlag;
}

jee_uint32_t lTalSpo2GetHeartRate(void)
{
   return heartRate; 
}

jee_uint32_t lTalSpo2GetBloodOxygen(void)
{
   return Spo2Data; 
}

jee_uint8_t ucTalSpo2GetDataFlag(void)
{
   return dataOkFlag;
}










