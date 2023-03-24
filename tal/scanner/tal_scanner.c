#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "esp_log.h"
#include "tal_scanner.h"
#include <string.h>

#define SCANNER_DEV "scan_md280"
jee_device_t scanner_dev = NULL;

jee_int32_t lTalScannerInit(void)
{

    // 查找设备
    scanner_dev = jee_device_find(SCANNER_DEV);
    if (scanner_dev == JEE_NULL)
    {
        printf("can not find scan_md280 Model\n");
        return JEE_ERROR;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(scanner_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open scan_md280 device\n");
        return JEE_ERROR;
    }
    return JEE_EOK;
}

jee_uint8_t ucTalScannerGetCode(jee_uint8_t *readdData)
{
    jee_uint8_t data[150];
    jee_device_read(scanner_dev, NULL, data, 1);
    memcpy(readdData,&data[1],data[0]);
    return data[0];
}

