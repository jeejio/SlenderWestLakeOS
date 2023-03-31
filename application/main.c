#include <stdio.h>
#include "esp_log.h"
#include "hal_jeeconfig.h"
#include "hal_wlan_mgnt.h"
#include "hal_wlan_prot.h"
#include "hal_wlan_cfg.h"
#include "hal_wlan_dev.h"
#include "jeedef.h"
#include "task.h"
#include "auto_init.h"
#include <string.h>
#include "net_socket.h"

#include "JeeMQTTConnection.h"
#include "JeeMQTTTransaction.h"
#include "JeeMQTTMessage.h"
#include "JeeRPC.h"
#include "JeeKeyManager.h"

#include "freertos/device.h"
#include "smt_connect.h"
#include <stdlib.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "factory_test.h"

#include "nvs_flash.h"
#include "edu_lan_app.h"
#include "cloud_http.h"


#define DBG_TAG "main"

#define PRINTF_BASE_INFO() (printf("Base info, file:%s func:%s line:%d\r\n", __FILE__, __FUNCTION__, __LINE__))
#define VERSION "0.0.4"

const char *base_path = "/data";
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static bool mounted_flag = false;

static void mount_fatfs()
{
    // printf("Mounting FAT filesystem\n");
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = false,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE};
    esp_err_t err;
    err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK)
    {
        printf("Failed to mount FATFS (%s)\n", esp_err_to_name(err));
        return;
    }
    else
    {
        mounted_flag = true;
    }
}

static void __vreport_online(void)
{
    jee_uint8_t mac[6] = {0};
    char *obuff = calloc(128, 1);

    printf("[%s] is runing\r\n", __FUNCTION__);

    if (rt_wlan_get_mac(mac))
    {
        printf("Get mac error\r\n");
    }

    PJeeMQTTMessage_t request = pxJeeMQTTMessage_Creator();
    strcpy(request->method, "onoffline");
    // FIXME 这个接口需要重构。ReportOnLine 接口是通用接口，不应当携带 devType 参数。
    // devType 是教育线专有参数。
    sprintf(obuff, "{\"clientId\":\"%lld\",\"status\":%d,\"firmwareCode\":%d,"
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}", \
        org_device_id, 1, devType, \
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    vJeeMQTTMessage_SetPayload(request, obuff);
    printf("[online->payload:%s]\n", request->payload);
    strcpy(request->jsonrpc, "1.2");

    PJeeMQTTTransaction_t trans = conn->pxCreateTransaction(conn, request, NULL);
    trans->vOnTransactionReceive = NULL;
    trans->vSendRequest(trans);
    free(obuff);
    obuff = NULL;
}

static void __vMqttConnectCallback(void *c)
{
    printf("inter mqtt_connect_callback!\r\n");
}

static void __vMqttOnlineCallback(void *c)
{
    printf("inter mqtt_online_callback!\r\n");
    // __vreport_online();
}

static void __vMqttOfflineCallback(void *c)
{
    printf("inter mqtt_offline_callback!\r\n");
}

static void vTestMQTT(void)
{
    const char *host = "tcp://mqtt-cn-zpr30s05k03.mqtt.aliyuncs.com:1883";
    const char *userName = "Token|LTAI5t8JjkjmcH8RZps1xx2S|mqtt-cn-zpr30s05k03";

    int8_t ret = 0;
    printf("wait get  token  \r\n");

    xSemaphoreTake(cloudMutex, portMAX_DELAY);

    do
    {
        ret = lGetIntDeviceID(&org_device_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } while (ret < 0);

    devType = lGetDeviceType();
    if (-1 == devType)
    {
        devType = 0;
        lSetDeviceType(devType);
    }


    devType = SINGLE_SWITCH_ID;
    printf("Get device type:%d\r\n", devType);

    if (lRegister_DevApiProcess(JeeRPC_Call))
    {
        printf("Register device api process failed\r\n");
    }

    JeeMQTTServerInfo_t info;
    bzero(&info, sizeof(JeeMQTTServerInfo_t));
    strcpy(info.host, host);
    snprintf(info.userName,64,"%s", userName);
    snprintf(info.password,1024,"R|%s|W|%s",token,token);
    {
        sprintf(info.clientId, "GID_wutong@@@%lld", org_device_id);
        sprintf(info.pubtopic, "iot_client/%lld/1", org_device_id);
        sprintf(info.subtopic, "iot_client/%lld/2", org_device_id);
        sprintf(info.will_info, "\
{\
\"id\":\"666\",\
\"method\":\"onoffline\",\
\"type\":\"rpc\",\
\"jsonrpc\":\"1.2\",\
\"params\":{\
\"clientId\":\"%lld\",\
\"status\":0\
}\
}",
                org_device_id);
    }

    printf("will %s  \n", info.will_info);
    printf("pubtopic %s  \n", info.pubtopic);
    printf("subtopic %s  \n", info.subtopic);
    printf("userName %s  \n", info.userName);


    conn = pxJeeMQTTConnection_Create(&info);
    lJeeMQTTConnect_Register(conn, \
        __vMqttConnectCallback, \
        __vMqttOnlineCallback, \
        __vMqttOfflineCallback);
    conn->cOpen(conn);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    vRegisterDeviceType();

    vTaskDelete(NULL);

    
}
void app_main(void)
{
    printf("Firmware build %s_%s \r\n", __DATE__, __TIME__);
    printf("VERSION %s \r\n", VERSION);

// Initialize NVS.
    jee_err_t ret = JEE_EOK;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wlan_connect_init();
    mount_fatfs(); //文件系统配置

    rt_components_init();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    printf("Wait wlan is connected\r\n");
    while (rt_wlan_is_connected() == JEE_FALSE)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        printf("Try connecting to the network again\r\n");
    }
    printf("Wlan is connected\r\n");
    webclient_post_cloud();

    xTaskCreate(vTestMQTT, "vTestMQTT", 4096*2, NULL, 5, NULL);

}


