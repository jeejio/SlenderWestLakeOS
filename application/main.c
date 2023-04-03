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

/*  Fatfs I/O Example
    int a[5] = {1, 2, 3, 4, 5};
    mount_fatfs();
    FILE *f1 = fopen("/data/test.bin", "wb");
    if (f1 == NULL) {
        printf("Failed to open file for writing\n");
        return;
    }
    size_t ret_w = fwrite(a, sizeof(a[0]), 5, f1);
    printf("wrote %zu elements out of %d requested\n", ret_w,  5);
    fclose(f1);

    int b[5];
    FILE *f2 = fopen("/data/test.bin", "rb");
    if (f2 == NULL) {
        printf("Failed to open file for reading\n");
        return;
    }
    size_t ret_r = fread(b, sizeof(b[0]), 5, f2);
    fclose(f2);
    printf("read back: ");
    for(size_t i = 0; i < ret_r; i++) {
        printf("%d ", b[i]);
    }
*/

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
    // FIXME 杩欎釜鎺ュ彛闇€瑕侀噸鏋勩€俁eportOnLine 鎺ュ彛鏄€氱敤鎺ュ彛锛屼笉搴斿綋鎼哄甫 devType 鍙傛暟銆?
    // devType 鏄暀鑲茬嚎涓撴湁鍙傛暟銆?
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
    __vreport_online();
}

static void __vMqttOfflineCallback(void *c)
{
    printf("inter mqtt_offline_callback!\r\n");
}

static void vTestMQTT(void)
{
    const char *host = "tcp://mqtt-cn-zpr30s05k03.mqtt.aliyuncs.com:1883";
    const char *userName = "Token|LTAI5t8JjkjmcH8RZps1xx2S|mqtt-cn-zpr30s05k03";
    // const char *password = "jeejio123";

    int8_t ret = 0;
    printf("wait get  token  \r\n");

    xSemaphoreTake(cloudMutex, portMAX_DELAY);
    // printf("get token  %s\r\n", token);

    do
    {
        ret = lGetIntDeviceIDFromFlash(&org_device_id);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } while (ret < 0);

    printf("device id   %lld \r\n",org_device_id );
    devType = lGetDeviceType();
    if (-1 == devType)
    {
        devType = 0;
        lSetDeviceType(devType);
    }
#if (DEV == 1)

#else
//chang device type by user
     devType = SINGLE_SWITCH_ID;
#endif
    printf("Get device type:%d\r\n", devType);
    // vRegisterDeviceType();

    if (lRegister_DevApiProcess(JeeRPC_Call))
    {
        printf("Register device api process failed\r\n");
    }

    JeeMQTTServerInfo_t info;
    bzero(&info, sizeof(JeeMQTTServerInfo_t));
    strcpy(info.host, host);
    // strcpy(info.userName, userName);
    // strcpy(info.password, password);
    snprintf(info.userName,64,"%s", userName);
    snprintf(info.password,1024,"R|%s|W|%s",token,token);
    {
        sprintf(info.clientId, "GID_wutong@@@%lld", org_device_id);
#if (DEV == 1)
        sprintf(info.pubtopic, "iot_server/%lld", org_device_id);
        sprintf(info.subtopic, "iot_client/%lld", org_device_id);
#else
        sprintf(info.pubtopic, "iot_client/%lld/1", org_device_id);
        sprintf(info.subtopic, "iot_client/%lld/2", org_device_id);

#endif
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
    // printf("password %s  \n", info.password);


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

static void __vReadKeyPartition(void)
{
    size_t offset = 0;
    char buf[128];
    const esp_partition_t *partition = esp_partition_find_first( \
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "jee_key");
    if (!partition)
    {
        printf("KEY PARTITION NOT FOUND!!\n");
        return;
    }
    memset(buf, 0xFF, sizeof(buf));
    esp_err_t ret = esp_partition_read(partition, offset, buf, sizeof(buf));
    if (ESP_OK != ret)
    {
        printf("Read from key partition failed: %d\n", ret);
        return;
    }

    int i;
    printf("Key partition==========\n");
    for (i = 0; i < sizeof(buf); i++)
    {
        printf("%02X ", buf[i]);
        if (i && !((i+1) & 0x0F))
        {
            printf("\n");
        }
    }
    printf("\n");
}

#if 1
void app_main(void)
{
    printf("Firmware build %s_%s \r\n", __DATE__, __TIME__);
    printf("VERSION %s \r\n", VERSION);

#if 1
// Initialize NVS.
    jee_err_t ret = JEE_EOK;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    __vReadKeyPartition();

    wlan_connect_init();
    mount_fatfs(); //鏂囦欢绯荤粺閰嶇疆

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
//     // vTestMQTT();

  xTaskCreate(vTestMQTT, "vTestMQTT", 4096*2, NULL, 5, NULL);

#else
    mount_fatfs(); //鏂囦欢绯荤粺閰嶇疆
    rt_components_init(); //xuyuhu: test sensor
    factory_test_init();  //xuyuhu
#endif
}
#else
void app_main(void)
{
    jee_device_t device = JEE_NULL;
    struct rt_wlan_scan_result result = {0};
    struct rt_wlan_info wlan_info;
    int i = 0;
    jee_err_t ret = JEE_EOK;

    char *ipaddr = "192.168.137.1";
    uint8_t buf[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};
    uint8_t recvbuf[10] = {0};
    uint16_t port = 8004;

    wlan_connect_init();
    rt_components_init();

    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    // device = jee_device_find(RT_WLAN_DEVICE_STA_NAME);
    // if (device != JEE_NULL)
    // {
    //     LOGI(DBG_TAG, "find device, name:%s", device->name);
    // }
#if 0
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);

    ret = rt_wlan_connect(ssid, password);
    if (ret == JEE_EOK)
    {
        LOGI(DBG_TAG, "wlan_connect success");
    }
#endif
    // net_socket socket = net_socket_udp_open(ipaddr, 8004);
    // net_socket_udp_recv(socket, recvbuf, 10, 5000, ipaddr, &port);
    // for(int i = 1; i < 10; i++)
    // {
    //     LOGI("jeejio", "test 0x%x", recvbuf[i]);
    // }

    vTalSteeringGearInit();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // while(1)
    // {
    //     LOGI(DBG_TAG, "=====");
    //     vTalSteeringGearSetRotationAngle(20);
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    //     vTalSteeringGearSetRotationAngle(0);
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    // }

    int angle = 0;
    int dirction = 1;
    while (1)
    {
        /* code */
        angle = angle + 10;
        vTalSteeringGearSetRotationAngle(angle);
        if (angle == 180)
        {
            /*鏀瑰彉鏂瑰悜*/
            dirction = lTalSteeringGearGetRotationDirction();
            vTalSteeringGearSetRotationDirction(1 - dirction);
            angle = 0;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif
