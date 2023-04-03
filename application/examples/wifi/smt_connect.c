#include <unistd.h>
#include <string.h>
#include <time.h>
#include "jeedef.h"
#include "smt_connect.h"
#include "esp_log.h"
#include "hal_jeeconfig.h"
#include "hal_wlan_mgnt.h"
#include "hal_wlan_prot.h"
#include "hal_wlan_cfg.h"
#include "hal_wlan_dev.h"
#include <lwip/sockets.h>
#include "net_socket.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_wifi.h"
#include "nvs_flash.h"


#define DBG_TAG "smt_connect.c"

#define SOFTAP_SSID "jeejio_test"
#define SOFTAP_PWD "12345678"

static void wifi_scan_report(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_scan_done(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_sta_connected(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_sta_disconnect(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_ap_start(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_ap_stop(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_ap_associted(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_ap_disassocited(int event, struct rt_wlan_buff *buff, void *parameter);
static void example_disp_buf(char* buf, int length);


static void wlan_connect_task(void *param)
{
    int i = 0;
    jee_err_t ret = JEE_EOK;

    //char* ipaddr = "192.168.137.1";
    char *ssid = "Jeejio_TP_2.4G";
    char *password = "88888888";
    char* ipaddr = "10.100.100.17";
    uint8_t recvbuf[50] = {0};
    uint16_t port = 8004;

    if (nvs_flash_init() != ESP_OK)
    {
        LOGE(DBG_TAG, "F:%s L:%d nvs_flash_init fail", __FUNCTION__, __LINE__);
        return;
    }

    /* 注册 WiFi 事件*/
    rt_wlan_register_event_handler(RT_WLAN_EVT_SCAN_REPORT, wifi_scan_report, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_SCAN_DONE, wifi_scan_done, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED, wifi_sta_connected, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED, wifi_sta_disconnect, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_AP_START, wifi_ap_start, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_AP_STOP, wifi_ap_stop, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_AP_ASSOCIATED, wifi_ap_associted, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_AP_DISASSOCIATED, wifi_ap_disassocited, NULL);

#if 1
    /* 设置工作模式*/
    vTaskDelay(500 / portTICK_PERIOD_MS);
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION); //sta模式
    //vTaskDelay(100 / portTICK_PERIOD_MS);

    /* 扫描 WiFi 网络*/
    // rt_wlan_scan_with_info(JEE_NULL);

    /* 网络连接 */
    ret = rt_wlan_connect(ssid, password);
    if (ret != JEE_EOK)
    {
        LOGE("smt_connect", "wlan_connect failed");
    }

    if(rt_wlan_is_connected() == JEE_TRUE)
    {
        LOGI(DBG_TAG, "==[rt_wlan_connect success]===");
    }
    // rt_wlan_disconnect();
#endif

// test ap
// ok
#if 0
    vTaskDelay(400 / portTICK_PERIOD_MS);
    ret = rt_wlan_set_mode(RT_WLAN_DEVICE_AP_NAME, RT_WLAN_AP);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    LOGI(DBG_TAG, "rt_wlan_set_mode ret = %d", ret);

    ret = rt_wlan_start_ap(SOFTAP_SSID, SOFTAP_PWD);
    LOGI(DBG_TAG, "rt_wlan_start_ap ret = %d", ret);

    // vTaskDelay(15000 / portTICK_PERIOD_MS);
    // jee_uint8_t mac[6] = {0x94, 0x87, 0xe0, 0x60, 0x78, 0x2e}; // {0x2e, 0x78, 0x60, 0xe0, 0x87, 0x94};
    // ret = rt_wlan_ap_deauth_sta(mac);
    // LOGI(DBG_TAG, "rt_wlan_ap_deauth_sta ret = %d", ret);

    // vTaskDelay(15000 / portTICK_PERIOD_MS);
    // ret = rt_wlan_ap_stop();
    // LOGI(DBG_TAG, "rt_wlan_ap_stop ret = %d", ret);
#endif

// test promisc
// ok
#if 0
    device = jee_device_find(RT_WLAN_DEVICE_AP_NAME);
    if (device != JEE_NULL)
    {
        LOGI(DBG_TAG, "find device, name:%s", device->name);
    }
    ret = rt_wlan_dev_enter_promisc(device);
    LOGI(DBG_TAG, "rt_wlan_dev_enter_promisc ret = %d", ret);

    vTaskDelay(10000 / portTICK_PERIOD_MS);
    ret = rt_wlan_dev_exit_promisc(device);
    LOGI(DBG_TAG, "rt_wlan_dev_exit_promisc ret = %d", ret);
#endif

// test rssi
// ok
#if 0
    int rssi = rt_wlan_get_rssi();
    LOGI(DBG_TAG, "drv_wlan_get_rssi rssi = %d", rssi);
#endif

// test powersave
// ok
#if 0
    int type = rt_wlan_get_powersave();
    LOGI(DBG_TAG, "rt_wlan_get_powersave type = %d", type);

    ret = rt_wlan_set_powersave(2);
    LOGI(DBG_TAG, "rt_wlan_set_powersave ret = %d", ret);

    type = rt_wlan_get_powersave();
    LOGI(DBG_TAG, "rt_wlan_get_powersave type = %d", type);
#endif

// test channel
// ok
#if 0
    device = jee_device_find(RT_WLAN_DEVICE_AP_NAME);
    if (device != JEE_NULL)
    {
        LOGI(DBG_TAG, "find device, name:%s", device->name);
    }

    int channel = rt_wlan_dev_get_channel(device);
    LOGI(DBG_TAG, "rt_wlan_dev_get_channel channel = %d", channel);

    ret = rt_wlan_dev_set_channel(device, 5);
    LOGI(DBG_TAG, "rt_wlan_dev_set_channel ret = %d", ret);

    channel = rt_wlan_dev_get_channel(device);
    LOGI(DBG_TAG, "rt_wlan_dev_get_channel channel = %d", channel);
#endif

// test mac
// ok
#if 0
    jee_uint8_t mac_set[6] = {0x7c, 0xdf, 0xa3, 0xd4, 0x72, 0xf8};
    jee_uint8_t mac_addr[6] = {0};
    ret = rt_wlan_get_mac(mac_addr);
    LOGI(DBG_TAG, "rt_wlan_get_mac ret = %d", ret);
    LOGI(DBG_TAG, "WIFI mac address: %x %x %x %x %x %x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    ret = rt_wlan_set_mac(mac_set);
    LOGI(DBG_TAG, "rt_wlan_set_mac ret = %d", ret);
    
    ret = rt_wlan_get_mac(mac_addr);
    LOGI(DBG_TAG, "rt_wlan_get_mac ret = %d", ret);
    LOGI(DBG_TAG, "WIFI mac address: %x %x %x %x %x %x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
#endif

// test country
// ok
#if 0
    int country = rt_wlan_ap_get_country();
    LOGI(DBG_TAG, "rt_wlan_ap_get_country country = %d", country);
    ret = rt_wlan_ap_set_country(RT_COUNTRY_CHINA);
    LOGI(DBG_TAG, "rt_wlan_ap_set_country ret = %d", ret);
    country = rt_wlan_ap_get_country();
    LOGI(DBG_TAG, "rt_wlan_ap_get_country country = %d", country);
#endif

    /* udp网络测试 */
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    net_socket socket = net_socket_udp_open(ipaddr, port);

    while(1) 
    {
        
        net_socket_udp_recv(socket, recvbuf, sizeof(recvbuf), 5000, ipaddr, &port);
        LOGI("jeejio", "recvbuf: %s", recvbuf);
    }
}


void wlan_connect_init(void)
{
    // LOGI("smt_connect", "%s =====================================", __func__);
    xTaskCreate(wlan_connect_task, "wlan_connect_task", 4096, NULL, 1, NULL);
}



// 输入参数：event    事件类型
//          buff     查找的WiFi参数
//          parameter 事件注册传入的参数

static void wifi_scan_report(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[enter wifi_scan_report] SSID:%s  RSSI:%d==", ((struct rt_wlan_info *)buff->data)->ssid.val, \
        ((struct rt_wlan_info *)buff->data)->rssi);
}

static void wifi_scan_done(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[enter wifi_scan_done]==\n");
}

static void wifi_sta_connected(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[enter wifi_sta_connected]==");
}

static void wifi_sta_disconnect(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[enter wifi_sta_disconnect]==");
}

static void wifi_ap_start(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[enter wifi_ap_start]==");
}

static void wifi_ap_stop(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[rrrrrrrrr]=======[enter wifi_ap_stop]==");
}

static void wifi_ap_associted(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "==[enter wifi_ap_associted]==");
}

static void wifi_ap_disassocited(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI(DBG_TAG, "====[enter wifi_ap_disassocited]=="); 
}
