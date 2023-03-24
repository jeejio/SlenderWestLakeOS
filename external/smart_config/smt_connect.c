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

char *ssid = "yul_test";
char *password = "jeejio123";

static void wifi_sta_disconnect(int event, struct rt_wlan_buff *buff, void *parameter);
static void wifi_sta_connect_fail(int event, struct rt_wlan_buff *buff, void *parameter);

static void wlan_connect_task(void *param)
{
#ifdef UDP_TEST
    int i = 0;
#endif
    jee_err_t ret = JEE_EOK;

    //char* ipaddr = "192.168.137.1";

#ifdef UDP_TEST
    char* ipaddr = "10.100.100.127";
    uint8_t buf[10] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10};
    uint8_t recvbuf[10] = {0};
    uint16_t port = 8004;
#endif
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED, wifi_sta_disconnect, NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED_FAIL, wifi_sta_connect_fail, NULL);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    LOGI("smt_connect", "%s =====================================step 1: set mode start", __func__);
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    LOGI("smt_connect", "%s =====================================step 1: set mode finished", __func__);
    LOGI("smt_connect", "%s =====================================step 2: connect start", __func__);
    ret = rt_wlan_connect(ssid, password);
    if (ret == JEE_EOK)
    {
        LOGI("smt_connect", "wlan_connect success");
    }
    else
    {
        LOGE("smt_connect", "wlan_connect failed");
        //return;
    }
    LOGI("smt_connect", "%s =====================================step 2: connect finished", __func__);
#ifdef UDP_TEST
    LOGI("smt_connect", "%s =====================================step 3: udp test start", __func__);
    net_socket socket = net_socket_udp_open(ipaddr, port);
#endif

    while(1) {
#ifdef UDP_TEST
        net_socket_udp_recv(socket, recvbuf, 10, 5000, ipaddr, &port);
        for(int i = 0; i < 10; i++)
        {
            LOGI("smt_connect", "test 0x%x", recvbuf[i]);
        }
#endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void wlan_connect_init(void)
{
    LOGI("smt_connect", "%s =====================================", __func__);
    xTaskCreate(wlan_connect_task, "wlan_connect_task", 4096, NULL, 1, NULL);
}

static void wifi_sta_disconnect(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI("smt_connect", "==[enter wifi_sta_disconnect]==");
    printf("wifi_sta_disconnect+++++\r\n");

    // rt_wlan_connect(ssid, password);
    esp_wifi_connect(); //TODO:因为rt_wlan_connect会重复调用start，有内存泄露，暂时先用esp的接口规避，后期冯坤修正
}

static void wifi_sta_connect_fail(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOGI("smt_connect", "==[enter connect fail]==");
    printf("wifi_sta_connect_fail++++\r\n");
    // rt_wlan_connect(ssid, password);
    esp_wifi_connect(); //TODO:因为rt_wlan_connect会重复调用start，有内存泄露，暂时先用esp的接口规避，后期冯坤修正
}