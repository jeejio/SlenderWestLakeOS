/*
 * Copyright (c) 2019 Winner Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-15     flyingcys    1st version
 */

//#include <rtthread.h>
//#include <wlan_dev.h>
//#include "wm_type_def.h"
//#include "wm_wifi.h
#include <string.h>
#include "drv_wifi_iomodel.h"
#include "esp_wifi_types.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <jeedef.h>
#include "hal_wlan_dev.h"
#include "hal_jeeconfig.h"
#include "auto_init.h"
#include "hal_wlan_mgnt.h"
#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <ethernetif_port.h>
#include <esp_netif_net_stack.h>
#include <io_dev_mgt.h>
#include "hal_wlan_lwip.h"
#include "lwip/esp_netif_net_stack.h"
 
#define DBG_ENABLE
#define DBG_LEVEL DBG_INFO
#define DBG_SECTION_NAME  "WIFI"
#define DBG_COLOR
static const char *TAG = "drv_wifi_iomodel";

#define MAX_ADDR_LEN        (6)

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif

struct drv_wifi
{
    struct rt_wlan_device *wlan;
    jee_uint8_t dev_addr[MAX_ADDR_LEN];
};

static struct drv_wifi wifi_sta;
static struct drv_wifi wifi_ap;

static char country_table[RT_COUNTRY_UNKNOWN + 1][3] = {
    "AF", 
    "AL",
    "DZ",
    "AS",
    "AO",
    "AI",
    "AG",
    "AR",
    "AM",
    "AW",
    "AU",
    "AU",
    "AZ",
    "BS",
    "BH",
    "XX", // RT_COUNTRY_BAKER_ISLAND
    "BD",
    "BB",
    "BY",
    "BE",
    "BZ",
    "BJ",
    "BM",
    "BT",
    "BO",
    "BA",
    "BW",
    "BR",
    "IO",
    "BN",
    "BG",
    "BF",
    "BI",
    "KH",
    "CM",
    "CA",
    "CV",
    "KY",
    "CF",
    "TD",
    "CL",
    "CN",
    "CX",
    "CO",
    "KM",
    "CG",
    "CD",
    "CR",
    "CI",
    "HR",
    "CU",
    "CY",
    "CZ",
    "DK",
    "DJ",
    "DM",
    "DO",
    "XX", // case RT_COUNTRY_DOWN_UNDER:
    "EC",
    "EG",
    "SV",
    "GQ",
    "ER",
    "EE",
    "ET",
    "FK",
    "FO",
    "FJ",
    "FI",
    "FR",
    "GF",
    "PF",
    "TF",
    "GA",
    "GM",
    "GE",
    "DE",
    "GH",
    "GI",
    "GR",
    "GD",
    "GP",
    "GU",
    "GT",
    "GG",
    "GN",
    "GW",
    "GY",
    "HT",
    "VA",
    "HN",
    "HK",
    "HU",
    "IS",
    "IN",
    "ID",
    "IR",
    "IQ",
    "IE",
    "IL",
    "IT",
    "JM",
    "JP",
    "JE",
    "JO",
    "KZ",
    "KE",
    "KI",
    "KR",
    "XK",
    "KW",
    "KG",
    "LA",
    "LV",
    "LB",
    "LS",
    "LR",
    "LY",
    "FL",
    "LT",
    "LU",
    "MO",
    "MK",
    "MG",
    "MW",
    "MY",
    "MV",
    "ML",
    "MT",
    "IM",
    "MQ",
    "MR",
    "MU",
    "YT",
    "MX",
    "FM",
    "MD",
    "MC",
    "MN",
    "ME",
    "MS",
    "MA",
    "MZ",
    "MM",
    "NA",
    "NR",
    "NP",
    "NL",
    "AN",
    "NC",
    "NZ",
    "NI",
    "NE",
    "NG",
    "NF",
    "MP",
    "NO",
    "OM",
    "PK",
    "PW",
    "PA",
    "PG",
    "PY",
    "PE",
    "PH",
    "PL",
    "PT",
    "US",
    "QA",
    "RE",
    "RO",
    "RU",
    "RW",
    "KN",
    "LC",
    "PM",
    "VC",
    "WS",
    "XX", // RT_COUNTRY_SANIT_MARTIN_SINT_MARTEEN
    "ST",
    "SA",
    "SN",
    "RS",
    "SC",
    "SL",
    "SG",
    "SK",
    "SI",
    "SB",
    "SO",
    "ZA",
    "ES",
    "LK",
    "SR",
    "SZ",
    "SE",
    "CH",
    "SY",
    "TW",
    "TJ",
    "TZ",
    "TH",
    "TG",
    "TO",
    "TT",
    "TN",
    "TR",
    "TM",
    "TC",
    "TV",
    "UG",
    "UA",
    "AE",
    "GB",
    "US",
    "XX", // RT_COUNTRY_UNITED_STATES_REV4:
    "XX", // RT_COUNTRY_UNITED_STATES_NO_DFS
    "UM",
    "UY",
    "UZ",
    "VU",
    "VE",
    "VN",
    "VG",
    "VI",
    "WF",
    "PS", // RT_COUNTRY_WEST_BANK 约旦河西岸，属于巴勒斯坦
    "EH",
    "XX", // RT_COUNTRY_WORLD_WIDE_XX
    "YE",
    "ZM",
    "ZW",
    "XX"  // RT_COUNTRY_UNKNOWN
};


/**
 * @brief debug buffer data
 */
void example_disp_buf(char* title, char* buf, int length)
{
    printf("\n========[%s]===========\n", title);
    for (int i = 0; i < length; i++) 
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 32 == 0) 
        {
            printf("\n");
        }
    }
    printf("\n===========================\n");
}


static void rt_country_to_esp_country(rt_country_code_t country_code, char *country)
{
    if (country_code >= RT_COUNTRY_AFGHANISTAN && country_code <= RT_COUNTRY_UNKNOWN)
    {
        memcpy(country, country_table[country_code], 2);
    }
}

static void esp_country_to_rt_country(const char *country, rt_country_code_t *country_code)
{
    for (int i = RT_COUNTRY_AFGHANISTAN; i <= RT_COUNTRY_UNKNOWN; i++)
    {
        if (!strncmp(country, country_table[i], 2))
        {
            *country_code = i;
        }
    }
}

#if defined(ETH_RX_DUMP) ||  defined(ETH_TX_DUMP)
static void packet_dump(const char *msg, const void *ptr, jee_uint32_t len)
{
    jee_uint32_t j;
    jee_uint8_t *p = (jee_uint8_t *)ptr;
    rt_kprintf("%s %d byte\n", msg, len);

#ifdef MINI_DUMP
    return;
#endif

    for (j = 0; j < len; j++)
    {
        if ((j % 8) == 0)
        {
            rt_kprintf("  ");
        }

        if ((j % 16) == 0)
        {
            rt_kprintf("\r\n");
        }
        rt_kprintf("%02x ", *p ++);
    }

    rt_kprintf("\n\n");
}
#endif /* dump */

static void wm_wlan_client_event(jee_uint8_t *mac, wifi_event_t event)
{
    struct rt_wlan_buff buff;
    struct rt_wlan_info sta;
    memcpy(sta.bssid, mac, MAX_ADDR_LEN);
    buff.data = &sta;
    buff.len = sizeof(sta);

    if (WIFI_EVENT_AP_STACONNECTED == event)
    {
        rt_wlan_dev_indicate_event_handle(wifi_ap.wlan, RT_WLAN_DEV_EVT_AP_ASSOCIATED, &buff);
    }
    if (WIFI_EVENT_AP_STADISCONNECTED == event)
    {
        rt_wlan_dev_indicate_event_handle(wifi_ap.wlan, RT_WLAN_DEV_EVT_AP_DISASSOCIATED, &buff);
    }
}

static void wm_wlan_promisc_dataframe_callback(struct rt_wlan_device *device, void *data, int len)
{
    rt_wlan_dev_promisc_handler(device, data, len);
}

static void esp_promisc_dataframe_callback(void *buf, wifi_promiscuous_pkt_type_t type)
{
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    wm_wlan_promisc_dataframe_callback(wifi_ap.wlan, (void *)(pkt->payload), pkt->rx_ctrl.sig_len);
}

static void wm_wlan_scan_callback(void)
{
    struct rt_wlan_info wlan_info;
    struct rt_wlan_buff buff;
    wifi_ap_record_t *ap_list_buffer;
    uint16_t ap_count = 0;

    if (esp_wifi_scan_get_ap_num(&ap_count) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get scan ap num");
        return;
    }
    LOGI(TAG, "wm_wlan_scan_callback esp_wifi_scan_get_ap_num ap_count = %d", ap_count);

    ap_list_buffer = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_list_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to malloc buffer to print scan results");
        return;
    }
    else
    {
        memset(ap_list_buffer, 0, sizeof(ap_count * sizeof(wifi_ap_record_t)));
    }

    if (esp_wifi_scan_get_ap_records(&ap_count, ap_list_buffer) == ESP_OK) {
        for (int i = 0; i < ap_count; i++) {
            // ESP_LOGI(TAG, "[%s][rssi=%d]", ap_list_buffer[i].ssid, ap_list_buffer[i].rssi);

            memset(&wlan_info, 0, sizeof(wlan_info));

            memcpy(&wlan_info.bssid[0], ap_list_buffer[i].bssid, 6);
            memcpy(wlan_info.ssid.val, ap_list_buffer[i].ssid, strlen((char*)(ap_list_buffer[i].ssid)));
            wlan_info.ssid.len = strlen((char*)(ap_list_buffer[i].ssid));
            if (wlan_info.ssid.len)
                wlan_info.hidden = 0;
            else
                wlan_info.hidden = 1;

            wlan_info.channel = (jee_int16_t)(ap_list_buffer[i].primary); // ap channel
            wlan_info.rssi = -(char)(0x100 - ap_list_buffer[i].rssi);

            //wlan_info.datarate = bss_info->max_data_rate * 1000000;

            wlan_info.band = RT_802_11_BAND_2_4GHZ;

            wlan_info.security = SECURITY_OPEN;
            if (WIFI_AUTH_WEP & ap_list_buffer[i].authmode)
                wlan_info.security |= WEP_ENABLED;
            if (WIFI_AUTH_WPA_PSK & ap_list_buffer[i].authmode)
                wlan_info.security |= WPA_SECURITY | TKIP_ENABLED | AES_ENABLED;
            if (WIFI_AUTH_WPA2_PSK & ap_list_buffer[i].authmode)
                wlan_info.security |= WPA2_SECURITY | TKIP_ENABLED | AES_ENABLED;
            if (WIFI_AUTH_WPA_WPA2_PSK & ap_list_buffer[i].authmode)
                wlan_info.security |= WPA_SECURITY | WPA2_SECURITY | TKIP_ENABLED | AES_ENABLED;
            if (ap_list_buffer[i].wps)
                wlan_info.security |= WPS_ENABLED;
            if (WIFI_AUTH_MAX == ap_list_buffer[i].authmode)
                wlan_info.security = SECURITY_UNKNOWN;

            /* rtt incompleted... */
            if (wlan_info.security & SECURITY_WPA2_MIXED_PSK)
                wlan_info.security = SECURITY_WPA2_MIXED_PSK;
            else if (wlan_info.security & SECURITY_WPA2_TKIP_PSK)
                wlan_info.security = SECURITY_WPA2_TKIP_PSK;
            else if (wlan_info.security & SECURITY_WPA2_AES_PSK)
                wlan_info.security = SECURITY_WPA2_AES_PSK;
            else if (wlan_info.security & SECURITY_WPA_AES_PSK)
                wlan_info.security = SECURITY_WPA_AES_PSK;
            else if (wlan_info.security & SECURITY_WPA_TKIP_PSK)
                wlan_info.security = SECURITY_WPA_TKIP_PSK;
            else if (wlan_info.security & SECURITY_WEP_PSK)
                wlan_info.security = SECURITY_WEP_PSK;
            else if ((SECURITY_UNKNOWN == wlan_info.security) && ap_list_buffer[i].wps)
                wlan_info.security = SECURITY_WPS_SECURE;

            // LOGI(TAG, "%s-%x", wlan_info.ssid.val, wlan_info.security);

            buff.data = &wlan_info;
            buff.len = sizeof(wlan_info);
            rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_SCAN_REPORT, &buff);
        }
    }
    free(ap_list_buffer);

    rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_SCAN_DONE, NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    // if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    // {
    //     esp_wifi_connect();
    // }
    // else 
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        LOGI(TAG,"get WIFI_EVENT_STA_CONNECTED event");
        rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_CONNECT, NULL);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        rt_wlan_dev_indicate_event_handle(wifi_sta.wlan, RT_WLAN_DEV_EVT_DISCONNECT, NULL);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        LOGI(TAG,"get WIFI_EVENT_SCAN_DONE event");
        wm_wlan_scan_callback();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
    else if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        LOGI(TAG, "station %02x:%02x:%02x:%02x:%02x:%02x join, AID=%d", MAC2STR(event->mac), event->aid);
        wm_wlan_client_event(event->mac, WIFI_EVENT_AP_STACONNECTED);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        LOGI(TAG, "station %02x:%02x:%02x:%02x:%02x:%02x leave, AID=%d", MAC2STR(event->mac), event->aid);
        wm_wlan_client_event(event->mac, WIFI_EVENT_AP_STADISCONNECTED);
    }
    else if (event_id == WIFI_EVENT_AP_START)
    {
        rt_wlan_dev_indicate_event_handle(wifi_ap.wlan, RT_WLAN_DEV_EVT_AP_START, NULL);
    }
    else if (event_id == WIFI_EVENT_AP_STOP)
    {
        rt_wlan_dev_indicate_event_handle(wifi_ap.wlan, RT_WLAN_DEV_EVT_AP_STOP, NULL);
    }
}

static jee_err_t drv_wlan_init(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "============[enter drv_wlan_init]==================== wlan->mode = %d", wlan->mode);
    esp_err_t ret = ESP_OK;

    if (esp_netif_init() != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_netif_init fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    if (esp_event_loop_create_default() != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_event_loop_create_default fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    if (wlan->mode == RT_WLAN_STATION)
    {
        esp_netif_create_default_wifi_sta();
        LOGI(TAG, "drv_wlan_init esp_netif_create_default_wifi_sta");
    }
    else if (wlan->mode == RT_WLAN_AP)
    {
        esp_netif_create_default_wifi_ap();
        LOGI(TAG, "drv_wlan_init esp_netif_create_default_wifi_ap");
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_wifi_init fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    LOGI(TAG, "drv_wlan_init ESP_EVENT_ANY_ID ret = %d", ret);
    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);
    LOGI(TAG, "drv_wlan_init IP_EVENT_STA_GOT_IP ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_mode(struct rt_wlan_device *wlan, rt_wlan_mode_t mode)
{
    wifi_mode_t wifiMode;

    switch (mode)
    {
        case RT_WLAN_NONE:
            wifiMode = WIFI_MODE_NULL;
            break;
        case RT_WLAN_STATION:
            wifiMode = WIFI_MODE_STA;
            break;
        case RT_WLAN_AP:
            wifiMode = WIFI_MODE_AP;
            break;
        case RT_WLAN_MODE_MAX:
            wifiMode = WIFI_MODE_MAX;
            break;
        default:
            wifiMode = WIFI_MODE_NULL;
            break;
    }

    esp_err_t ret = esp_wifi_set_mode(wifiMode);
    LOGI(TAG, "drv_wlan_mode esp_wifi_set_mode ret = %d, wifiMode = %d", ret, wifiMode);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_scan(struct rt_wlan_device *wlan, struct rt_scan_info *scan_info)
{
    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_wifi_start fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    ret = esp_wifi_scan_start(NULL, true);
    LOGI(TAG, "drv_wlan_scan esp_wifi_scan_start ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_join(struct rt_wlan_device *wlan, struct rt_sta_info *sta_info)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);
 
    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config_t));
    
    if (wlan->mode == RT_WLAN_STATION)
    {
        wifi_config.sta.bssid_set = 0;
        memcpy(wifi_config.sta.ssid, sta_info->ssid.val, sta_info->ssid.len);
        memcpy(wifi_config.sta.password, sta_info->key.val, sta_info->key.len);
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    }
    else
    {
        LOGE(TAG, "F:%s L:%d wlan mode not sta", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_wifi_set_config fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_wifi_start fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    ret = esp_wifi_connect();
    LOGI(TAG, "drv_wlan_join esp_wifi_connect ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_softap(struct rt_wlan_device *wlan, struct rt_ap_info *ap_info)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));
    memcpy(wifi_config.ap.ssid, ap_info->ssid.val, ap_info->ssid.len);
    memcpy(wifi_config.ap.password, ap_info->key.val, ap_info->key.len);
    wifi_config.ap.ssid_len = ap_info->ssid.len;
    wifi_config.ap.max_connection = 2;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.channel = ap_info->channel;
    wifi_config.ap.beacon_interval = 100;

    if (ap_info->key.len == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_wifi_set_config fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    ret = esp_wifi_start();
    LOGI(TAG, "drv_wlan_softap esp_wifi_start ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_disconnect(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    esp_err_t ret = esp_wifi_disconnect();
    LOGI(TAG, "drv_wlan_disconnect esp_wifi_disconnect ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_ap_stop(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);
    esp_err_t ret = esp_wifi_stop();
    LOGI(TAG, "drv_wlan_ap_stop esp_wifi_stop ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_ap_deauth(struct rt_wlan_device *wlan, jee_uint8_t mac[])
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);
    uint16_t aid = 0;

    esp_err_t ret = esp_wifi_ap_get_sta_aid(mac, &aid);
    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d esp_wifi_ap_get_sta_aid fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    ret = esp_wifi_deauth_sta(aid);
    LOGI(TAG, "drv_wlan_ap_deauth esp_wifi_deauth_sta ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_scan_stop(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    esp_err_t ret = esp_wifi_scan_stop();
    LOGI(TAG, "drv_wlan_scan_stop esp_wifi_scan_stop ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static int drv_wlan_get_rssi(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    wifi_ap_record_t ap_records;
    memset(&ap_records, 0, sizeof(wifi_ap_record_t));

    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_records);
    LOGI(TAG, "lk SSID \t\t%s", ap_records.ssid);
    LOGI(TAG, "lk RSSI \t\t%d", ap_records.rssi);

    return -ap_records.rssi - 1;
}

static jee_err_t drv_wlan_set_powersave(struct rt_wlan_device *wlan, int level)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    wifi_ps_type_t type;

    if (level == 2)
    {
        type = WIFI_PS_MAX_MODEM;
    }
    else if (level == 1)
    {
        type = WIFI_PS_MIN_MODEM;
    }
    else
    {
        type = WIFI_PS_NONE;
    }

    esp_err_t ret = esp_wifi_set_ps(type);
    LOGI(TAG, "drv_wlan_set_powersave esp_wifi_set_ps ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static int drv_wlan_get_powersave(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    wifi_ps_type_t type = WIFI_PS_NONE;

    esp_err_t ret = esp_wifi_get_ps(&type);
    LOGI(TAG, "drv_wlan_get_powersave esp_wifi_get_ps ret = %d", ret);

    return (int)type;
}

static jee_err_t drv_wlan_cfg_promisc(struct rt_wlan_device *wlan, jee_bool_t start)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    esp_err_t ret = esp_wifi_set_promiscuous(start);
    LOGI(TAG, "drv_wlan_cfg_promisc esp_wifi_set_promiscuous ret = %d", ret);

    if (ret != ESP_OK)
    {
        LOGE(TAG, "F:%s L:%d fail", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    if (JEE_TRUE == start)
    {
        ret = esp_wifi_set_promiscuous_rx_cb(esp_promisc_dataframe_callback);
    }
    else
    {
        ret = esp_wifi_set_promiscuous_rx_cb(NULL);
    }

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_cfg_filter(struct rt_wlan_device *wlan, struct rt_wlan_filter *filter)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    wifi_promiscuous_filter_t proFilter;
    esp_wifi_set_promiscuous_filter(&proFilter);

    return JEE_EINVAL;/* not support */
}

static jee_err_t drv_wlan_set_channel(struct rt_wlan_device *wlan, int channel)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    esp_err_t ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    LOGI(TAG, "drv_wlan_set_channel esp_wifi_set_channel ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static int drv_wlan_get_channel(struct rt_wlan_device *wlan)
{
    LOGI(TAG, "F:%s L:%d", __FUNCTION__, __LINE__);

    uint8_t primary;
    wifi_second_chan_t second;
    esp_err_t ret = esp_wifi_get_channel(&primary, &second);
    LOGI(TAG, "drv_wlan_get_channel esp_wifi_get_channel ret = %d", ret);

    return primary;
}

static jee_err_t drv_wlan_set_country(struct rt_wlan_device *wlan, rt_country_code_t country_code)
{
    char country[3] = {0};

    rt_country_to_esp_country(country_code, country);

    esp_err_t ret = esp_wifi_set_country_code(country, false);
    LOGI(TAG, "drv_wlan_set_country esp_wifi_set_country_code ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static rt_country_code_t drv_wlan_get_country(struct rt_wlan_device *wlan)
{
    rt_country_code_t country_code = RT_COUNTRY_UNKNOWN;
    char country[3] = {0};

    esp_err_t ret = esp_wifi_get_country_code(country);
    LOGI(TAG, "drv_wlan_get_country esp_wifi_get_country_code ret = %d, country = %s", ret, country);

    esp_country_to_rt_country(country, &country_code);
    LOGI(TAG, "drv_wlan_get_country country_code = %d", country_code);

    return country_code;
}

static jee_err_t drv_wlan_set_mac(struct rt_wlan_device *wlan, jee_uint8_t mac[])
{
    if (wlan->mode != RT_WLAN_STATION)
    {
        LOGE(TAG, "%s %d wlan mode not sta", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    esp_err_t ret = esp_wifi_set_mac(WIFI_IF_STA, mac);
    LOGI(TAG, "drv_wlan_set_mac esp_wifi_set_mac ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_get_mac(struct rt_wlan_device *wlan, jee_uint8_t mac[])
{
    if (wlan->mode != RT_WLAN_STATION)
    {
        LOGE(TAG, "%s %d wlan mode not sta", __FUNCTION__, __LINE__);
        return JEE_ERROR;
    }

    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    LOGI(TAG, "drv_wlan_get_mac esp_wifi_get_mac ret = %d", ret);

    return ret == ESP_OK ? JEE_EOK : JEE_ERROR;
}

static jee_err_t drv_wlan_low_level_output(struct netif *netif, struct pbuf *p)
{
    // ESP_LOGI("low_level_output", "=========[dddddd]=====[payload:%s]=====", p->payload);

    // example_disp_buf(p->payload, 40);

    esp_netif_t *esp_netif = netif->state;
    if (esp_netif == NULL) {
        return JEE_ERROR;
    }

    struct pbuf *q = p;
    esp_err_t ret;

    if(q->next == NULL) {
        ret = esp_netif_transmit_wrap(esp_netif, q->payload, q->len, q);

    } else {
        LWIP_DEBUGF(PBUF_DEBUG, ("low_level_output: pbuf is a list, application may has bug"));
        q = pbuf_alloc(PBUF_RAW_TX, p->tot_len, PBUF_RAM);
        if (q != NULL) {
            pbuf_copy(q, p);
        } else {
            return ERR_MEM;
        }
        ret = esp_netif_transmit_wrap(esp_netif, q->payload, q->len, q);

        pbuf_free(q);
    }

    if (ret == ESP_OK) {
        return JEE_EOK;
    }
    if (ret == ESP_ERR_NO_MEM) {
        return JEE_ENOMEM;
    }
    if (ret == ESP_ERR_INVALID_ARG) {
        return JEE_EINVAL;
    }
    return JEE_ERROR;
}

static int drv_wlan_recv(struct rt_wlan_device *wlan, void *buff, int len)
{
    return JEE_EOK;
}

static int drv_wlan_send(struct rt_wlan_device *wlan, void *buff, int len)
{
#if 0
    struct jee_device *device = jee_container_of(&wlan, struct jee_device, (struct rt_wlan_device *)userdata);
    struct eth_device *eth = jee_container_of(device, struct eth_device, parent);
    struct pbuf *p = jee_container_of(&buff, struct pbuf, payload);
#else
    struct lwip_prot_des *lwip_prot = jee_container_of(((struct rt_wlan_prot *)(wlan->prot)), struct lwip_prot_des, prot);
    struct pbuf *p = jee_container_of(&buff, struct pbuf, payload);
#endif
    return drv_wlan_low_level_output(lwip_prot->eth.netif, p);
}



static jee_err_t wm_wlan_drv_input(struct rt_wlan_device *wlan, void *args)
{
    struct wlan_trans_input_func_args *wiargs = (struct wlan_trans_input_func_args *)args;

    wlanif_input(wiargs->wlanif, wiargs->buffer, wiargs->len, wiargs->l2_buff);
    return JEE_EOK;
}

static jee_err_t wm_wlan_drv_output(struct rt_wlan_device *wlan, void *args)
{
    struct wlan_trans_output_func_args *woargs = (struct wlan_trans_output_func_args *)args;

    wlanif_output(woargs->wlanif, woargs->pbuf);
    return JEE_EOK;
}


const struct rt_wlan_dev_ops ops =
{
    .wlan_init              = drv_wlan_init,
    .wlan_mode              = drv_wlan_mode,
    .wlan_scan              = drv_wlan_scan,
    .wlan_join              = drv_wlan_join,
    .wlan_softap            = drv_wlan_softap,
    .wlan_disconnect        = drv_wlan_disconnect,
    .wlan_ap_stop           = drv_wlan_ap_stop,
    .wlan_ap_deauth         = drv_wlan_ap_deauth,
    .wlan_scan_stop         = drv_wlan_scan_stop,
    .wlan_get_rssi          = drv_wlan_get_rssi,
    .wlan_set_powersave     = drv_wlan_set_powersave,
    .wlan_get_powersave     = drv_wlan_get_powersave,
    .wlan_cfg_promisc       = drv_wlan_cfg_promisc,
    .wlan_cfg_filter        = drv_wlan_cfg_filter,
    .wlan_set_channel       = drv_wlan_set_channel,
    .wlan_get_channel       = drv_wlan_get_channel,
    .wlan_set_country       = drv_wlan_set_country,
    .wlan_get_country       = drv_wlan_get_country,
    .wlan_set_mac           = drv_wlan_set_mac,
    .wlan_get_mac           = drv_wlan_get_mac,
    .wlan_recv              = drv_wlan_recv,
    .wlan_send              = drv_wlan_send,
    .wlan_drv_input      = wm_wlan_drv_input,
    .wlan_drv_output     = wm_wlan_drv_output,
};


int wm_hw_wifi_init(void)
{
    static struct rt_wlan_device wlan;
    static struct rt_wlan_device wlan2;

    memset(&wifi_sta, 0, sizeof(wifi_sta));
    jee_err_t ret = rt_wlan_dev_register(&wlan, RT_WLAN_DEVICE_STA_NAME, &ops, 0, &wifi_sta);
    wifi_sta.wlan = &wlan;

    memset(&wifi_ap, 0, sizeof(wifi_ap));
    ret |= rt_wlan_dev_register(&wlan2, RT_WLAN_DEVICE_AP_NAME, &ops, 0, &wifi_ap);
    wifi_ap.wlan = &wlan2;

    return ret; //RT_EOK;
}
INIT_PREV_EXPORT(wm_hw_wifi_init);
