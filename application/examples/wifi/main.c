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
#include <hal_wlan_workqueue.h>
#include "drv_wifi_iomodel.h"
#include "esp_netif.h"
#include "hal_wlan_lwip.h"
#include "net_socket.h"
#include "smt_connect.h"



void app_main(void)
{
    wlan_connect_init();
    rt_components_init();

    while(1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

