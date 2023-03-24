#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/device.h"
#include "esp_log.h"
#include "hal_spi.h"
#include "auto_init.h"
#include "drv_spi_master_iomodel.h"
#include "string.h"

#define TAG "SPI SLAVE"

void app_main(void)
{
    
    rt_components_init();

    esp_spi_bus_attach_device(SPI_BUS_NAME,SPI_DEV_NAME,PIN_NUM_CS);
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);


    struct jee_spi_device *slave_spi;
    uint8_t test_buf[50] = "";

    slave_spi = (struct jee_spi_device *)jee_device_find(SPI_DEV_NAME);

    if (!slave_spi)
    {
        printf("spi sample run failed! can't find spi20 device!");
    }
    else{
        slave_spi->config.mode = JEE_SPI_SLAVE;

        while(1)
        {
            memset(test_buf, 0, 50);
            char test_str[20] = "Hello master!";
            LogInfo(TAG, "send: %s", test_str);
            struct jee_spi_message msg1;
            msg1.send_buf   = test_str;
            msg1.recv_buf   = test_buf;
            msg1.length     = sizeof(test_str);
            msg1.cs_take    = 1;
            msg1.cs_release = 0;

        jee_spi_transfer_message(slave_spi, &msg1);
        LogInfo(TAG, "recv: %s", test_buf);
        vTaskDelay(100);
      }
    }

    
}


