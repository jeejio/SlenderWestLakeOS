#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <spi_common.h>
#include <driver/spi_master.h>
#include "freertos/task.h"
#include "freertos/device.h"
#include "esp_log.h"
#include "hal_spi.h"
#include "auto_init.h"
#include "drv_spi_master_iomodel.h"
#include "string.h"

#define TAG "main"

#define PIN_NUM_MISO    2
#define PIN_NUM_MOSI    7
#define PIN_NUM_CLK     6
#define PIN_NUM_CS      10
#define SPI_BUS_NAME    "spi2"
#define SPI_DEV_NAME    "spi20"
#define MASTER_HOST     SPI2_HOST

    
void app_main(void)
{
    rt_components_init();
    jee_spi_bus_init();
    esp_spi_bus_attach_device(SPI_BUS_NAME,SPI_DEV_NAME,10);
    struct jee_spi_device *master_spi;
    

    printf("master spi test begin\n");
    master_spi = (struct jee_spi_device *)jee_device_find(SPI_DEV_NAME);
    //     spi_bus_config_t bus_cfg={
    //     .miso_io_num = PIN_NUM_MISO,//param->miso_io_num,//PIN_NUM_MISO,
    //     .mosi_io_num = PIN_NUM_MOSI,//param->mosi_io_num,//PIN_NUM_MOSI,
    //     .sclk_io_num = PIN_NUM_CLK,//param->sclk_io_num,//PIN_NUM_CLK,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = 32,
    // };
    spi_bus_device_configuration_t bus_cfg;
    spi_bus_config_t bus1_cfg=SPI_BUS_TEST_DEFAULT_CONFIG();
    spi_device_interface_config_t devcfg=SPI_DEVICE_TEST_DEFAULT_CONFIG();
    bus_cfg.spi_bus = &bus1_cfg;
    bus_cfg.spi_device = &devcfg;

    master_spi->config.pConfig = (void *)(&bus_cfg);
    
    if (!master_spi)
    {
        printf("spi sample run failed! can't find spi20 device!\n");
    }
    else{
        while(1)
        {

            char recvbuf[50] = {0};
            memset(recvbuf, 0, 50);
            char test_str[20] = "Hello slave!";
            LogInfo(TAG, "send: %s", test_str);

            struct jee_spi_message msg1;
            msg1.send_buf   = test_str;
            msg1.recv_buf   = recvbuf;
            msg1.length     = sizeof(test_str);
            msg1.cs_take    = 1;
            msg1.cs_release = 0;
            msg1.next = NULL;
            jee_spi_transfer_message(master_spi,&msg1);

            LogInfo(TAG, "recv: %s", recvbuf);
            vTaskDelay(100);
        } 
    }

    
}


