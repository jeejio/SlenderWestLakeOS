#include <stdio.h>
#include "esp32c3/rom/ets_sys.h" //for ets_delay_us()
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "hal_spi.h"
#include "hal_sensor.h"
#include "drv_spi_master_iomodel.h"
#include "st7789.h"
#include "auto_init.h"
#define LCD_DC (2)
#define LCD_RST (3)
#define LCD_CS (21)
#define LCD_MOSI (8)
#define LCD_MISO (-1)
#define LCD_SCLK (9)
#define SPI_DEV_NAME "spi20"

struct jee_spi_device *master_spi;

static uint16_t point_color = 0xFFFF;

void vWriteCommand(uint8_t cmd)
{
    struct jee_spi_message msg;
    uint8_t buff = cmd;
    msg.cs_release = 0;
    msg.cs_take = 1;
    msg.send_buf = &buff;
    msg.recv_buf = NULL;
    msg.next = NULL;
    msg.length = 1;
    jee_pin_write(LCD_CS, 0);
    jee_pin_write(LCD_DC, 0);
    jee_spi_transfer_message(master_spi, &msg);
    jee_pin_write(LCD_CS, 1);
}

void vWriteData(uint8_t *data, int len)
{
    struct jee_spi_message msg;
    msg.cs_release = 0;
    msg.cs_take = 1;
    msg.send_buf = data;
    msg.length = len;
    msg.next = NULL;
    msg.recv_buf = NULL;
    jee_pin_write(LCD_CS, 0);
    jee_pin_write(LCD_DC, 1);
    jee_spi_transfer_message(master_spi, &msg);
    jee_pin_write(LCD_CS, 1);
}

void vWriteData8(uint8_t data)
{
    struct jee_spi_message msg;
    msg.length = 1;
    msg.cs_release = 0;
    msg.cs_take = 1;
    msg.next = NULL;
    msg.send_buf = &data;
    jee_pin_write(LCD_CS, 0);
    jee_pin_write(LCD_DC, 1);
    jee_spi_transfer_message(master_spi, &msg);
    jee_pin_write(LCD_CS, 1);
}

void vWriteData16(uint16_t data)
{
    struct jee_spi_message msg;
    uint8_t buff[2];
    buff[0] = data >> 8;
    buff[1] = data;
    msg.length = 2;
    msg.cs_release = 0;
    msg.cs_take = 1;
    msg.send_buf = buff;
    msg.recv_buf = NULL;
    msg.next = NULL;
    jee_pin_write(LCD_CS, 0);
    jee_pin_write(LCD_DC, 1);
    jee_spi_transfer_message(master_spi, &msg);
    jee_pin_write(LCD_CS, 1);
}

void vLcdSetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint8_t cmd_buff[8];
    cmd_buff[0] = x1 >> 8;
    cmd_buff[1] = x1;
    cmd_buff[2] = x2 >> 8;
    cmd_buff[3] = x2;
    cmd_buff[4] = y1 >> 8;
    cmd_buff[5] = y1;
    cmd_buff[6] = y2 >> 8;
    cmd_buff[7] = y2;
    if (USE_HORIZONTAL == 0)
    {
        vWriteCommand(0x2a); // 列地址设置
        vWriteData(&cmd_buff[0], 4);
        vWriteCommand(0x2b); // 行地址设置
        vWriteData(&cmd_buff[4], 4);
        vWriteCommand(0x2c); // 储存器写
    }
    else if (USE_HORIZONTAL == 1)
    {
        vWriteCommand(0x2a); // 列地址设置
        vWriteData(&cmd_buff[0], 4);
        vWriteCommand(0x2b); // 行地址设置
        vWriteData(&cmd_buff[4], 4);
        vWriteCommand(0x2c); // 储存器写
    }
    else if (USE_HORIZONTAL == 2)
    {
        vWriteCommand(0x2a); // 列地址设置
        vWriteData(&cmd_buff[0], 4);
        vWriteCommand(0x2b); // 行地址设置
        vWriteData(&cmd_buff[4], 4);
        vWriteCommand(0x2c); // 储存器写
    }
    else
    {
        vWriteCommand(0x2a); // 列地址设置
        vWriteData(&cmd_buff[0], 4);
        vWriteCommand(0x2b); // 行地址设置
        vWriteData(&cmd_buff[4], 4);
        vWriteCommand(0x2c); // 储存器写
    }
}

#define IMG_PACK_SIZE (1024)
void vLcdClear(uint16_t color)
{
    uint32_t img_len = LCD_W * LCD_H;
    uint32_t remain = img_len; // 像素点为单位
    int pix_pos = 0;
    uint8_t color_buff[IMG_PACK_SIZE * 2];
    for (int i = 0; i < IMG_PACK_SIZE; i++)
    {
        color_buff[i * 2] = color >> 8;
        color_buff[i * 2 + 1] = color;
    }
    vLcdSetWindow(0, 0, LCD_W - 1, LCD_H - 1);
    while (remain > 0)
    {
        if (remain >= IMG_PACK_SIZE)
        {
            vWriteData(color_buff, IMG_PACK_SIZE * 2);
            pix_pos += IMG_PACK_SIZE * 2;
            remain -= IMG_PACK_SIZE;
        }
        else
        {
            vWriteData(color_buff, remain * 2);
            remain = 0;
        }
    }
}

// void vLcdClear(uint16_t color)
// {
//     uint16_t color_buff[1024*3];
//     memset(color_buff, color, sizeof(color_buff));
//     vLcdSetWindow(0, 0, LCD_W - 1, LCD_H - 1);

//     for (int i = 0; i < LCD_W; i++)
//     {
//         for (int j = 0; j < LCD_H; j++)
//         {
//             //vWriteData(&color, 2);
//             vWriteData16(color);
//         }
//     }
// }

void vLcdSetPoint(uint16_t x, uint16_t y)
{
    uint8_t data[2];
    vWriteCommand(0x2a);
    data[0] = x >> 8;
    data[1] = x;
    vWriteData(data, 2);
    vWriteCommand(0x2b);
    data[0] = y >> 8;
    data[1] = y;
    vWriteData(data, 2);
    vWriteCommand(0x2c);
}

void vLcdWriteRam(void)
{
    vWriteCommand(0x2c);
}

void vLcdDrawPoint(uint16_t x, uint16_t y)
{
    uint8_t data[2];
    vLcdSetPoint(x, y);
    vLcdWriteRam();
    data[0] = point_color >> 8;
    data[1] = point_color;
    vWriteData(data, 2);
}

uint16_t lLcdGetPointColor(void)
{
    return point_color;
}

void vLcdSetPointColor(uint16_t color)
{
    point_color = color;
}

int vSt7789Init(void)
{
    // init gpio for st7789
    point_color = 0xFFFF;
    jee_pin_mode(LCD_DC, PIN_MODE_OUTPUT);
    jee_pin_mode(LCD_RST, PIN_MODE_OUTPUT);
    jee_pin_mode(LCD_CS, PIN_MODE_OUTPUT);
    esp_spi_bus_attach_device(SPI_BUS_NAME, SPI_DEV_NAME, LCD_CS);
    printf("master spi test begin\n");

    master_spi = (struct jee_spi_device *)jee_device_find(SPI_DEV_NAME);

    if (!master_spi)
    {
        printf("spi sample run failed! can't find spi20 device!\n");
    }
    else
    {
        spi_bus_device_configuration_t dev_bus_cfg;
        spi_bus_config_t bus_cfg = SPI_BUS_TEST_DEFAULT_CONFIG();
        spi_device_interface_config_t dev_cfg = SPI_DEVICE_TEST_DEFAULT_CONFIG();
        dev_cfg.clock_speed_hz = 20 * 1000 * 1000;
        bus_cfg.miso_io_num = LCD_MISO;
        bus_cfg.mosi_io_num = LCD_MOSI;
        bus_cfg.sclk_io_num = LCD_SCLK;
        dev_bus_cfg.spi_bus = &bus_cfg;
        dev_bus_cfg.spi_device = &dev_cfg;
        master_spi->config.pConfig = (void *)(&dev_bus_cfg);
        jee_pin_write(LCD_CS, 1);
        jee_pin_write(LCD_RST, 1);
        jee_pin_write(LCD_DC, 1);
        jee_pin_write(LCD_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        jee_pin_write(LCD_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        vWriteCommand(0x11);
        vTaskDelay(pdMS_TO_TICKS(120)); // ms
        vWriteCommand(0x36); // MX, MY, RGB mode
        if (USE_HORIZONTAL == 0)
            vWriteData8(0x00);
        else if (USE_HORIZONTAL == 1)
            vWriteData8(0xC0);
        else if (USE_HORIZONTAL == 2)
            vWriteData8(0x70);
        else
            vWriteData8(0xA0);

        vWriteCommand(0x3A);
        vWriteData8(0x05); //    262K? 06
        vWriteCommand(0xB2);
        vWriteData8(0x0C);
        vWriteData8(0x0C);
        vWriteData8(0x00);
        vWriteData8(0x33);
        vWriteData8(0x33);

        vWriteCommand(0xB7);
        vWriteData8(0x75); // VGH=14.97V, VGL=-10.43V

        vWriteCommand(0xBB); // VCOM
        vWriteData8(0x21);

        vWriteCommand(0xC0);
        vWriteData8(0x2C);

        vWriteCommand(0xC2);
        vWriteData8(0x01);

        vWriteCommand(0xC3); // GVDD
        vWriteData8(0x13);

        vWriteCommand(0xC4);
        vWriteData8(0x20);

        vWriteCommand(0xC6);
        vWriteData8(0x0F);

        vWriteCommand(0xD0);
        vWriteData8(0xA4);
        vWriteData8(0xA1);

        vWriteCommand(0xd6);
        vWriteData8(0xa1);

        vWriteCommand(0xE0);
        vWriteData8(0x70);
        vWriteData8(0x04);
        vWriteData8(0x0A);
        vWriteData8(0x08);
        vWriteData8(0x07);
        vWriteData8(0x05);
        vWriteData8(0x32);
        vWriteData8(0x32);
        vWriteData8(0x48);
        vWriteData8(0x38);
        vWriteData8(0x15);
        vWriteData8(0x15);
        vWriteData8(0x2A);
        vWriteData8(0x2E);

        vWriteCommand(0xE1);
        vWriteData8(0x70);
        vWriteData8(0x07);
        vWriteData8(0x0D);
        vWriteData8(0x09);
        vWriteData8(0x09);
        vWriteData8(0x16);
        vWriteData8(0x30);
        vWriteData8(0x44);
        vWriteData8(0x49);
        vWriteData8(0x39);
        vWriteData8(0x16);
        vWriteData8(0x16);
        vWriteData8(0x2B);
        vWriteData8(0x2F);
        vWriteCommand(0x21);
        vWriteData8(0x00);
        vWriteCommand(0x29);
        vWriteData8(0x00);
        // vLcdClear(0x001F);
    }
    return 0;
}

// INIT_DEVICE_EXPORT(vSt7789Init);