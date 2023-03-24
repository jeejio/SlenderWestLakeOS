#include "dht20.h"
#include <stdio.h>
// #include "esp_log.h"
#include "esp32c3/rom/ets_sys.h" //for ets_delay_us()
#include "driver/i2c.h"
#include "hal_i2c.h"

// #include "product.h"

typedef struct
{
    uint8_t key_sta[4];
    uint8_t led_sta[4];
    int temperature;
    int humidity;
    uint8_t smog_status;
    uint8_t buzzer_status;
    uint16_t smog_value;
    uint8_t pir_status;
    uint8_t fire_status;
} device_info_t;

device_info_t device_info;

#define DHT20_ADDR (0x38)

#define TAG "DHT20"

jee_device_t Dth20Dev = NULL;



void dht20_init(void)
{

    Dth20Dev = jee_device_find("i2c0hard");
    if (Dth20Dev == JEE_NULL)
    {
        printf("can not find %s Model\n", "i2c0hard");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(Dth20Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", "i2c0hard");
        return;
    }

    uint8_t write_data[3] = {0xa8, 0x00, 0x00};
    jee_i2c_master_send(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, write_data, 3);

#if 0
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xa8, true); // enter normal workmode
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        //ESP_LOGI(TAG, "dht20 init ok");
        printf("dht20 init ok\n");
    }
    else
    {
        //ESP_LOGE(TAG, "dht20 init failed. code: 0x%.2X", espRc);
        printf("dht20 init failed. code: 0x%.2X\n", espRc);
    }
    i2c_cmd_link_delete(cmd);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t write_data2[3] = {0xbe, 0x08, 0x00};
    jee_i2c_master_send(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, write_data2, 3);

#if 0
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xbe, true); // init dht20
    i2c_master_write_byte(cmd, 0x08, true); // calibration output
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        //ESP_LOGI(TAG, "dht20 init ok");
        printf("dht20 init ok\n");

    }
    else
    {
        //ESP_LOGE(TAG, "dht20 init failed. code: 0x%.2X", espRc);
        printf("dht20 init failed. code: 0x%.2X\n", espRc);

    }
    i2c_cmd_link_delete(cmd);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
   
}

uint8_t dht20_read_status(void)
{
    uint8_t readData;
    jee_i2c_master_recv(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, &readData, 1);
#if 0
    uint8_t status = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        // ESP_LOGI(TAG, "dht20 status read ok");
    }
    else
    {
        //ESP_LOGE(TAG, "dht20 status read failed. code: 0x%.2X", espRc);
        printf("dht20 status read failed. code: 0x%.2X\n", espRc);
    }
    i2c_cmd_link_delete(cmd);
  
    return status;
#endif
    return readData;
}

void jh_reset_reg(uint8_t addr)
{
    uint8_t write_data[3] = {0x00, 0x00, 0x00};
    write_data[0] = addr;
    jee_i2c_master_send(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, write_data, 3);

#if 0
    uint8_t data[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, addr, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        //ESP_LOGI(TAG, "dht20 init ok");
    }
    else
    {
        //ESP_LOGE(TAG, "dht20 init failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
    jee_i2c_master_recv(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, write_data, 3);

#if 0
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        // ESP_LOGI(TAG, "jh_reset_reg ok");
    }
    else
    {
        //ESP_LOGE(TAG, "jh_reset_reg failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
    write_data[0] = 0xb0 | addr;
    jee_i2c_master_send(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, &write_data[1], 2);
#if 0
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xb0 | addr, true);
    i2c_master_write_byte(cmd, data[1], true);
    i2c_master_write_byte(cmd, data[2], true);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        // ESP_LOGI(TAG, "dht20 init ok");
    }
    else
    {
        // ESP_LOGE(TAG, "dht20 init failed. code: 0x%.2X", espRc);
        printf("dht20 init failed. code: 0x%.2X\n", espRc);
    }
    i2c_cmd_link_delete(cmd);
#endif
    vTaskDelay(pdMS_TO_TICKS(10));
}

void dht20_startup_init(void)
{
    jh_reset_reg(0x1b);
    jh_reset_reg(0x1c);
    jh_reset_reg(0x1e);
}

void dht20_send_ac(void) // send cmd 0xac
{
    uint8_t write_data[3] = {0xac, 0x33, 0x00};

    jee_i2c_master_send(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, write_data, 3);
#if 0
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, 0xac, true);
    i2c_master_write_byte(cmd, 0x33, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        // ESP_LOGI(TAG, "dht20 init ok");
    }
    else
    {
        // ESP_LOGE(TAG, "dht20 init failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
#endif
}

void dht20_read_humi_temp(uint32_t *pdata)
{
    uint8_t data[6] = {0};
    uint16_t cnt = 0;
    dht20_send_ac();
    vTaskDelay(pdMS_TO_TICKS(80));
    while (((dht20_read_status() & 0x80) == 0x80))
    {
        ets_delay_us(1508);
        if (cnt++ >= 100)
        {
            break;
        }
    }
    jee_i2c_master_recv(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, data, 6);
#if 0
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[3], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[4], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[5], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        //ESP_LOGI(TAG, "read temp and humi ok");
    }
    else
    {
        //ESP_LOGE(TAG, "read temp and humi failed. code: 0x%.2X", espRc);
        printf("read temp and humi failed. code: 0x%.2X\n",espRc);
    }
    i2c_cmd_link_delete(cmd);
#endif
    pdata[0] = ((data[1] << 16) + (data[2] << 8) + (data[3])) >> 4;      // humidity
    pdata[1] = ((data[3] << 16) + (data[4] << 8) + (data[5])) & 0xfffff; // temperature
}

uint8_t calc_crc8(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t byte;
    uint8_t crc = 0xff;
    for (byte = 0; byte < len; byte++)
    {
        crc ^= (data[byte]);
        for (i = 8; i > 0; --i)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

int dht20_read_humi_temp_crc(uint32_t *pdata)
{
    uint8_t data[7] = {0};
    // uint8_t humi_temp_data[6] = {0};
    // uint32_t ret = 0;
    uint16_t cnt = 0;
    dht20_send_ac();
    vTaskDelay(pdMS_TO_TICKS(80));
    while (((dht20_read_status() & 0x80) == 0x80))
    {
        ets_delay_us(1508);
        if (cnt++ >= 100)
        {
            break;
        }
    }
     jee_i2c_master_recv(Dth20Dev, DHT20_ADDR, JEE_I2C_STOP, data, 7);
     #if 0
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[3], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[4], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[5], I2C_MASTER_ACK);

    i2c_master_read_byte(cmd, &data[6], I2C_MASTER_NACK); // crc data
    i2c_master_stop(cmd);
    esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        // ESP_LOGI(TAG, "read temp and humi ok");
    }
    else
    {
        // ESP_LOGE(TAG, "read temp and humi failed. code: 0x%.2X", espRc);
        printf("read temp and humi failed. code: 0x%.2X\n", espRc);
    }
    i2c_cmd_link_delete(cmd);
    #endif
    if (calc_crc8(data, 6) == data[7])
    {
        pdata[0] = ((data[1] << 16) + (data[2] << 8) + (data[3])) >> 4;      // humidity
        pdata[1] = ((data[3] << 16) + (data[4] << 8) + (data[5])) & 0xfffff; // temperature
        return 0;
    }
    return -1;
}

uint32_t humi_temp_data[2] = {0};
void dht20_get_data(int32_t *data)
{
    dht20_read_humi_temp(humi_temp_data);
    device_info.humidity = (humi_temp_data[0] * 100 * 10 / 1024 / 1024) / 10;
    device_info.temperature = (humi_temp_data[1] * 200 * 10 / 1024 / 1024 - 500); //
   // printf("dht20_task tem %d,hum %d\n", device_info.temperature, device_info.humidity);
    data[0] = device_info.temperature;
    data[1] = device_info.humidity;
}


