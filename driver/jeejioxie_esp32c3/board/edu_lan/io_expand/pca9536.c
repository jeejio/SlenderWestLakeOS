#include "pca9536.h"
#include <stdio.h>
#include "hal_pwm.h"
#include "driver/i2c.h"
#include "hal_i2c.h"

jee_device_t pca9536Dev = NULL;

#define PCA9536_ADDR (0x41)

#define CMD_READ_PORT (0)
#define CMD_WRITE_PORT (1)
#define CMD_CONFIG_PORT (3) 

// output_status[0]: IO0.7--->IO0.0; output_status[1]: IO1.7--->IO1.0

// bit 1:input
// bit 0:output
void pca9536_io_config(uint8_t cfg0)
{
    uint8_t data[] = {CMD_CONFIG_PORT, cfg0};
    jee_i2c_master_send(pca9536Dev, PCA9536_ADDR, JEE_I2C_STOP, data, 2);
#if 0
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DEVICE_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, CMD_CONFIG_PORT, true);
	i2c_master_write_byte(cmd, cfg0, true);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (ret == ESP_OK)
	{
		//ESP_LOGI("io", "io configuration ok");
	}
	else
	{
		//ESP_LOGE("io", "io configuration failed. code: 0x%.2X", ret);
	}
	i2c_cmd_link_delete(cmd);
#endif
}

void pca9536_set_output(uint8_t io0)
{
    uint8_t data[] = {CMD_WRITE_PORT, io0};
    jee_i2c_master_send(pca9536Dev, PCA9536_ADDR, JEE_I2C_STOP, data, 2);

#if 0
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CMD_WRITE_PORT, true);
    i2c_master_write_byte(cmd, io0, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (ret == ESP_OK)
    {
        // ESP_LOGI("io", "io set ok");
    }
    else
    {
        // ESP_LOGE("io", "io set failed. code: 0x%.2X", ret);
    }
    i2c_cmd_link_delete(cmd);
#endif
}

int pca9536_read_io(void)
{
    uint8_t data = CMD_READ_PORT;
    int res=0;
    jee_i2c_master_send(pca9536Dev, PCA9536_ADDR, JEE_I2C_STOP, &data, 1);
    res=jee_i2c_master_recv(pca9536Dev, PCA9536_ADDR, JEE_I2C_STOP, &data, 1);
    //printf("-----%s:%x\n",__func__,data);
    if(res>0)
    {
        return data;

    }
    else
    {
        return -1;
    }

#if 0
    uint8_t buff[2] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CMD_READ_PORT, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEVICE_ADDR << 1) | I2C_MASTER_READ, true);

    i2c_master_read_byte(cmd, &buff[0], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        // ESP_LOGI("io", "io read ok");

        return buff[0];
    }
    else
    {
        // ESP_LOGE("io", "io read failed. code: 0x%.2X", ret);
        return -1;
    }
#endif
}

void pca9536Init(void)
{
    pca9536Dev = jee_device_find("i2c0hard");
    if (pca9536Dev == JEE_NULL)
    {
        printf("can not find %s Model\n", "i2c0hard");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(pca9536Dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open %s device\n", "i2c0hard");
        return;
    }
    pca9536_io_config(0xff);
}
