/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2022-12-13     zhengqian      the first version
 */



#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件
#include "mq135.h"      //厂家驱动头文件
#include "string.h"
#include "auto_init.h"
#include "driver/i2c.h"

 /**
   * @brief       读取空气质量传感器状态
   *
   * NOTE:        -
   * 
   * @param[in]   sensor    - 传感器指针
   * @param[out]  buf      - 出参指针，读取的状态通过该指针带出
   * @param[in]   len      - 入参指针，指示入参指针的数据长度
   * @return      固定为0
   *
   */
static jee_size_t sensor_fetch_data(struct jee_sensor_device *sensor, void *buf, jee_size_t len)
{
     uint8_t *pbuf = buf;

    if (buf == NULL)
        return 0;


    pbuf[0] = mq135_get_data();

    return (1);
}

 /**
   * @brief       sensor指令处理函数
   *
   * NOTE:        -
   * 
   * @param[in]   sensor    - 传感器指针
   * @param[in]   cmd       - 命令，用于制定对传感器的操作类型
   * @param[in]   arg       - 入参指针，用于传入各种类型的数据
   * @return      固定为JEE_EOK
   *
   */
static jee_err_t sensor_control(struct jee_sensor_device *sensor, int cmd, void *arg)
{
    printf("mq135_init sensor_control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            printf(" start : mq135_init \n");
            mq135_init();
            printf(" end   : mq135_init \n");
        }
        break;
        case JEE_SENSOR_POWER_DOWN:
        {
        }
        break;
        default:
            break;
        }
    }

    break;

    default:
        break;
    }
    return JEE_EOK;
}

static struct jee_sensor_ops sensor_ops =
    {
        sensor_fetch_data,
        sensor_control};

 /**
   * @brief       sensor框架注册
   *
   * NOTE:        -
   * 
   * @param[in]   sensor    - 设备名称
   * @param[in]   cfg       - 传感器的配置参数
   * @return      返回sensor框架注册结果
   *
   */
int sensor_mq135_init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_getsture = JEE_NULL;

    /* sensor register */
    sensor_getsture = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_getsture == JEE_NULL)
        return -1;

    memset(sensor_getsture, 0, sizeof(struct jee_sensor_device));

    sensor_getsture->info.type = JEE_SENSOR_CLASS_IAQ;
    sensor_getsture->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_getsture->info.model = "mq135";
    // sensor_temp_humi->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor_getsture->info.intf_type = JEE_SENSOR_INTF_I2C;
    // sensor_temp_humi->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_temp_humi->info.range_min  = SENSOR_TEMP_RANGE_MIN;

    if (cfg != JEE_NULL)
        memcpy(&sensor_getsture->config, cfg, sizeof(struct jee_sensor_config));

    sensor_getsture->ops = &sensor_ops;

    result = jee_hw_sensor_register(sensor_getsture, name, JEE_DEVICE_FLAG_RDONLY, JEE_NULL);
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

   
    return JEE_EOK;

__exit:
    if (sensor_getsture)
        vPortFree(sensor_getsture);

    return -JEE_ERROR;
}

 /**
   * @brief       sensor初始化
   *
   * NOTE:        -
   * 
   * @return      固定为JEE_EOK
   *
   */
int jee_hw_sensor_mq135_init()
{
    //printf("begin to execute jee_hw_sensor_mq135_init\n");
    // i2c_master_init();

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = "iic0";
    cfg.intf.user_data = NULL;
    sensor_mq135_init("mq135", &cfg);

    //printf("finish to execute jee_hw_sensor_mq135_init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jee_hw_sensor_mq135_init);


