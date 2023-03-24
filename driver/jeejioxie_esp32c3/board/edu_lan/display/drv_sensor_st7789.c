#include "device.h"
#include "hal_sensor.h" //sesor IO模型驱动框架头文件

#include "string.h"
#include "auto_init.h"
#include "drv_sensor_st7789.h"

/**
 * @brief       读取传感器状态
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
    return 0;
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
    LOGI("display", "display sensor_control enter [cmd ] %d\n", cmd);

    switch (cmd)
    {
    case JEE_SENSOR_CTRL_SET_POWER:
    {
        int power_mode_type = (int)arg;
        switch (power_mode_type)
        {
        case JEE_SENSOR_POWER_NORMAL:
        {
            LOGI("display", " start : display \n");
            vSt7789Init();
            LOGI("display", " end   : display \n");
        }
        break;
        case JEE_SENSOR_POWER_DOWN:
        {
        }
        break;

        default:
            break;
        }
    }break;
    case SENSOR_DISPLAY_CLEAR:
    {
        vLcdClear(((uint16_t *)arg)[0]);
    }
    break;
    case SENSOR_DISPLAY_SET_POINT_COLOR:
    {
        vLcdSetPointColor(((uint16_t *)arg)[0]);
    }break;
    case SENSOR_DISPLAY_DRAW_POINT:
    {
        vLcdDrawPoint(((uint16_t *)arg)[0], ((uint16_t *)arg)[1]);
    }break;
    case SENSOR_DISPLAY_SET_WINDOW:
    {
        vLcdSetWindow(((uint16_t *)arg)[0], ((uint16_t *)arg)[1], ((uint16_t *)arg)[2], ((uint16_t *)arg)[3]);
    }break;

    case SENSOR_DISPLAY_WRITE_DATA:
    {
        lcd_data_t *data = (lcd_data_t *)arg;
        vWriteData(data->buffer,data->length);
    }break;

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
int sensor_display_init(const char *name, struct jee_sensor_config *cfg)
{
    jee_int8_t result;
    jee_sensor_t sensor_display = JEE_NULL;

    /* sensor register */
    sensor_display = pvPortMalloc(sizeof(struct jee_sensor_device));
    if (sensor_display == JEE_NULL)
        return -1;
    LOGI("display", "reg lcd model  1\n");
    memset(sensor_display, 0, sizeof(struct jee_sensor_device));
    LOGI("display", "reg lcd model  2\n");
    sensor_display->info.type = JEE_SENSOR_CLASS_DISPLAY;
    sensor_display->info.vendor = JEE_SENSOR_VENDOR_UNKNOWN;
    sensor_display->info.model = "st7789";
    // sensor_display->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    // sensor_display->info.intf_type = JEE_SENSOR_INTF_SPI;
    // sensor_display->info.range_max  = SENSOR_TEMP_RANGE_MAX;
    // sensor_display->info.range_min  = SENSOR_TEMP_RANGE_MIN;
    LOGI("display", "reg lcd model  3\n");
    if (cfg != JEE_NULL)
        memcpy(&sensor_display->config, cfg, sizeof(struct jee_sensor_config));

    sensor_display->ops = &sensor_ops;
    LOGI("display", "reg lcd model 4\n");
    result = jee_hw_sensor_register(sensor_display, name, JEE_DEVICE_FLAG_RDWR, JEE_NULL);
    LOGI("display", "reg lcd model 5\n");
    if (result != JEE_EOK)
    {
        // LOG_E("device register err code: %d", result);
        goto __exit;
    }

    return JEE_EOK;

__exit:
    if (sensor_display)
        vPortFree(sensor_display);

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
int jee_hw_sensor_display_init()
{
    //LOGI("display", "begin to execute jee_hw_sensor_display_init\n");

    struct jee_sensor_config cfg = {0};
    cfg.intf.dev_name = "lcd";
    cfg.intf.user_data = NULL;
    sensor_display_init("st7789", &cfg);

    //LOGI("display", "finish to execute jee_hw_sensor_display_init\n");
    return JEE_EOK;
}

INIT_DEVICE_EXPORT(jee_hw_sensor_display_init);
