#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "driver/adc.h"
//#include "flexible_button.h"

/*
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
*/
#include <stdlib.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "hal_gpio.h"
#include "hal_adc.h"
#include "auto_init.h"

//#include "jos_kv.h"
#include "ky_023.h"
#define ADC_VRX_IO (2)
#define ADC_VRY_IO (3)
#define JOY_KEY_IO (9)

joystick_t joystick;

static int adc_raw[2][10];
static const char *TAG = "ADC SINGLE";

#define ADC1_CHAN0 2 //ADC1_CHANNEL_2
#define ADC1_CHAN1 3 //ADC1_CHANNEL_3

#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP

#define ADC_EXAMPLE_ATTEN ADC_ATTEN_DB_11

#define JOYSTICK_CHECK_CLICK_TIME 25      // 50*10MS =500MS
#define JOYSTICK_CHECK_LONG_PRESS_TIME 50 // 100*10MS =1000MS

#define JOYSTICK_CLICK 1
#define JOYSTICK_DOUBLE_CLICK 2
#define JOYSTICK_LONG_PRESS 3

#define ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED (1)
#define ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED (0)

bool joystick_calibration_zero_flags = 0;
bool joystick_calibration_max_min_flags = 1; /* 0 start  1 end*/
/*
static adc_cali_handle_t adc1_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
*/
static uint32_t joystick_count = 0;
static uint32_t first_joystick_count = 0; /**/
static uint8_t check_count = 0;
static uint8_t check_down = JOYSTICK_CHECK_CLICK_TIME;
static uint8_t check_key_status = 0;

int calibration_x_min = 0, calibration_y_min = 0;
int calibration_x_max = 0, calibration_y_max = 0;

static jee_adc_device_t ky023_adc_dev = NULL;
#define KY_ADC_DEV_NAME "adc1"

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
/*
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
*/

FILE *f1_joystick;

static int joystick_read_flash(void)
{
    f1_joystick = fopen("/data/joystick.bin", "rb");
    if (f1_joystick == NULL) {
        printf("Failed to open file for reading\n");
        return -1;
    }
    int data[4];
    size_t ret_r = fread(data, 4, 4, f1_joystick);
    fclose(f1_joystick);
    printf("read back: ");
    for(size_t i = 0; i < ret_r; i++) {
        printf("%d ", data[i]);
    }
    printf("\n");
    calibration_x_max=data[0];
    calibration_x_min=data[1];
    calibration_y_max=data[2];
    calibration_y_min=data[3];
    
    //jos_kv_get_data("joystick_x_max", &calibration_x_max, sizeof(calibration_x_max));
    //jos_kv_get_data("joystick_x_min", &calibration_x_min, sizeof(calibration_x_min));
    //jos_kv_get_data("joystick_y_max", &calibration_y_max, sizeof(calibration_y_max));
    //jos_kv_get_data("joystick_y_min", &calibration_y_min, sizeof(calibration_y_min));
    //printf("joystick_read_flash  %d  %d  %d  %d \n", calibration_x_max, calibration_x_min, calibration_y_max, calibration_y_min);
    return 0;
}

static int joystick_write_flash(void)
{
    f1_joystick = fopen("/data/joystick.bin", "wb");
    if (f1_joystick == NULL) {
        printf("Failed to open file for writing\n");
        return -1;
    }
    int data[4];
    data[0]=calibration_x_max;
    data[1]=calibration_x_min;
    data[2]=calibration_y_max;
    data[3]=calibration_y_min;
    size_t ret_r = fwrite(data, 4, 4, f1_joystick);
    fclose(f1_joystick);
    printf("write back: ");
    for(size_t i = 0; i < ret_r; i++) {
        printf("%d ", data[i]);
    }
    printf("\n");

    //jos_kv_set_data("joystick_x_max", &calibration_x_max, 4);
    //jos_kv_set_data("joystick_x_min", &calibration_y_min, 4);
    //jos_kv_set_data("joystick_y_max", &calibration_y_max, 4);
    //jos_kv_set_data("joystick_y_min", &calibration_y_min, 4);
    //printf("joystick_write_flash  %d  %d  %d  %d \n", calibration_x_max, calibration_x_min, calibration_y_max, calibration_y_min);
    return 0;
}

static int joystick_key_scan(void)
{
    check_count++;

    if (check_count == 1)
    {
        first_joystick_count = joystick_count;
        return 0;
    }

    if (check_count == 2)
    {
        uint32_t times;
        times = joystick_count - first_joystick_count;
        if (times > (JOYSTICK_CHECK_LONG_PRESS_TIME * 3))
        {
            check_count = 0;
            printf("\n long long click \n");
            joystick_calibration_max_min_flags = 0;
            return 4;
        }
        if (times > JOYSTICK_CHECK_LONG_PRESS_TIME)
        {
            check_count = 0;
            printf("\n long click \n");
            return JOYSTICK_LONG_PRESS;
        }
        if (times > JOYSTICK_CHECK_CLICK_TIME)
        {
            check_count = 0;
            printf("\n click \n");
            return JOYSTICK_CLICK;
        }
    }
    if (check_count == 3)
    {
        check_down = JOYSTICK_CHECK_CLICK_TIME;
        return 0;
    }
    if (check_count == 4)
    {
        check_count = 0;
        printf("\n double click \n");
        return JOYSTICK_DOUBLE_CLICK;
    }
    return 0;
}

static uint8_t read_key(void)
{
    return jee_pin_read(JOY_KEY_IO)? 1 : 0;
    //return gpio_get_level(JOY_KEY_IO) ? 1 : 0;
}

static void joy_key_init(void)
{
    /*
    uint8_t i;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << JOY_KEY_IO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    */
    jee_pin_mode(JOY_KEY_IO,PIN_MODE_INPUT_PULLUP);
}

void BubbleSort(int *arr, int size)
{
    int i, j, tmp;
    for (i = 0; i < size - 1; i++)
    {
        for (j = 0; j < size - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                tmp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = tmp;
            }
        }
    }
}

static void joystick_task(void *arg)
{
    uint32_t voltage = 0;

    joy_key_init();
    // ADC1 config
    joystick.key_sta = 1;
    joystick.x_value = 0;
    joystick.y_value = 0;
    /************************以下是：esp接口-方式*****************************/
    /* 
    //-------------ADC1 Init---------------//

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//

    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
    */
    /************************以下是：jee-iomodel-方式************************/    
    char *result_str;
    ky023_adc_dev = (jee_adc_device_t)jee_device_find(KY_ADC_DEV_NAME);
    result_str = (ky023_adc_dev == NULL) ? "failure" : "success";
    printf("probe %s \n", result_str);
    jee_adc_enable(ky023_adc_dev, (1<<2)|(1<<3));

    /*calibration  start*/
    int calibration_x_zero = 0, calibration_y_zero = 0;
    calibration_x_min = 0, calibration_y_min = 0;
    calibration_x_max = 0, calibration_y_max = 0;
    joystick_read_flash();
    printf("calibration  start\n");
    {
        /*定标模式：获取原点参数*/
        /*上电1S中 定标零点*/
        int store_x_value[100], store_y_value[100];
        uint8_t adc_count = 0, store_count = 0;
        while (!joystick_calibration_zero_flags)
        {
//            adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][adc_count]); // X
//            adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[1][adc_count]); // Y
            adc_raw[0][adc_count] = jee_adc_read(ky023_adc_dev, 3);
            adc_raw[1][adc_count] = jee_adc_read(ky023_adc_dev, 2);

            adc_count++;
            if (adc_count == 5) /* 每次进入需要10ms   每5笔数据中，取中间值参与计算 */
            {
                BubbleSort(&adc_raw[0][0], 5);
                BubbleSort(&adc_raw[1][0], 5);
                store_x_value[store_count] = adc_raw[0][2];
                store_y_value[store_count] = adc_raw[1][2];
                store_count++;
                adc_count = 0;
            }
            if (store_count == 100) /*  100笔数据中取平均值 作为 0飘参数 */
            {
                for (int i = 0; i < 100; i++)
                {
                    calibration_x_zero += store_x_value[i];
                    calibration_y_zero += store_y_value[i];
                }
                calibration_x_zero = calibration_x_zero / 100;
                calibration_y_zero = calibration_y_zero / 100;
                joystick_calibration_zero_flags = 1;
                printf("calibration_x_zero : %d,calibration_y_zero :%d \n", calibration_x_zero, calibration_y_zero);
                printf("check max min value  waiting \n");
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    while (1)
    {
        {
            printf("check max_min value  start \n");
            /*校准模式：获取最大值，最小值*/
            bool onoff_status = 1;
            uint8_t adc_count = 0, store_count = 0;
            int store_x_value[100], store_y_value[100];
            // check_key_status = 0;
            while (!joystick_calibration_max_min_flags)
            {
                //adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][adc_count]); // X
                //adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[1][adc_count]); // Y
                adc_raw[0][adc_count] = jee_adc_read(ky023_adc_dev, 3);
                adc_raw[1][adc_count] = jee_adc_read(ky023_adc_dev, 2);
                adc_count++;
                if (adc_count == 5) /*  每5笔数据中，取中间值参与计算 */
                {
                    BubbleSort(&adc_raw[0][0], 5);
                    BubbleSort(&adc_raw[1][0], 5);
                    store_x_value[store_count] = adc_raw[0][2];
                    store_y_value[store_count] = adc_raw[1][2];
                    store_count++;
                    adc_count = 0;
                }
                if (store_count == 100) /* 100次参与计算的数据中，排序取最大最小值，减去0飘参数。 */
                {
                    BubbleSort(&store_x_value[0], 100);
                    BubbleSort(&store_y_value[0], 100);
                    /*10-90 数据 作为 最大最小值 0,100   10以及以下都为0  90以及以上都为100*/
                    calibration_x_min = store_x_value[10];
                    calibration_x_max = store_x_value[90];
                    calibration_y_min = store_y_value[10];
                    calibration_y_max = store_y_value[90];

                    calibration_x_min -= calibration_x_zero;
                    calibration_x_max -= calibration_x_zero;
                    calibration_y_min -= calibration_y_zero;
                    calibration_y_max -= calibration_y_zero;
                    joystick_calibration_max_min_flags = 1;
                    printf("calibration_x_min : %d,calibration_x_max :%d calibration_y_min : %d,calibration_y_max :%d \n", calibration_x_min, calibration_x_max, calibration_y_min, calibration_y_max);
                    printf("check max_min value  end \n");
                    joystick_write_flash();
                    check_key_status = 5;
                }
                vTaskDelay(pdMS_TO_TICKS(2 * 10));
            }
        }
        /*calibration  end*/
        {
            /*正常模式*/
            uint8_t adc_count = 0;
            int current_x_value = 0, current_y_value = 0;
            // check_key_status = 0;
            while (joystick_calibration_max_min_flags)
            {
                {
                    //check_key_status = 0;
                }
                //adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw[0][adc_count]); // X
                //adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw[1][adc_count]); // Y
                adc_raw[0][adc_count] = jee_adc_read(ky023_adc_dev, 3);
                adc_raw[1][adc_count] = jee_adc_read(ky023_adc_dev, 2);
                if ((adc_raw[0][0] != 0) && (adc_raw[1][0] != 0))                  /*剔除采集为0的数据*/
                {
                    adc_count++;
                    if (adc_count >= 5) /*  每5笔数据中，取中间值参与计算 */
                    {
                        BubbleSort(&adc_raw[0][0], 5);
                        BubbleSort(&adc_raw[1][0], 5);
                        adc_count = 0;
                        /*取中间值作为有效参数*/
                        current_x_value = adc_raw[0][2];
                        current_y_value = adc_raw[1][2];
                        /*减去O飘 得到相对0点的参数*/
                        current_x_value -= calibration_x_zero;
                        current_y_value -= calibration_y_zero;
                        /*计算坐标*/
                        {
                            if (current_x_value > 0)
                            {
                                if (calibration_x_max == 0)
                                {
                                    joystick.x_value = 111;
                                }
                                else
                                {
                                    joystick.x_value = (current_x_value * 100) / calibration_x_max;
                                    if (joystick.x_value > 100)
                                    {
                                        joystick.x_value = 100;
                                    }
                                }
                            }
                            else
                            {
                                if (calibration_x_min == 0)
                                {
                                    joystick.x_value = 111;
                                }
                                else
                                {
                                    joystick.x_value = -(current_x_value * 100) / calibration_x_min;
                                    if (joystick.x_value < -100)
                                    {
                                        joystick.x_value = -100;
                                    }
                                }
                            }
                            if (current_y_value > 0)
                            {
                                if (calibration_y_max == 0)
                                {
                                    joystick.y_value = 111;
                                }
                                else
                                {
                                    joystick.y_value = (current_y_value * 100) / calibration_y_max;
                                    if (joystick.y_value > 100)
                                    {
                                        joystick.y_value = 100;
                                    }
                                }
                            }
                            else
                            {
                                if (calibration_y_min == 0)
                                {
                                    joystick.y_value = 111;
                                }
                                else
                                {
                                    joystick.y_value = -(current_y_value * 100) / calibration_y_min;
                                    if (joystick.y_value < -100)
                                    {
                                        joystick.y_value = -100;
                                    }
                                }
                            }
                            /*此处再次过滤一下，让原点状态数据稳定*/
                            {
                                if ((joystick.x_value > -10) && (joystick.x_value < 10))
                                {
                                    joystick.x_value = 0;
                                }
                                if ((joystick.y_value > -10) && (joystick.y_value < 10))
                                {
                                    joystick.y_value = 0;
                                }
                            }
                        }
                        //printf("current_x_value %d  current_y_value %d \n", joystick.x_value, joystick.y_value);
                    }
                    else
                    {
                        vTaskDelay(pdMS_TO_TICKS(2));
                        continue;
                    }
                }
                else
                {

                    continue;
                }

                if ((read_key() != joystick.key_sta))
                {
                    check_key_status = joystick_key_scan();
                }
                if (check_count == 2)
                {
                    check_down--;
                    if (check_down == 0)
                    {
                        check_down = JOYSTICK_CHECK_CLICK_TIME;
                        check_count--;
                        check_key_status = joystick_key_scan();
                    }
                }
                joystick.key_sta = read_key();
                joystick_count++;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }
}

uint8_t joystick_upload(void)
{
    uint8_t ret = check_key_status;
    check_key_status = 0;
    return ret;
}

void joystick_init(void)
{
    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
}

bool joystick_get_button_status(void)
{
    return joystick.key_sta;
}

button_state_t joystick_get_button_mode(void)
{
    return BTN_STATE_CLICK;
}

int joystick_get_x_pos(void)
{
    return joystick.x_value;
}

int joystick_get_y_pos(void)
{
    return joystick.y_value;
}


uint8_t joystick_mode_receive_upload(char *str)
{
    char *str2 = NULL;

    uint8_t ret = joystick_upload();
    if (ret != 0)
    {
        switch (ret)
        {
        case 1:
            /* code */
            str2 = "Click";
            sprintf(str, "{\"mode\":\"%s\"}", str2);
            break;
        case 2:
            /* code */
            str2 = "DoubleClick";
            sprintf(str, "{\"mode\":\"%s\"}", str2);
            break;
        case 3:
            /* code */
            str2 = "LongPress";
            sprintf(str, "{\"mode\":\"%s\"}", str2);
            break; 
        case 4:
            str2 = "CalibrationStrat";
            sprintf(str, "{\"mode\":\"%s\"}", str2);
            break;
        case 5:
            str2 = "CalibrationEnd";
            sprintf(str, "{\"mode\":\"%s\"}", str2);
            break;      
        default:
            break;
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t joystick_position_recevice(char *str)
{
    uint8_t flag = 1;
/*
    if((joystick.x_value!=0) && (joystick.y_value!=0 ))
    {
        flag=1;
    }else{
        flag=0;
    }
*/
    if(flag==1)
    {
        sprintf(str, "{\"positionX\":%d,\"positionY\":%d}", joystick.x_value, joystick.y_value);
    }
    return flag;
}

void joystick_get_info(int *buf)
{
    buf[0]=joystick.x_value;
    buf[1]=joystick.y_value;
    buf[2]=check_key_status;
    buf[3]=1-joystick.key_sta;
}

void joystick_clean_mode(void)
{
    /*清除按键(单击 双击 长按的状态)*/
    check_key_status=0;
}
