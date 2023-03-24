#include "mk002.h"
#include "driver/gpio.h"
//#include "driver/adc_oneshot.h"
//#include "driver/adc_cali.h"
//#include "driver/adc_cali_scheme.h"
#include "esp_log.h"
#include "string.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "device.h"
#include "auto_init.h"

//#define ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED (1)
//#define ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED (0)

#define RainDropGPIO_INPUT 8
#define RainDrop_INPUT_PIN_SEL ((1ULL << RainDropGPIO_INPUT))
#define ESP_INTR_FLAG_DEFAULT 0

#define DEFAULT_VREF 1100                           // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64                            // Multisampling

static const char *TAG = "Hyetometer";
/*
static const adc_channel_t channel = ADC_CHANNEL_2; // GPIO3
//static const adc_atten_t atten = ADC_ATTEN_DB_11;
//static const adc_unit_t unit = ADC_UNIT_1;
static int adc_raw[10];
adc_cali_handle_t adc1_cali_handle = NULL;
adc_oneshot_unit_handle_t adc1_handle;
*/
#define Light_RainDropVoltage 2770
#define Middle_RainDropVoltage 1800
#define Heavy_RainDropVoltage 1600

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
#define MK002_ADC_DEV_NAME "adc1"
#define ADC_DEV_CHANNEL 2
static jee_adc_device_t mk002_adc_dev = NULL;

void hyetometer_init(void)
{
    /************************以下是：esp接口-方式*****************************/
    /*
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = RainDrop_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    */
    /************************以下是：jee-iomodel-方式************************/
    jee_pin_mode(PIN_MODE_INPUT_PULLUP,RainDropGPIO_INPUT);


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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));
    //-------------ADC1 Calibration Init---------------//

    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
    */   

    /************************以下是：jee-iomodel-方式************************/
    char *result_str;
    mk002_adc_dev = (jee_adc_device_t)jee_device_find(MK002_ADC_DEV_NAME);
    result_str = (mk002_adc_dev == NULL) ? "failure" : "success";
    printf("probe %s \n", result_str);
    jee_adc_enable(mk002_adc_dev,1 << ADC_DEV_CHANNEL);
    
}

bool hyetometer_get_status(void)
{

    if (jee_pin_read(RainDropGPIO_INPUT) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t hyetometer_get_level(void)
{
    uint32_t adc_reading = 0;
    int adc_value = 0;
    // Multisampling
    /*
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        adc_oneshot_read(adc1_handle, channel, &adc_value);
        adc_reading += adc_value;
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert adc_reading to voltage in mV
    uint32_t voltage = 0;
    adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &voltage);
    printf("voltage is %ld,adc_reading %ld\n", voltage, adc_reading);
    */
    int voltage = jee_adc_read(mk002_adc_dev, ADC_DEV_CHANNEL);
    printf("voltage %d \n",voltage);
    
    if (gpio_get_level(RainDropGPIO_INPUT) == 1)
    {
        return None;
    }
    else if (voltage > Middle_RainDropVoltage)
    {
        return Light;
    }
    else if (voltage <= Middle_RainDropVoltage && voltage > Heavy_RainDropVoltage)
    {
        return Middle;
    }
    else if (voltage < Heavy_RainDropVoltage)
    {
        return Heavy;
    }
    else
    {
        return 0;
    }
}

void hyetometer_get_info(int *buf)
{
    buf[0]=hyetometer_get_status();
    buf[1]=hyetometer_get_level();
}

static bool rain_state = false;
static uint8_t rain_level = 0;

uint8_t rian_sensor_data_pack(char *str)
{

    uint8_t flag = 0;

    bool ra_s = hyetometer_get_status();
    uint8_t ra_l = hyetometer_get_level();
 
//    if (ra_s != rain_state)
    {
        rain_state = ra_s;
        flag = 1;
    }
//    else if (rain_level != ra_l)
    {
        rain_level = ra_l;
        flag = 1;
    }
  
    printf("rain_level %d,state %d\n", rain_level, rain_state);

    if (flag == 1)
    {
        if (rain_state == 1)
        {

            char *str2 = NULL;
            if (rain_level >= 1)
            {
                if (rain_level == Light)
                {
                    str2 = "Light";
                }
                else if (rain_level == Middle)
                {
                    str2 = "Middle";
                }
                else if (rain_level == Heavy)
                {
                    str2 = "Heavy";
                }
                else
                {
                    str2 = "No";
                }
                sprintf(str, "{\"isRaining\":true,\"precipitationLevel\":\"%s\"}", str2);
            }
        }
        else
        {
            sprintf(str, "{\"isRaining\":false}");
        }
    }
    return flag;
}

/*
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
static void vMk002TestTask(void *arg)
{
    char data[128];
    while (1)
    {
        rian_sensor_data_pack(data);
        printf("%s",data);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// main

void vMk002Test(void)
{
    printf("sensor example Start....\n");
    hyetometer_init();
    xTaskCreate(vMk002TestTask, "vMk002TestTask", 2048, NULL, 10, NULL);
}
INIT_APP_EXPORT(vMk002Test);
*/


