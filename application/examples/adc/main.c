#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "device.h"
#include "hal_adc.h"
#include "hal_cputime.h"
#include "esp_log.h"
#include "auto_init.h"

#define ADC_DEV_NAME "adc1"
#define ADC_DEV_CHANNEL 0

void app_main(void)
{
    static jee_adc_device_t adc_dev = NULL;
    int value;
    char *result_str;

#ifdef configUSING_COMPONENTS_INIT
    // Onboard components initialization.
    //rt_components_board_init();
    // components initialization.
    rt_components_init();
#endif

    //jee_hw_adc_init();
    adc_dev = (jee_adc_device_t)jee_device_find(ADC_DEV_NAME);
    result_str = (adc_dev == NULL) ? "failure" : "success";
    LOGI(ADC_DEV_NAME,"probe %s", result_str);

    jee_adc_enable(adc_dev, 1 << ADC_DEV_CHANNEL);

    for(int i = 0; i < 100; i++)
    {
        value = jee_adc_read(adc_dev, ADC_DEV_CHANNEL);
        LOGI(ADC_DEV_NAME,"channel 0 voltage is %d mV", value);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
