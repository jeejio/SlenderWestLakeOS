/*
 * Copyright (c) 2018-2023, jeejio
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jeedef.h"
#include "device.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "driver/adc_oneshot.h"
#include "driver/adc_cali.h"
#include "driver/adc_cali_scheme.h"
#include "auto_init.h"
#include "hal_adc.h"

const static char *TAG = "adc";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels

#define	GET_LOW_BIT(x,y)	    ((x >> y) & 0x00000001)	/* 获取X第ybit的值 */

struct esp_adc_type {
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config;
    adc_oneshot_chan_cfg_t config;
    adc_cali_handle_t adc_cali_handle;
    bool do_calibration;
};

struct esp32c3_adc {
    struct jee_adc_device esp32c3_adc_device;
    struct esp_adc_type adcx;
    char *name;
};

static struct esp32c3_adc adc_obj_list[] = {
    {
        /************************以下为 雨滴传感器 配置参数*****************************/  
        .name = "adc1",
        .adcx = {
            .init_config = {.unit_id = ADC_UNIT_1},
            .config = {.bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_11}   
        },
    },
#if (SOC_ADC_PERIPH_NUM >= 2)
    {
        .name = "adc2",
        .adcx = {
            .init_config = {.unit_id = ADC_UNIT_2, .ulp_mode = ADC_ULP_MODE_DISABLE},
            .config = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11}
        }
    }
#endif
};

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void adc_calibration_deinit(adc_cali_handle_t handle);

static jee_err_t esp_adc_enabled(struct jee_adc_device *device, jee_uint32_t channel, jee_bool_t enabled)
{
    struct esp_adc_type *adc_type;

    adc_type = device->parent.user_data;
    if(!enabled) {
        ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_type->adc_handle));
        if (adc_type->do_calibration) {
            adc_calibration_deinit(adc_type->adc_cali_handle);
        }
    } else {
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_type->init_config, &adc_type->adc_handle));
        {   /*为了满足一次使能多条channel的需求：传入channel 1-10bit 对应实际乐鑫 的channel(0-9)  xu:20230214 */
            for(int i=0;i<10;i++)
            {
                if(GET_LOW_BIT(channel,i))
                {
                    printf("esp_adc_enabled adc channel %d \n",i);
                    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_type->adc_handle, i, &adc_type->config));
                }
            }
        }
        adc_type->do_calibration = adc_calibration_init(adc_type->init_config.unit_id, adc_type->config.atten, &adc_type->adc_cali_handle);
    }
    return JEE_EOK;
}

static jee_err_t esp_get_adc_value(struct jee_adc_device *device, jee_uint32_t channel, int *value)
{
    struct esp_adc_type *adc_type;
    int raw_data;

    adc_type = device->parent.user_data;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_type->adc_handle, channel, &raw_data));
    if (adc_type->do_calibration)
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_type->adc_cali_handle, raw_data, value));
    }
    return JEE_EOK;
}

static const struct jee_adc_ops esp_adc_ops =
{
    .enabled = esp_adc_enabled,
    .convert = esp_get_adc_value,
};

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

int jee_hw_adc_init(void)
{
    int result = JEE_EOK;
    int i = 0;

    for(i = 0; i< sizeof(adc_obj_list) / sizeof(adc_obj_list[0]); i++)
    {
        if(jee_hw_adc_register(&adc_obj_list[i].esp32c3_adc_device, adc_obj_list[i].name, &esp_adc_ops, &adc_obj_list[i].adcx) == JEE_EOK)
        {
            LOGI(TAG, "%s register success", adc_obj_list[i].name);
        }
        else
        {
            LOGE(TAG, "%s register failed", adc_obj_list[i].name);
            result = -JEE_ERROR;
        }
    }
    return result;
}

INIT_PREV_EXPORT(jee_hw_adc_init);