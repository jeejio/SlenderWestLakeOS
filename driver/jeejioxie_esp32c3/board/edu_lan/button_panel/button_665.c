#include "button_665.h"
#include "hal_adc.h"
// #include "hal_gpio.h"
/*
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
*/
// #include "flexible_button.h"
#include "device.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "auto_init.h"

const char *TAG = "button_panel";

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 1   // Multisampling
/*
static const adc_channel_t channel = ADC_CHANNEL_3; // GPIO3
// ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static adc_cali_handle_t adc1_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
*/
#define BUTTON_665_ADC_DEV_NAME "adc1"
#define ADC_DEV_CHANNEL 3
static jee_adc_device_t button_665_adc_dev = NULL;

////////////////////////////////////////////////////////////////////////////
uint32_t adc_reading;
// Multisampling

// Convert adc_reading to voltage in mV
uint32_t voltage = 0;
bool button_flag = 0;
uint8_t key_press_num = 0;
uint8_t key_press_mode = 0;
uint8_t key_num = 0;
uint8_t key_mode = 0;
///////////////////////////////////////////////////////
#define FLEX_BTN_SCAN_FREQ_HZ 50 // How often flex_button_scan () is called
#define FLEX_MS_TO_SCAN_CNT(ms) (ms / (1000 / FLEX_BTN_SCAN_FREQ_HZ))

/* Multiple clicks interval, default 300ms */
#define MAX_MULTIPLE_CLICKS_INTERVAL (FLEX_MS_TO_SCAN_CNT(300))

typedef void (*flex_button_response_callback)(void *);

typedef struct flex_button
{
    struct flex_button *next;

    uint8_t (*usr_button_read)(void *);
    flex_button_response_callback cb;

    uint16_t scan_cnt;
    uint16_t click_cnt;
    uint16_t max_multiple_clicks_interval;

    uint16_t debounce_tick;
    uint16_t short_press_start_tick;
    uint16_t long_press_start_tick;
    uint16_t long_hold_start_tick;

    uint8_t id;
    uint8_t pressed_logic_level : 1;
    uint8_t event : 4;
    uint8_t status : 3;
} button_t;

#define EVENT_SET_CB(btn, evt)        \
    do                                \
    {                                 \
        btn->event = evt;             \
        if (btn->cb)                  \
            btn->cb((button_t *)btn); \
    } while (0)

#define BTN_IS_PRESSED(i) (g_btn_status_reg & (1 << i))

enum FLEX_BTN_STAGE
{
    BTN_STAGE_DEFAULT = 0,
    BTN_STAGE_DOWN = 1,
    BTN_STAGE_MULTIPLE_CLICK = 2
};

typedef uint32_t btn_type_t;

static button_t *btn_head = NULL;

static btn_type_t g_logic_level = (btn_type_t)0;

static btn_type_t g_btn_status_reg = (btn_type_t)0;

static uint8_t button_cnt = 0;

static void adc_key_read(void);
static int32_t flex_button_register(button_t *button)
{
    button_t *curr = btn_head;

    if (!button || (button_cnt > sizeof(btn_type_t) * 8))
    {
        return -1;
    }

    while (curr)
    {
        if (curr == button)
        {
            return -1; /* already exist. */
        }
        curr = curr->next;
    }

    /**
     * First registered button is at the end of the 'linked list'.
     * btn_head points to the head of the 'linked list'.
     */
    button->next = btn_head;
    button->status = BTN_STAGE_DEFAULT;
    button->event = BTN_PRESS_NONE;
    button->scan_cnt = 0;
    button->click_cnt = 0;
    button->max_multiple_clicks_interval = MAX_MULTIPLE_CLICKS_INTERVAL;
    btn_head = button;

    /**
     * First registered button, the logic level of the button pressed is
     * at the low bit of g_logic_level.
     */
    g_logic_level |= (button->pressed_logic_level << button_cnt);
    button_cnt++;

    return button_cnt;
}

/**
 * @brief Read all key values in one scan cycle
 *
 * @param void
 * @return none
 */
static void flex_button_read(void)
{
    uint8_t i;
    button_t *target;
    adc_key_read();
    // printf("v %d\n", voltage);
    /* The button that was registered first, the button value is in the low position of raw_data */
    btn_type_t raw_data = 0;

    for (target = btn_head, i = button_cnt - 1;
         (target != NULL) && (target->usr_button_read != NULL);
         target = target->next, i--)
    {
        raw_data = raw_data | ((target->usr_button_read)(target) << i);
    }

    g_btn_status_reg = (~raw_data) ^ g_logic_level;
}

/**
 * @brief Handle all key events in one scan cycle.
 *        Must be used after 'flex_button_read' API
 *
 * @param void
 * @return Activated button count
 */
static uint8_t flex_button_process(void)
{
    uint8_t i;
    uint8_t active_btn_cnt = 0;
    button_t *target;

    for (target = btn_head, i = button_cnt - 1; target != NULL; target = target->next, i--)
    {
        if (target->status > BTN_STAGE_DEFAULT)
        {
            target->scan_cnt++;
            if (target->scan_cnt >= ((1 << (sizeof(target->scan_cnt) * 8)) - 1))
            {
                target->scan_cnt = target->long_hold_start_tick;
            }
        }

        switch (target->status)
        {
        case BTN_STAGE_DEFAULT:    /* stage: default(button up) */
            if (BTN_IS_PRESSED(i)) /* is pressed */
            {
                target->scan_cnt = 0;
                target->click_cnt = 0;

                EVENT_SET_CB(target, BTN_PRESS_DOWN);

                /* swtich to button down stage */
                target->status = BTN_STAGE_DOWN;
            }
            else
            {
                target->event = BTN_PRESS_NONE;
            }
            break;

        case BTN_STAGE_DOWN:       /* stage: button down */
            if (BTN_IS_PRESSED(i)) /* is pressed */
            {
                if (target->click_cnt > 0) /* multiple click */
                {
                    if (target->scan_cnt > target->max_multiple_clicks_interval)
                    {
                        EVENT_SET_CB(target,
                                     target->click_cnt < BTN_PRESS_REPEAT_CLICK ? target->click_cnt : BTN_PRESS_REPEAT_CLICK);

                        /* swtich to button down stage */
                        target->status = BTN_STAGE_DOWN;
                        target->scan_cnt = 0;
                        target->click_cnt = 0;
                    }
                }
                else if (target->scan_cnt >= target->long_hold_start_tick)
                {
                    if (target->event != BTN_PRESS_LONG_HOLD)
                    {
                        EVENT_SET_CB(target, BTN_PRESS_LONG_HOLD);
                    }
                }
                else if (target->scan_cnt >= target->long_press_start_tick)
                {
                    if (target->event != BTN_PRESS_LONG_START)
                    {
                        EVENT_SET_CB(target, BTN_PRESS_LONG_START);
                    }
                }
                else if (target->scan_cnt >= target->short_press_start_tick)
                {
                    if (target->event != BTN_PRESS_SHORT_START)
                    {
                        EVENT_SET_CB(target, BTN_PRESS_SHORT_START);
                    }
                }
            }
            else /* button up */
            {
                if (target->scan_cnt >= target->long_hold_start_tick)
                {
                    EVENT_SET_CB(target, BTN_PRESS_LONG_HOLD_UP);
                    target->status = BTN_STAGE_DEFAULT;
                }
                else if (target->scan_cnt >= target->long_press_start_tick)
                {
                    EVENT_SET_CB(target, BTN_PRESS_LONG_UP);
                    target->status = BTN_STAGE_DEFAULT;
                }
                else if (target->scan_cnt >= target->short_press_start_tick)
                {
                    EVENT_SET_CB(target, BTN_PRESS_SHORT_UP);
                    target->status = BTN_STAGE_DEFAULT;
                }
                else
                {
                    /* swtich to multiple click stage */
                    target->status = BTN_STAGE_MULTIPLE_CLICK;
                    target->click_cnt++;
                }
            }
            break;

        case BTN_STAGE_MULTIPLE_CLICK: /* stage: multiple click */
            if (BTN_IS_PRESSED(i))     /* is pressed */
            {
                /* swtich to button down stage */
                target->status = BTN_STAGE_DOWN;
                target->scan_cnt = 0;
            }
            else
            {
                if (target->scan_cnt > target->max_multiple_clicks_interval)
                {
                    EVENT_SET_CB(target,
                                 target->click_cnt < BTN_PRESS_REPEAT_CLICK ? target->click_cnt : BTN_PRESS_REPEAT_CLICK);

                    /* swtich to default stage */
                    target->status = BTN_STAGE_DEFAULT;
                }
            }
            break;
        }

        if (target->status > BTN_STAGE_DEFAULT)
        {
            active_btn_cnt++;
        }
    }

    return active_btn_cnt;
}
static button_event_t flex_button_event_read(button_t *button)
{
    return (button_event_t)(button->event);
}
///////////////////////////////////////
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
static void adc_key_read(void)
{
    /************************以下是：esp接口-方式*****************************/
    /*
   uint8_t i;
   int adc_raw = 0;
   uint32_t adc = 0;
   // Multisampling
   for (int i = 0; i < NO_OF_SAMPLES; i++)
   {
       adc_oneshot_read(adc1_handle, channel, &adc_raw);
       adc += adc_raw;
   }

   adc /= (NO_OF_SAMPLES);

#if 1
   if (adc == 0)
   {
       //  printf("v %d\n",adc);
       adc_oneshot_read(adc1_handle, channel, &adc_raw);
       adc += adc_raw;
   }
   adc /= (NO_OF_SAMPLES);
#endif
   // Convert adc_reading to voltage in mV
   adc_cali_raw_to_voltage(adc1_cali_handle, adc, &voltage);
   */
    /************************以下是：jee-iomodel-方式************************/
    voltage = jee_adc_read(button_665_adc_dev, ADC_DEV_CHANNEL);
    if (voltage == 0)
   {
         voltage = jee_adc_read(button_665_adc_dev, ADC_DEV_CHANNEL);
   }

}

////////////////////////////////////////////////////////////////////////////

#define ENUM_TO_STR(e) (#e)

typedef enum
{
    USER_BUTTON_1 = 0,
    USER_BUTTON_2,
    USER_BUTTON_3,
    USER_BUTTON_4,
    USER_BUTTON_5,
    USER_BUTTON_6,
    USER_BUTTON_7,
    USER_BUTTON_8,
    USER_BUTTON_9,
    USER_BUTTON_MAX
} user_button_t;

static char *enum_event_string[] = {
    ENUM_TO_STR(BTN_PRESS_DOWN),
    ENUM_TO_STR(BTN_PRESS_CLICK),
    ENUM_TO_STR(BTN_PRESS_DOUBLE_CLICK),
    ENUM_TO_STR(BTN_PRESS_REPEAT_CLICK),
    ENUM_TO_STR(BTN_PRESS_SHORT_START),
    ENUM_TO_STR(BTN_PRESS_SHORT_UP),
    ENUM_TO_STR(BTN_PRESS_LONG_START),
    ENUM_TO_STR(BTN_PRESS_LONG_UP),
    ENUM_TO_STR(BTN_PRESS_LONG_HOLD),
    ENUM_TO_STR(BTN_PRESS_LONG_HOLD_UP),
    ENUM_TO_STR(BTN_PRESS_MAX),
    ENUM_TO_STR(FLEX_BTN_PRESS_NONE),
};

static char *enum_btn_id_string[] = {
    ENUM_TO_STR(USER_BUTTON_1),
    ENUM_TO_STR(USER_BUTTON_2),
    ENUM_TO_STR(USER_BUTTON_3),
    ENUM_TO_STR(USER_BUTTON_4),
    ENUM_TO_STR(USER_BUTTON_5),
    ENUM_TO_STR(USER_BUTTON_6),
    ENUM_TO_STR(USER_BUTTON_7),
    ENUM_TO_STR(USER_BUTTON_8),
    ENUM_TO_STR(USER_BUTTON_9),
    ENUM_TO_STR(USER_BUTTON_MAX),
};

static button_t user_button[USER_BUTTON_MAX];

static uint8_t common_btn_read(void *arg)
{
    uint8_t value = 1;
    // adc_key_read();
    button_t *btn = (button_t *)arg;
    switch (btn->id)
    {
    case USER_BUTTON_1:
        if (voltage <= 70)
        {
            value = 0;
        }

        break;
    case USER_BUTTON_2:
        if (voltage <= 750 + 70 && voltage >= 750 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_3:
        if (voltage <= 1200 + 70 && voltage >= 1200 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_4:
        if (voltage <= 250 + 70 && voltage >= 250 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_5:
        if (voltage <= 900 + 70 && voltage >= 900 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_6:
        if (voltage <= 1350 + 70 && voltage >= 1350 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_7:
        if (voltage <= 520 + 70 && voltage >= 520 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_8:
        if (voltage <= 1050 + 80 && voltage >= 1050 - 70)
        {
            value = 0;
        }
        break;
    case USER_BUTTON_9:
        if (voltage <= 1550 + 70 && voltage >= 1550 - 120)
        {
            value = 0;
        }
        break;

    default:
        break;
    }

    return value;
}

static void common_btn_evt_cb(void *arg)
{
    button_t *btn = (button_t *)arg;
    /*
        printf("id: [%d - %s]  event: [%d - %s]  repeat: %d\r\n",
               btn->id, enum_btn_id_string[btn->id],
               btn->event, enum_event_string[btn->event],
               btn->click_cnt);
    */
    key_num = btn->id + 1;
    if (btn->event <= 0)
    {
        key_mode = 0;
    }
    else
    {
        key_mode = btn->event - 1;
    }

    if (btn->event == BTN_PRESS_CLICK || btn->event == BTN_PRESS_DOUBLE_CLICK || btn->event == BTN_PRESS_REPEAT_CLICK || btn->event == BTN_PRESS_SHORT_UP || btn->event == BTN_PRESS_LONG_UP || btn->event == BTN_PRESS_LONG_HOLD_UP)
    {
        key_press_num = btn->id + 1;
        key_press_mode = btn->event;
        button_flag = 1;
        // key_num=btn->id+1;
    }

    /*
    if ((flex_button_event_read(&user_button[USER_BUTTON_1]) == FLEX_BTN_PRESS_CLICK) && (flex_button_event_read(&user_button[USER_BUTTON_2]) == FLEX_BTN_PRESS_CLICK))
    {
        printf("[combination]: button 1 and button 2\r\n");
    }*/
}

static void user_button_init(void)
{
    int i;
    memset(&user_button[0], 0x0, sizeof(user_button));
    for (i = 0; i < USER_BUTTON_MAX; i++)
    {
        user_button[i].id = i;
        user_button[i].usr_button_read = common_btn_read;
        user_button[i].cb = common_btn_evt_cb;
        user_button[i].pressed_logic_level = 0;
        user_button[i].short_press_start_tick = FLEX_MS_TO_SCAN_CNT(500);
        user_button[i].long_press_start_tick = FLEX_MS_TO_SCAN_CNT(5000);
        user_button[i].long_hold_start_tick = FLEX_MS_TO_SCAN_CNT(9000);

        flex_button_register(&user_button[i]);
    }
}

static uint8_t flex_button_scan(void)
{
    flex_button_read();
    return flex_button_process();
}

static void key_task(void *arg)
{
    while (1)
    {

        flex_button_scan();
        vTaskDelay(pdMS_TO_TICKS((1000 / FLEX_BTN_SCAN_FREQ_HZ))); // 20 ms
    }
}

void button_panel_init(void)
{
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
    button_665_adc_dev = (jee_adc_device_t)jee_device_find(BUTTON_665_ADC_DEV_NAME);
    result_str = (button_665_adc_dev == NULL) ? "failure" : "success";
    printf("probe %s \n", result_str);
    jee_adc_enable(button_665_adc_dev, 1 << ADC_DEV_CHANNEL);

    user_button_init();

    xTaskCreate(key_task, "key_task", 2048, NULL, 10, NULL);
}

uint8_t button_panel_get_button_num(void) //
{

    return key_press_num;
}

int button_panel_get_button_mode(void)
{

    return key_press_mode;
}
void button_panel_get_button_info(int *buf)
{
    if (button_flag == 1)
    {
        button_flag = 0;
        buf[0] = key_press_num;
        buf[1] = key_press_mode;
    }
    else
    {
        buf[0] = 0;
        buf[1] = 0;
    }
}

#if 1
uint32_t button_panel_upload(char *buff)
{
    if (button_flag == 1)
    {
        button_flag = 0;
        sprintf(buff, "{\"code\":1,\"value\":%d,\"buttonMode\":%d}", key_press_num, key_press_mode);
        return strlen(buff);
    }
    else
    {
        return 0;
    }
}
#endif
