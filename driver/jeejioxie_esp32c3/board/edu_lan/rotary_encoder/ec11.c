#include "ec11.h"
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/device.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <esp_timer.h>
#include "hal_gpio.h"

#define MUTEX_TIMEOUT 10
#define CONFIG_RE_BTN_PRESSED_LEVEL_0
#ifdef CONFIG_RE_BTN_PRESSED_LEVEL_0
#define BTN_PRESSED_LEVEL 0
#else
#define BTN_PRESSED_LEVEL 1
#endif

#define TAG "EC11-TAG"

#define ESP_TIME_USE 1

#define CONFIG_RE_BTN_LONG_PRESS_TIME_US 1000 * 1000
#define CONFIG_RE_BTN_DEAD_TIME_US 10000 /*10ms*/
#define CONFIG_RE_INTERVAL_US 10000      /*10ms*/
#define GPIO_NUM_MAX 31

#define CONFIG_RE_MAX 1

typedef enum
{
    RE_BTN_RELEASED = 0,    //!< Button currently released
    RE_BTN_PRESSED = 1,     //!< Button currently pressed
    RE_BTN_LONG_PRESSED = 2 //!< Button currently long pressed
} RotaryEncoderBtnState_t;

/**
 * Rotary encoder descriptor
 */
typedef struct
{
    jee_base_t pin_a, pin_b, pin_btn; //!< Encoder pins. pin_btn can be >= GPIO_NUM_MAX if no button used
    uint8_t code;
    uint16_t store;
    size_t index;
    uint64_t btn_pressed_time_us;
    RotaryEncoderBtnState_t btn_state;
} RotaryEncoder_t;

/**
 * Event type
 */
typedef enum
{
    RE_ET_CHANGED = 0,      //!< Encoder turned
    RE_ET_BTN_RELEASED,     //!< Button released
    RE_ET_BTN_PRESSED,      //!< Button pressed
    RE_ET_BTN_LONG_PRESSED, //!< Button long pressed (press time (us) > RE_BTN_LONG_PRESS_TIME_US)
    RE_ET_BTN_CLICKED       //!< Button was clicked
} rotary_encoder_event_type_t;

/**
 * Event
 */
typedef struct
{
    rotary_encoder_event_type_t type; //!< Event type
    RotaryEncoder_t *sender;          //!< Pointer to descriptor
    int32_t diff;                     //!< Difference between new and old positions (only if type == RE_ET_CHANGED)
} rotary_encoder_event_t;

static RotaryEncoder_t *encs[CONFIG_RE_MAX] = {0};
static const int8_t valid_states[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
static SemaphoreHandle_t mutex;
static QueueHandle_t _queue;

#define GPIO_BIT(x) ((x) < 32 ? BIT(x) : ((uint64_t)(((uint64_t)1) << (x))))
#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

inline static void read_encoder(RotaryEncoder_t *re)
{
    rotary_encoder_event_t ev = {
        .sender = re};
    if (re->pin_btn < GPIO_NUM_MAX)
    {
        do
        {
            if (re->btn_state == RE_BTN_PRESSED && re->btn_pressed_time_us < CONFIG_RE_BTN_DEAD_TIME_US)
            {
                // Dead time
                re->btn_pressed_time_us += CONFIG_RE_INTERVAL_US;
                break;
            }

            // read button state
            if (jee_pin_read(re->pin_btn) == BTN_PRESSED_LEVEL)
            {
                if (re->btn_state == RE_BTN_RELEASED)
                {
                    // first press
                    re->btn_state = RE_BTN_PRESSED;
                    re->btn_pressed_time_us = 0;
                    ev.type = RE_ET_BTN_PRESSED;
                    xQueueSendToBack(_queue, &ev, 0);
                    break;
                }

                re->btn_pressed_time_us += CONFIG_RE_INTERVAL_US;

                if (re->btn_state == RE_BTN_PRESSED && re->btn_pressed_time_us >= CONFIG_RE_BTN_LONG_PRESS_TIME_US)
                {
                    // Long press
                    re->btn_state = RE_BTN_LONG_PRESSED;
                    ev.type = RE_ET_BTN_LONG_PRESSED;

                    xQueueSendToBack(_queue, &ev, 0);
                }
            }
            if (jee_pin_read(re->pin_btn) == 1)
                if (re->btn_state != RE_BTN_RELEASED)
                {
                    bool clicked = re->btn_state == RE_BTN_PRESSED;
                    // released
                    re->btn_state = RE_BTN_RELEASED;
                    ev.type = RE_ET_BTN_RELEASED;

                    xQueueSendToBack(_queue, &ev, 0);
                    if (clicked)
                    {
                        ev.type = RE_ET_BTN_CLICKED;
                        xQueueSendToBack(_queue, &ev, 0);
                    }
                }

        } while (0);
    }
    re->code <<= 2;
    re->code |= jee_pin_read(re->pin_a);
    re->code |= jee_pin_read(re->pin_b) << 1;
    re->code &= 0xf;

    if (!valid_states[re->code])
    {
        return;
    }
    int8_t inc = 0;
    re->store = (re->store << 4) | re->code;

    if (re->store == 0xe817)
        inc = 1;
    if (re->store == 0xd42b)
        inc = -1;

    if (inc)
    {
        ev.type = RE_ET_CHANGED;
        ev.diff = inc;
        xQueueSendToBack(_queue, &ev, 0);
    }
}
/*
static void timer_handler(void *arg)
{
    if (!xSemaphoreTake(mutex, 0))
    {
        return;
    }
    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (encs[i])
        {
            read_encoder(encs[i]);
        }

    xSemaphoreGive(mutex);
}

static const esp_timer_create_args_t timer_args = {
    .name = "__encoder__",
    .arg = NULL,
    .callback = timer_handler,
    .dispatch_method = ESP_TIMER_TASK};

static esp_timer_handle_t timer;
*/
TimerHandle_t xRotaryEncoderTimers;
static void timer_handler(TimerHandle_t pxTimer)
{
    if (!xSemaphoreTake(mutex, 0))
    {
        return;
    }
    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (encs[i])
        { 
            read_encoder(encs[i]);
        }

    xSemaphoreGive(mutex);
}

esp_err_t rotary_encoder_init(QueueHandle_t queue)
{
    CHECK_ARG(queue);
    _queue = queue;

    mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(mutex);

    if (!mutex)
    {
        printf("Failed to create mutex\n");
        return ESP_ERR_NO_MEM;
    }
    /*
    CHECK(esp_timer_create(&timer_args, &timer));
    CHECK(esp_timer_start_periodic(timer, CONFIG_RE_INTERVAL_US));
    */
    xRotaryEncoderTimers = xTimerCreate("xRotaryEncoderTimers", // Just a text name, not used by the kernel.
                                        (1),                   // The timer period in ticks.
                                        pdTRUE,                 // The timers will auto-reload themselves when they expire.
                                        0,                      // Assign each timer a unique id equal to its array index.
                                        timer_handler           // Each timer calls the same callback when it expires.
    );
    if (xTimerStart(xRotaryEncoderTimers, 0) != pdPASS)
    {
        // The timer could not be set into the Active state.
    }
    printf("Initialization complete, timer interval: %dms \n", CONFIG_RE_INTERVAL_US / 1000);
    return ESP_OK;
}

esp_err_t rotary_encoder_add(RotaryEncoder_t *re)
{
    CHECK_ARG(re);
    if (!xSemaphoreTake(mutex, MUTEX_TIMEOUT))
    {
        printf("Failed to take mutex \n");
        return ESP_ERR_INVALID_STATE;
    }

    bool ok = false;
    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (!encs[i])
        {
            re->index = i;
            encs[i] = re;
            ok = true;
            break;
        }
    if (!ok)
    {
        printf("Too many encoders \n");
        xSemaphoreGive(mutex);
        return ESP_ERR_NO_MEM;
    }

    // setup GPIO
    /*
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(gpio_config_t));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_BIT(re->pin_a) | GPIO_BIT(re->pin_b);
    if (re->pin_btn < GPIO_NUM_MAX)
        io_conf.pin_bit_mask |= GPIO_BIT(re->pin_btn);
    CHECK(gpio_config(&io_conf));
    */
    jee_pin_mode(re->pin_a, PIN_MODE_INPUT);
    jee_pin_mode(re->pin_b, PIN_MODE_INPUT);

    re->btn_state = RE_BTN_RELEASED;
    re->btn_pressed_time_us = 0;

    xSemaphoreGive(mutex);

    printf("Added rotary encoder %d, A: %ld, B: %ld, BTN: %ld \n", re->index, re->pin_a, re->pin_b, re->pin_btn);
    return ESP_OK;
}

esp_err_t rotary_encoder_remove(RotaryEncoder_t *re)
{
    CHECK_ARG(re);
    if (!xSemaphoreTake(mutex, MUTEX_TIMEOUT))
    {
        printf("Failed to take mutex \n");
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < CONFIG_RE_MAX; i++)
        if (encs[i] == re)
        {
            encs[i] = NULL;
            printf("Removed rotary encoder %d \n", i);
            xSemaphoreGive(mutex);
            return ESP_OK;
        }

    printf("Unknown encoder \n");
    xSemaphoreGive(mutex);
    return ESP_ERR_NOT_FOUND;
}

// Connect common encoder pin to ground
// Clockwise jianxiao Counterclockwise zengja
#define RE_A_GPIO 2
#define RE_B_GPIO 9
#define RE_BTN_GPIO 3
#define EV_QUEUE_LEN 5

static QueueHandle_t event_queue;
static RotaryEncoder_t re;
bool RotationDirection = 0;
int32_t level = 0;
bool encoder_change_flag = 0;

bool buton_state = 0; // 1-dowm 0-rese
void encoder_task(void *arg)
{
    // Create event queue for rotary encoders
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));
    // Add one encoder
    memset(&re, 0, sizeof(RotaryEncoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));
    rotary_encoder_event_t e;
    int32_t val = 0;
    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        switch (e.type)
        {
        case RE_ET_BTN_PRESSED:
            buton_state = 1;
            printf("Button pressed \n");
            break;
        case RE_ET_BTN_RELEASED:
            printf("Button released \n");
            buton_state = 0;
            break;
        case RE_ET_BTN_CLICKED:
            printf("Button clicked \n");
            break;
        case RE_ET_BTN_LONG_PRESSED:
            printf("Long pressed button \n");
            break;
        case RE_ET_CHANGED:
            val += e.diff;

            if (val - level > 0)
            {
                RotationDirection = false;
            }
            else
            {
                RotationDirection = true; // Clockwise
            }
            if (val >= 20)
            {
                val = 20;
            }
            else if (val <= -1)
            {
                val = 0;
            }
            level = val;
            encoder_change_flag = 1;
            printf("Value = %ld \n", val);
            break;
        default:
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void encoderInit(void)
{
    /*
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 8);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(8, 0);
    */
    printf("encoder_init \n");
    jee_pin_mode(8, PIN_MODE_OUTPUT);
    jee_pin_write(8, PIN_LOW);
    xTaskCreate(encoder_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}

/*接口*/
int encoderGetRotationDirection(void)
{
    return RotationDirection;
}

int encoderGetRelativePos(void)
{
    return level;
}

int encoderIsPressed(void)
{
    return buton_state;
}

void encoderGetDirectinPressedRelativePos(uint32_t *pdata)
{
    pdata[0]=level;
    pdata[1]=buton_state;
    pdata[2]=RotationDirection;
}

uint8_t encoderUpload(char *str)
{

    static int32_t level_old = 0;
    static bool buton_state_old = 0;
    if (level_old != level || buton_state_old != buton_state || encoder_change_flag==1)
    {
        level_old = level;
        buton_state_old = buton_state;
        encoder_change_flag=0;
        char *str1 = NULL;
        char *str2 = NULL;
        if (buton_state == 1)
        {
            str1 = "ture";
        }
        else
        {
            str1 = "false";
        }
        if (RotationDirection == true)
        {
            str2 = "Clockwise";
        }
        else
        {
            str2 = "Counterclockwise";
        }
        sprintf(str, "{\"isPressed\":\"%s\",\"rotationDirection\":\"%s\",\"position\":%ld}", str1, str2, level);
        printf("hal_encoder_upload %s\n", str);
        return 1;
    }

    return 0;
}
