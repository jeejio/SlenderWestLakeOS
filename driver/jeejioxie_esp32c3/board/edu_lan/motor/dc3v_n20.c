#include "dc3v_n20.h"
//#include "driver/ledc.h"
//#include "driver/gpio.h"
#include "esp_err.h"
#include "hal_pwm.h"
#include "hal_gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WATER_GPIO_OUTPUT 2
#define WATER_GPIO_OUTPUT_PIN_SEL ((1ULL << WATER_GPIO_OUTPUT))

#define WATER_GPIO_OUTPUT2 3
#define WATER_GPIO_OUTPUT_PIN_SEL2 ((1ULL << WATER_GPIO_OUTPUT2))

#define LEDC_MODE 0   /*!< LEDC high speed speed_mode */
#define LEDC_DUTY_RES 13 // Set duty resolution to 13 bits
#define LEDC_DUTY (4095)                // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (50)             // Frequency in Hertz. Set frequency at 5 kHz
#define TARGET_MCPWM_UNIT MCPWM_UNIT_0

//

#define LEDC_TIMER_FOR2 0
#define LEDC_OUTPUT_IO_FOR2 (8) // Define the output GPIO
#define LEDC_CHANNEL_FOR2 1

#define LEDC_TIMER_FOR1 1
#define LEDC_OUTPUT_IO_FOR1 (9) // Define the output GPIO
#define LEDC_CHANNEL_FOR1 2

#define LEDC_TIMER_BACK2 2
#define LEDC_OUTPUT_IO_BACK2 (2) // Define the output GPIO
#define LEDC_CHANNEL_BACK2 3

#define LEDC_TIMER_BACK1 3
#define LEDC_OUTPUT_IO_BACK1 (3) // Define the output GPIO
#define LEDC_CHANNEL_BACK1 4


#define FAST_SPEED (8190)
#define MIDDLE_SPEED (8190 * 0.7)
#define SLOW_SPEED (8190 * 0.4)

static struct jee_pwm_configuration pwm_cfg_for1 = {
    .channel = LEDC_CHANNEL_FOR1,
    .speed_mode = LEDC_MODE,
    .period = 1000 * 20000, // 单位ns,这里等价于5khz
    .timer_num = LEDC_TIMER_FOR1,
    .pin_num = LEDC_OUTPUT_IO_FOR1};
static struct jee_pwm_configuration pwm_cfg_for2 = {
    .channel = LEDC_CHANNEL_FOR2,
    .speed_mode = LEDC_MODE,
    .period = 1000 * 20000, // 单位ns,这里等价于5khz
    .timer_num = LEDC_TIMER_FOR2,
    .pin_num = LEDC_OUTPUT_IO_FOR2};
static struct jee_pwm_configuration pwm_cfg_back1 = {
    .channel = LEDC_CHANNEL_BACK1,
    .speed_mode = LEDC_MODE,
    .period = 1000 * 20000, // 单位ns,这里等价于5khz
    .timer_num = LEDC_TIMER_BACK1,
    .pin_num = LEDC_OUTPUT_IO_BACK1};
static struct jee_pwm_configuration pwm_cfg_back2 = {
    .channel = LEDC_CHANNEL_BACK2,
    .speed_mode = LEDC_MODE,
    .period = 1000 * 20000, // 单位ns,这里等价于5khz
    .timer_num = LEDC_TIMER_BACK2,
    .pin_num = LEDC_OUTPUT_IO_BACK2};
static jee_device_t motor_pwm_dev = NULL;

static uint32_t motor_duty = SLOW_SPEED;
static motor_dir_t motor_dir = MOTOR_INIT;
static motor_speed_t motor_speed = MOTOR_SPEED_SLOW;

typedef struct
{
    /* data */
    uint8_t change_flag;
    uint32_t now_duty[4];
    uint32_t target_duty[4];
    uint8_t lock_flag;
    uint32_t change_duty[4];

} motor_duty_set_t;
motor_duty_set_t motor_duty_set;
uint8_t OnOff_flag = 0;

void set_motor_speed_mode(void)
{
#if 1
    // printf("motor_dir %d\n",motor_dir);

    if (motor_dir == MOTOR_DIR_FORWARD)
    {
// printf("motor_dir 1\n");
#if 1
        motor_duty_set.target_duty[0] = motor_duty;
        motor_duty_set.target_duty[1] = motor_duty;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
#endif
        //  printf("motor_dir 11\n");
    }
    else if (motor_dir == MOTOR_DIR_BACKWARD)
    {
        motor_duty_set.target_duty[0] = 0;
        motor_duty_set.target_duty[1] = 0;
        motor_duty_set.target_duty[2] = motor_duty;
        motor_duty_set.target_duty[3] = motor_duty;
    }
    else if (motor_dir == MOTOR_DIR_LEFT)
    {
        // printf("motor_dir 3\n");
        motor_duty_set.target_duty[0] = motor_duty;
        motor_duty_set.target_duty[1] = 0;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
    }
    else if (motor_dir == MOTOR_DIR_RIGHT)
    {
        // printf("motor_dir 4\n");
        motor_duty_set.target_duty[0] = 0;
        motor_duty_set.target_duty[1] = motor_duty;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
    }
    else if (motor_dir == MOTOR_DIR_FORWARD_RIGHT)
    {
        motor_duty_set.target_duty[0] = motor_duty / 2;
        motor_duty_set.target_duty[1] = motor_duty;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
    }
    else if (motor_dir == MOTOR_DIR_FORWARD_LEFT)
    {
        motor_duty_set.target_duty[0] = motor_duty;
        motor_duty_set.target_duty[1] = motor_duty / 2;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
    }
    else if (motor_dir == MOTOR_DIR_BACKWARD_RIGHT)
    {
        motor_duty_set.target_duty[0] = 0;
        motor_duty_set.target_duty[1] = 0;
        motor_duty_set.target_duty[2] = motor_duty / 2;
        motor_duty_set.target_duty[3] = motor_duty;
    }
    else if (motor_dir == MOTOR_DIR_BACKWARD_LEFT)
    {
        motor_duty_set.target_duty[0] = 0;
        motor_duty_set.target_duty[1] = 0;
        motor_duty_set.target_duty[2] = motor_duty;
        motor_duty_set.target_duty[3] = motor_duty / 2;
    }
    else if (motor_dir == MOTOR_INIT)
    {
        // printf("motor_dir 4\n");
        motor_duty_set.target_duty[0] = 0;
        motor_duty_set.target_duty[1] = 0;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
    }
    if (OnOff_flag == 0)
    {
        motor_duty_set.target_duty[0] = 0;
        motor_duty_set.target_duty[1] = 0;
        motor_duty_set.target_duty[2] = 0;
        motor_duty_set.target_duty[3] = 0;
    }
#endif
    /*
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FOR1, motor_duty));
      // Update duty to apply the new value
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FOR1));

      ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FOR2, 0));
      // Update duty to apply the new value
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FOR2));

      ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BACK1, 0));
      // Update duty to apply the new value
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BACK1));

      ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BACK2, 0));
      // Update duty to apply the new value
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BACK2));
      */
}
void change_duty_func(uint8_t count, uint32_t duty)
{
    if (count == 0)
    {
        /************************以下是：esp接口-方式*****************************/
        /*
        printf("count == 0\n");
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FOR1, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FOR1));
        */
        /************************以下是：jee-iomodel-方式************************/

        pwm_cfg_for1.duty = duty;
        jee_device_control(motor_pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg_for1);
        jee_device_control(motor_pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg_for1);
        

    }
    else if (count == 1)
    {
        /************************以下是：esp接口-方式*****************************/
        /*
        printf("count == 1\n");
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FOR2, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FOR2));
        */
        /************************以下是：jee-iomodel-方式************************/ 
        pwm_cfg_for2.duty = duty;
        jee_device_control(motor_pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg_for2);
        jee_device_control(motor_pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg_for2);
        
    }
    else if (count == 2)
    {
        /************************以下是：esp接口-方式*****************************/
        /*
        printf("count == 2\n");        
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BACK1, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BACK1));
        */
        /************************以下是：jee-iomodel-方式************************/ 
        
        pwm_cfg_back1.duty = duty;
        jee_device_control(motor_pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg_back1);
        jee_device_control(motor_pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg_back1);
        
    }
    else if (count == 3)
    {
        /************************以下是：esp接口-方式*****************************/
        /*
        printf("count == 3\n");
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BACK2, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BACK2));
        */
        /************************以下是：jee-iomodel-方式************************/ 
        
        pwm_cfg_back2.duty = duty;
        jee_device_control(motor_pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg_back2);
        jee_device_control(motor_pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg_back2);
        
    }
}
void motor_duty_task(void *arg)
{
    uint8_t i = 0;

    while (1)
    {
        if (motor_duty_set.change_flag == 1)
        {
            printf("duty change!11\n");
            motor_duty_set.change_flag = 0;

            set_motor_speed_mode();

            motor_duty_set.lock_flag = 1;
#if 1
            for (i = 0; i < 4; i++)
            {
                if (motor_duty_set.now_duty[i] > motor_duty_set.target_duty[i])
                {
                    motor_duty_set.change_duty[i] = (motor_duty_set.now_duty[i] - motor_duty_set.target_duty[i]) / 10;
                    if (motor_duty_set.change_duty[i] == 0)
                        motor_duty_set.change_duty[i] = 1;
                }
                else if (motor_duty_set.now_duty[i] < motor_duty_set.target_duty[i])
                {
                    motor_duty_set.change_duty[i] = (motor_duty_set.now_duty[i] - motor_duty_set.target_duty[i]) / 10;
                    if (motor_duty_set.change_duty[i] == 0)
                        motor_duty_set.change_duty[i] = 1;
                }
                else
                {
                    motor_duty_set.change_duty[i] = 0;
                }
                // printf("n %d,t %d\n",i,motor_duty_set.now_duty[i],motor_duty_set.target_duty[i]);
            }
#endif
        }
        if (motor_duty_set.lock_flag == 1)
        {
#if 1
            uint8_t flag = 0;
            for (i = 0; i < 4; i++)
            {

                if (motor_duty_set.now_duty[i] > motor_duty_set.target_duty[i])
                {
                    flag = 1;
                    if (motor_duty_set.now_duty[i] - motor_duty_set.target_duty[i] < motor_duty_set.change_duty[i] * 2)
                    {
                        motor_duty_set.now_duty[i] = motor_duty_set.target_duty[i];
                    }
                    else
                    {
                        motor_duty_set.now_duty[i] -= motor_duty_set.change_duty[i];
                    }
                    change_duty_func(i, motor_duty_set.now_duty[i]);
                }
                else if (motor_duty_set.now_duty[i] < motor_duty_set.target_duty[i])
                {
                    flag = 1;
                    if (motor_duty_set.target_duty[i] - motor_duty_set.now_duty[i] < motor_duty_set.change_duty[i] * 2)
                    {
                        motor_duty_set.now_duty[i] = motor_duty_set.target_duty[i];
                    }
                    else
                    {
                        motor_duty_set.now_duty[i] += motor_duty_set.change_duty[i];
                    }
                    change_duty_func(i, motor_duty_set.now_duty[i]);
                }
            }
            if (flag == 0)
            {
                motor_duty_set.lock_flag = 0;
                printf("duty change over!\n");
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void motor_init(void)
{
    /************************以下是：esp接口-方式*****************************/
    /*
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER_FOR1,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_FOR1,
        .timer_sel = LEDC_TIMER_FOR1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_FOR1,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};

    ledc_timer.timer_num = LEDC_TIMER_FOR1;
    ledc_channel.channel = LEDC_CHANNEL_FOR1;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_FOR1;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_timer.timer_num = LEDC_TIMER_FOR2;
    ledc_channel.channel = LEDC_CHANNEL_FOR2;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_FOR2;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_timer.timer_num = LEDC_TIMER_BACK1;
    ledc_channel.channel = LEDC_CHANNEL_BACK1;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_BACK1;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_timer.timer_num = LEDC_TIMER_BACK2;
    ledc_channel.channel = LEDC_CHANNEL_BACK2;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_BACK2;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    */
    /************************以下是：jee-iomodel-方式************************/ 
    
    // 查找设备
    motor_pwm_dev = jee_device_find("pwm");
    if (motor_pwm_dev == JEE_NULL)
    {
        printf("can not find pwm Model\n");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(motor_pwm_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pwm device\n");
        return;
    }

    // 初始化设备
    jee_device_control(motor_pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg_for1);
    jee_device_control(motor_pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg_for2);
    jee_device_control(motor_pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg_back1);
    jee_device_control(motor_pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg_back2);
    
    motor_duty_set.now_duty[0] = 0;
    motor_duty_set.now_duty[1] = 0;
    motor_duty_set.now_duty[2] = 0;
    motor_duty_set.now_duty[3] = 0;

    memset(&motor_duty_set, 0, sizeof(motor_duty_set));
    motor_duty_set.change_flag = 1;

    xTaskCreate(motor_duty_task, "motor_duty_task", 2048, NULL, configMAX_PRIORITIES, NULL);
}

void motor_set_speed(int speed)
{
    if (speed > 3)
        return;

    motor_speed = speed;
    if (OnOff_flag == 1)
    {
        if (motor_speed == MOTOR_SPEED_SLOW)
        {
            motor_duty = SLOW_SPEED;
        }
        else if (motor_speed == MOTOR_SPEED_MIDDLE)
        {
            motor_duty = MIDDLE_SPEED;
        }
        else if (motor_speed == MOTOR_SPEED_FAST)
        {
            motor_duty = FAST_SPEED;
        }
        motor_duty_set.change_flag = 1;
    }
}

motor_speed_t motor_get_speed(void)
{

    return motor_speed;
}

void motor_set_direction(int dir)
{
    if (dir < 9)
    {
        motor_dir = dir;
        if (OnOff_flag == 1)
            motor_duty_set.change_flag = 1;
    }
}

motor_dir_t motor_get_direction(void)
{
    return motor_dir;
}

int motor_get_switch_status(void)
{

    return OnOff_flag;
}

void motor_set_switch(int onff)
{
    OnOff_flag = onff;
    if (OnOff_flag == 1)
    {
        if (OnOff_flag == 1)
        {
            if (motor_speed == MOTOR_SPEED_SLOW)
            {
                motor_duty = SLOW_SPEED;
            }
            else if (motor_speed == MOTOR_SPEED_MIDDLE)
            {
                motor_duty = MIDDLE_SPEED;
            }
            else if (motor_speed == MOTOR_SPEED_FAST)
            {
                motor_duty = FAST_SPEED;
            }
            motor_duty_set.change_flag = 1;
        }
    }
    else
    {
        motor_dir = MOTOR_INIT;
        motor_duty_set.change_flag = 1;
    }
}

void motor_get_info(int *buf)
{
    buf[0] = motor_speed;
    buf[1] = motor_dir;
    buf[2] = OnOff_flag;
}
