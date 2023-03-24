#include "sg90.h"
#include <stdio.h>
#include "hal_pwm.h"

#define TAG "SG90"

#define STEERING_GEAR_TIMER     (0)         //  LEDC_TIMER_0
#define STEERING_GEAR_MODE      (0)         //  LEDC_LOW_SPEED_MODE
#define STEERING_GEAR_OUTPUT_IO (9)         //  Define the output GPIO  9
#define STEERING_GEAR_CHANNEL   (0)         //  LEDC_CHANNEL_0
//#define STEERING_GEAR_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define STEERING_GEAR_DUTY (4095)                  // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define STEERING_GEAR_FREQUENCY (50)               // Frequency in Hertz. Set frequency at 5 kHz

steering_gear_config_t steering_gear_config;
jee_device_t sg90_pwm_dev = NULL;

static struct jee_pwm_configuration pwm_cfg = {
    .channel = STEERING_GEAR_CHANNEL,
    .speed_mode = STEERING_GEAR_MODE,
    .period = 1000*20000,    // 单位ns,这里等价于5khz
    .timer_num = STEERING_GEAR_TIMER,
    .pin_num = STEERING_GEAR_OUTPUT_IO
    };

void sg90Init(void)
{
    /* 
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = STEERING_GEAR_MODE,
        .timer_num = STEERING_GEAR_TIMER,
        .duty_resolution = STEERING_GEAR_DUTY_RES,
        .freq_hz = STEERING_GEAR_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = STEERING_GEAR_MODE,
        .channel = STEERING_GEAR_CHANNEL,
        .timer_sel = STEERING_GEAR_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = STEERING_GEAR_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    */
    
       // 查找设备
    sg90_pwm_dev = jee_device_find("pwm");
    if (sg90_pwm_dev == JEE_NULL)
    {
        printf("can not find pwm Model\n");
        return;
    }

    // 打开设备
    jee_err_t result = JEE_EOK;
    result = jee_device_open(sg90_pwm_dev, JEE_DEVICE_OFLAG_RDWR);
    if (result != JEE_EOK)
    {
        printf("can not open pwm device\n");
        return;
    }

    // 初始化设备

    jee_device_control(sg90_pwm_dev, PWM_CMD_INIT, (void *)&pwm_cfg);
    
    steering_gear_config.current_angle = 0;
    steering_gear_config.current_dirction = CLOCK_WISE;
    steeringGearSetAngle(steering_gear_config.current_angle);
    // water_pump_set_mode(SLOW);
    printf("sg90_init complete\n");
    // xTaskCreate(steering_gear_task, "water_pump_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}


#if 0
void steeringGearTask(void *arg)
{

    uint32_t value, counter = 0;
    int flag = 0;

    while (1)
    {
        if (counter < 180 && flag == 0)
        {
            counter++;
            // printf("counter  clear  ++++ ");
        }
        else if (counter == 180)
        {
            counter--;
            flag = 1;
        }
        else if (counter < 180 && flag == 1)
        {
            counter--;

            if (counter == 0)
            {
                flag = 0;
            }
        }
        printf("counter  %D  ++++ ", counter);

        hal_steering_gear_set_angle(counter);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
#endif


void steeringGearSetDirction(int valule)
{
    steering_gear_config.current_dirction = valule;
    printf("current_dirction  %d\r\n", steering_gear_config.current_dirction);
}

int steeringGearGetDirction(void)
{
    printf("current_dirction  %d\r\n", steering_gear_config.current_dirction);
    return steering_gear_config.current_dirction;
}
int steeringGearGetAngle(void)
{
    return steering_gear_config.current_angle;
}

void steeringGearGetDirctionAngle(uint32_t *pdata)
{
    pdata[0] = steering_gear_config.current_dirction;
    pdata[1] = steering_gear_config.current_angle;
}

void steeringGearSetAngle(int angle)
{
    
    printf("in   angle %d \n",angle);
    float duty;
    int conver_angle;
    steering_gear_config.current_angle = angle;
    if (steering_gear_config.current_dirction == CLOCK_WISE)
    {
        conver_angle = 180 - angle;
    }
    else
    {
        conver_angle = angle;
    }
    
    duty = ((float)2.0 / (float)180 * (float)conver_angle + 0.5) / ((float)20 / (float)8191);
    printf("duty  %f   angle %d \n", duty, angle);
    //ESP_ERROR_CHECK(ledc_set_duty(STEERING_GEAR_MODE, STEERING_GEAR_CHANNEL, duty));
    //ESP_ERROR_CHECK(ledc_update_duty(STEERING_GEAR_MODE, STEERING_GEAR_CHANNEL));
    // 操控设备:Set duty
    pwm_cfg.duty = duty;
    jee_device_control(sg90_pwm_dev, PWM_CMD_SET_DUTY, (void *)&pwm_cfg);
    jee_device_control(sg90_pwm_dev, PWM_CMD_UPDATE_DUTY, (void *)&pwm_cfg);

}




