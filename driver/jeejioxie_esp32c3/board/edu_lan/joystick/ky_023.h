#ifndef __KY_023_H__
#define __KY_023_H__

#include "stdint.h"

typedef struct
{
    int x_value;
    int y_value;
    uint8_t key_sta;
    uint8_t key_press_sta;
} joystick_t;

uint8_t joystick_upload(void);


typedef enum
{
    BTN_STATE_CLICK,
    BTN_STATE_DBCLICK,
    BTN_STATE_PRESSED,   
}button_state_t;


  /**
   * @brief       游戏遥感初始化 
   * 
   *   NOTE:      ADC初始化，配置传感器参数等
   * 
   */
void joystick_init(void);

  /**
   * @brief       获取当前游戏操纵杆的垂直按下状态.
   *
   * @return      True : 按下    False : 未按下  
   */
bool joystick_get_button_status(void);

  /**
   * @brief       获取按压模式 
   * 
   * @return      模式参数
   *  
   *    NOTE:    模式参数如下
   *             typedef enum
   *             {
   *                 BTN_STATE_CLICK,
   *                 BTN_STATE_DBCLICK,
   *                 BTN_STATE_PRESSED,   
   *             }button_state_t;
   *
   */
button_state_t joystick_get_button_mode(void);

  /**
   * @brief       获取当前X轴数值信息 
   * 
   * @return      X轴数值
   *
   */
int joystick_get_x_pos(void);

  /**
   * @brief       获取当前Y轴数值信息 
   * 
   * @return      Y轴数值
   *
   */
int joystick_get_y_pos(void);


  /**
   * @brief       上报按压模式.
   *  
   *   NOTE:      模式分为:单击，双击，长按
   *
   * @param[out]  *str - 输出字符串.
   *  
   *   NOTE:      字符串格式-如下 （单击:str2 = "Click"  双击：str2 = "DoubleClick" 长按 ：str2 = "LongPress"） 
   *             {
   *               sprintf(str, "{\"mode\":\"%s\"}", str2);
   *              }
   * 
   * @return      1 ：检测到按压模式   0：没有检测到按压模式
   * 
   */
uint8_t joystick_mode_receive_upload(char *str);

  /**
   * @brief       上报XY轴数据.
   *
   * @param[out]  *str - 输出字符串.
   *  
   *   NOTE:      字符串格式-如下 （X轴参数：joystick.x_value  Y轴参数：joystick.y_value） 
   *             {
   *                sprintf(str, "{\"positionX\":%d,\"positionY\":%d}", joystick.x_value, joystick.y_value);
   *              }
   * 
   * @return      1 ：XY轴不为同时0   0：XY轴同时为0
   * 
   */
uint8_t joystick_position_recevice(char *str);

  /**
   * @brief       获取当前所有信息 
   * 
   * @return      所有数值
   *
   */
void joystick_get_info(int *buf);

  /**
   * @brief      主动清除按键状态
   * 
   *   NOTE:  比如获取了一次 单击状态后 要主动清除按键状态记录
   */
void joystick_clean_mode(void);

#endif
