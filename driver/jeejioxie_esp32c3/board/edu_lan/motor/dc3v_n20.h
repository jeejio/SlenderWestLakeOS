#ifndef __DC3V_N20_H__
#define __DC3V_N20_H__

#include "stdint.h"
#include "stdbool.h"
typedef enum
{
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_LEFT,
    MOTOR_DIR_RIGHT,
    MOTOR_DIR_FORWARD_RIGHT,
    MOTOR_DIR_FORWARD_LEFT,
    MOTOR_DIR_BACKWARD_RIGHT,
    MOTOR_DIR_BACKWARD_LEFT,
    MOTOR_INIT,
} motor_dir_t;

typedef enum
{
    MOTOR_SPEED_SLOW = 0,
    MOTOR_SPEED_MIDDLE,
    MOTOR_SPEED_FAST

} motor_speed_t;

 /**
   * @brief       减速电机初始化函数
   *
   * NOTE:        在其他 减速电机 相关函数使用前调用，只调用一次  
   * 
   */
void motor_init(void);

 /**
   * @brief      设置电机速度
   *
   * NOTE:       一共两个电机，设置的是需要工作的电机的速度，
   *             如两个都要工作，则设置的是两个电机的速度
   * 
   * @param[in]   speed 设置电机的速度 
   *            
   *  
   */
void motor_set_speed(int speed);

 /**
   * @brief      获取电机速度
   *
   * NOTE:       一共两个电机，获取的是需要工作的电机的速度，
   *             需要工作的电机的速度总是同样的
   * 
   * @return    电机的速度
 
   */
motor_speed_t motor_get_speed(void);

 /**
   * @brief      设置电机的旋转方向
   *
   * NOTE:       向前、向后是两个电机共同工作，向左、向右是单个电机工作进行转向
   * 
   * @param[in]   dir - 要设置的电机的方向
   *             
   */
void motor_set_direction(int dir);

 /**
   * @brief      获取电机的旋转方向
   *
   * NOTE:       向前、向后是两个电机共同工作，向左、向右是单个电机工作进行转向
   * 
   * @return     电机的方向
   *             
   */
motor_dir_t motor_get_direction(void);

 /**
   * @brief      获取电机开关
   *
   * NOTE:       当开关为关闭时，设置方向、速度电机不会启动
   *           
   * @return     电机的开关  1:开，0:关  
   */
int motor_get_switch_status(void);

 /**
   * @brief      设置电机开关
   *
   * NOTE:       电机开启后将按照设置好的方向、速度启动
   * 
   * @param[in]  onff  0:关闭，1:开启
   * 
   *
   */
void motor_set_switch(int onff);

 /**
   * @brief      获取电机信息
   *
   * NOTE:       信息包括：速度，方向，开关状态
   *           
   * @param[out]   返回信息
   */
void motor_get_info(int *buf);

#endif
