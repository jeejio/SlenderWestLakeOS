#ifndef __TAL_MOTOR_H__
#define __TAL_MOTOR_H__

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
} MotorDirection_t;

typedef enum
{
    MOTOR_SPEED_SLOW = 0,
    MOTOR_SPEED_MIDDLE,
    MOTOR_SPEED_FAST

} MotorSpeed_t;

/**
 * @brief       设置开关
 * 
 * @param[in]   开关值
 */
void vTalMotorSetOnoff(uint8_t onoff);

/**
 * @brief       设置方向
 * 
 * @param[in]   方向
 */
void vTalMotorSetRotationDirection(MotorDirection_t direction);

/**
 * @brief       设置速度
 * 
 * @param[in]   速度
 */
void vTalMotorSetSpinVelocityLevel(MotorSpeed_t speed);

/**
 * @brief       获取马达所有信息
 * 
 * @return      信息
 */
int * lTalMotorGetInfo(void);

/**
 * @brief       获取开关状态
 * 
 * @return      开关状态
 */
int lTalMotorGetOnoff(void);

/**
 * @brief       获取方向
 * 
 * @return      方向值
 */
int lTalMotorGetRotationDirection(void);

/**
 * @brief       获取速度
 * 
 * @return      速度值
 */
int lTalMotorGetSpinVelocityLevel(void);

/**
 * @brief       马达初始化
 * 
 * @return      无
 */
int lTalMotorInit(void);




















#endif
