#ifndef __TAL_WATERPUMP_H__
#define __TAL_WATERPUMP_H__


typedef enum
{
   WATER_PUMP_SPEED_FAST = 0,
   WATER_PUMP_SPEED_MEDIAN,
   WATER_PUMP_SPEED_LOW,
   WATER_PUMP_SPEED_STOP
} waterPumpSpeed_t;

#define WATER_PUMP_SET_CMD (JEE_SENSOR_CTRL_USER_CMD_START + 1)

/**
   * @brief       初始化水泵设备.
   *
   *   NOTE:      PWM初始化
   */
jee_int32_t lTalWaterPumpInit(void);

/**
   * @brief       开启或者停止水泵运行.
   *
   * @param[in]  OnOff - 运行：1  停止：0 
   */
void vTalSetWaterPumpOnoff(jee_uint8_t OnOff);

/**
   * @brief       获取水泵运行状态
   * 
   * @return      运行：1  停止：0   
   */
jee_uint8_t ucTalGetWaterPumpOnoff(void);

/**
   * @brief       设置水泵出水速度.
   *
   * @param[in]  speed - 舵机速度
   * 
   *   NOTE:      传入 waterPumpSpeed_t
   */
void vTalSetWaterPumpMode(waterPumpSpeed_t speed);

/**
   * @brief       获取水泵速度.
   * 
   * @return      水泵速度类型（waterPumpSpeed_t）   
   */
waterPumpSpeed_t ucTalGetWaterPumpMode(void);

#endif
