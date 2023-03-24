#ifndef __TAL_SPO2_H__
#define __TAL_SPO2_H__

#include "jeedef.h"
#define MAX30102_SET_ONOFFCMD  (JEE_SENSOR_CTRL_USER_CMD_START + 1)    

 /**
   * @brief       血氧传感器初始化
   *
   * NOTE:        TAL层调用驱动框架查找设备，并对血氧感器硬件进行初始化
   *
   */
jee_int32_t lTalSpo2Init(void);

 /**
   * @brief       设置血氧传感器开关
   * 
   * @param[in]   OnOff ：1-开，0-关
   *
   */
void vTalSp02SetOnOff(jee_uint8_t OnOff);

 /**
   * @brief       获取血氧传感器开关
   * 
   * @return      1-开，0-关
   *
   */
jee_uint8_t ucTalSpo2GetOnOff(void);

 /**
   * @brief       获取心率
   * NOTE:        一秒更新一次
   * @return     心率 范围0-120
   *
   */
jee_uint32_t lTalSpo2GetHeartRate(void);

/**
   * @brief       获取血氧饱和度
   * NOTE:        一秒更新一次
   * @return     血氧饱和度 范围0-100
   *
   */
jee_uint32_t lTalSpo2GetBloodOxygen(void);

/**
   * @brief      获取数据标志位
   * NOTE:        一秒更新一次
   * @return     1-有新数据 0-没老数据
   *
   */
jee_uint8_t ucTalSpo2GetDataFlag(void);
#endif
