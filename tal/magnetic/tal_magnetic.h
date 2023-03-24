#ifndef __TAL_MAGNETIC_H__
#define __TAL_MAGNETIC_H__

/**
   * @brief     获取磁力传感器状态
   *
   * @return    三路霍尔传感器的状态值，低3 Bit的值分别表示三路传感器的状态
   * 
   *   NOTE:    读取到低电平表示检测到磁场，返回值对应Bit为1表示检测到磁场
   *            
   * 
   */
jee_uint8_t ucTalGetMagStatus(void);

/**
   * @brief   磁力传感器初始化.
   *
   * NOTE:     IO input初始化   
   */
jee_int32_t lTalMagneticInit(void);

/**
   * @brief  获取当前总体磁场状态(有/无)
   *
   * @return:   1- 三路只有一路有磁场 0 -三路都没磁场    
   */
jee_uint8_t ucTalgetMagnetic(void);

/**
   * @brief  获取当前某一路的磁场状态(有/无)
   *
   * @param[in] id   要获取第几路磁场 ，范围 ：1-3
   * 
   * @return:   1- 有 0 -无   
   */
jee_uint8_t ucTalGetIsMagnetic(jee_uint8_t id);

#endif
