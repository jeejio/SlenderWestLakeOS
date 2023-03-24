#ifndef __EC11_H
#define __EC11_H     //旋转编码器型号：EC110201N2D-HA1-012


#include "stdint.h"

/**
   * @brief       初始化编码器设备.
   *
   */
void encoderInit(void);

/**
   * @brief       获取编码器档位.
   *
   * @return      编码器档位(0～20档，20个档位=360度） 
   */
int encoderGetRelativePos(void);

/**
   * @brief       获取按钮的状态(是否按压).
   *
   * @return      按键状态（ 按下：1、抬起：0）   
   */
int encoderIsPressed(void);

/**
   * @brief       获取编码器OK键.
   *
   * @return      按键状态（ 按下：1、抬起：0）   
   */
int encoderGetRotationDirection(void);

/**
   * @brief       获取编码器档位和OK键状态和按钮状态.
   *
   *   NOTE:      编码器档位(0～20档，20个档位=360度） 
   *              按键状态（ 按下：1、抬起：0）   
   *              OK按键状态（ 按下：1、抬起：0） 
   * @param[out]  pdata - 档位，按键状态，OK键状态  
   */
void encoderGetDirectinPressedRelativePos(uint32_t *pdata);

/**
   * @brief       获取编码器的变化数据.
   *
   *   NOTE:      主要用于主动上报
   * 
   * @param[out]  *str - 输出字符串.
   * 
   *   NOTE:      字符串格式-如下 
   *             {
   *              isPressed:'True',
   *		      rotationDirection:'Clockwise'，
   *		      position:1,
   *              }
   * @return      数据获取状态（ 成功：0、失败：-1）   
   */
uint8_t encoderUpload(char *str);
#endif
