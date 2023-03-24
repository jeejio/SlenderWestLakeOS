#ifndef __BUTTON_665_H__
#define __BUTTON_665_H__
#include "string.h"
#include "stdint.h"

typedef enum
{
    BTN_PRESS_DOWN = 0,
    BTN_PRESS_CLICK,
    BTN_PRESS_DOUBLE_CLICK,
    BTN_PRESS_REPEAT_CLICK,
    BTN_PRESS_SHORT_START,
    BTN_PRESS_SHORT_UP,
    BTN_PRESS_LONG_START,
    BTN_PRESS_LONG_UP,
    BTN_PRESS_LONG_HOLD,
    BTN_PRESS_LONG_HOLD_UP,
    BTN_PRESS_MAX,
    BTN_PRESS_NONE,
} button_event_t;


 /**
   * @brief       矩阵按键初始化
   * 
   * NOTE:        在其他 矩阵按键 相关函数使用前调用，只调用一次 
   * 
  */
void button_panel_init(void);

 /**
   * @brief       获取按下的按键序号
   * 
   * NOTE:        获取最近一次完成按下操作的按键序号 
   * 
   * @return      0： 没有按键按下，1-9：按下的按键序号
   * 
   */
 uint8_t  button_panel_get_button_num(void);

 /**
   * @brief       按键按下模式
   * 
   * NOTE:        获取最近一次完成按下操作的按键模式
   * 
   * @return     按键按下模式
   *             1 - 单击 @ref 500ms 秒内完成按键动作  
   *             2 - 双击 @ref 连续两次完成单击动作 
   *             3 - 多击 @ref 连续三次及以上完成单击动作
   *             5 - 短按 @ref 500ms-5s 完成单次按下动作
   *             7 - 长按 @ref 5s-9s 内完成单次按下动作
   *             9 - 长按保持 @ref 超过9s 完成单次按下动作
   */
int button_panel_get_button_mode(void);

 /**
   * @brief       按键上传函数
   *
   * NOTE:        按键信息有变化的时候进行上传操作
   * 
   * @param[out]   buff - 指向储存上传数据地址的指针
   *
   * @return      1 - 按键信息有变化，0 - 按键信息没变化
   */
uint32_t button_panel_upload(char *buff);

 /**
   * @brief       获取按下的按键序号和模式
   * 
   * NOTE:        获取最近一次完成按下操作的按键序号和模式 
   * 
   * @return      0： 没有按键按下，1-9：按下的按键序号
   * 
   */

void button_panel_get_button_info(int *buf);

#endif
