#ifndef __TAL_PIR_H__
#define __TAL_PIR_H__

 /**
   * @brief       PIR初始化
   *
   * NOTE:        TAL层调用驱动框架查找设备，并对PIR硬件进行初始化
   *
   */
void vTalPirInit(void);

 /**
   * @brief       获取PIR检测结果
   *
   * NOTE:        检测活体，3秒内如果活体没有运动，则提示未检测到活体
   *
   * @return      1 - 检测到人体，0 - 未检测到人体
   *
   */
jee_base_t lTalGetIsExists(void);

#endif
