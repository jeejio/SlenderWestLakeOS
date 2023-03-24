#ifndef __TAL_SCANNER_H__
#define __TAL_SCANNER_H__


/**
   * @brief       扫码传感器初始化
   *
   * NOTE:        TAL层调用驱动框架查找设备，并对扫码传感器硬件进行初始化
   *
   */
jee_int32_t lTalScannerInit(void);


/**
   * @brief       获取扫码结果
   *
   * NOTE:        TAL层调用驱动框架查找设备，并对扫码传感器硬件进行初始化
   *
   * @param[out]  readdData : code - 指向存储扫码信息数组地址的指针
   * 
   * @return      返回扫码信息数组长度
   */
jee_uint8_t ucTalScannerGetCode(jee_uint8_t *readdData);

#endif
