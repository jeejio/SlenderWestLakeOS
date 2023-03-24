#ifndef __PCA9536_H
#define __PCA9536_H

#include "stdint.h"
#include "jeedef.h"

/**
 * @brief       io拓展芯片初始化
 *
 * NOTE:        在其他 io拓展芯片 相关函数使用前调用，只调用一次
 *
 */
void pca9536Init(void);

/**
 * @brief       配置io拓展芯片
 *
 * NOTE:        分别把io配置成输入或输出
 *              芯片总共有四个pin可供配置
 * 
 * @param[in]   cfg0 配置参数 ，低四位有效
 *
 */
void pca9536_io_config(uint8_t cfg0);

/**
 * @brief       获取io拓展芯片设置成input的引脚电平值
 *
 *        
 * @return      读取失败 -1，读取成功 f0-ff
 *
 */
int pca9536_read_io(void);

#endif
