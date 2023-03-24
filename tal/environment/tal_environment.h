#ifndef __TAL_ENVIRONMENT_H__
#define __TAL_ENVIRONMENT_H__

/**
 * @brief       环境光传感器初始化函数
 *
 *
 */
jee_int32_t lTalAmbientLightInit(void);

/**
 * @brief       获取光照强度值
 * 
 * @return      光照强度值
 *
 */
jee_uint16_t usTalGetAmbientLightData(void);

/**
 * @brief       温湿度传感器初始化函数
 *
 *
 */
jee_int32_t lTalTempHumInit(void);

/**
 * @brief      获取温湿度传感器数据
 *
 * @param[out]  两个int数据，第一个是温蒂，第二个是湿度
 * NOTE:         温度值是带1位小数点的值放大10倍的结果,
 *               单位：摄氏度
 */
void vTalGetTemHumData(jee_int32_t *data);

/**
 * @brief       烟雾传感器初始化函数
 *
 *
 */
jee_int32_t lTalMq2Init(void);

/**
 * @brief       获取烟雾超标状态
 *
 * @return      烟雾超标状态值，
 *                  0：未超标；
 *                  1：超标。
 */
jee_uint8_t ucTalMq2GetSmokeStatus(void);

/**
 * @brief       一氧化碳传感器初始化函数
 *
 *
 */
jee_int32_t lTalMq7Init(void);

/**
 * @brief       获取一氧化碳超标状态
 *
 * @return      一氧化碳超标状态值，
 *                  0：未超标；
 *                  1：超标。
 */
jee_uint8_t ucTalMq7GetCoStatus(void);

/**
 * @brief       空气质量传感器初始化函数
 *
 *
 */
jee_int32_t lTalMq135Init(void);

/**
 * @brief       获取空气质量超标状态
 *
 * @return      空气质量超标状态值，
 *                  0：未超标；
 *                  1：超标。
 */
jee_uint8_t ucTalMq135GetAirStatus(void);

#endif
