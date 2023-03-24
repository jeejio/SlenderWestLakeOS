#ifndef _BSP_DEFINE_H_
#define _BSP_DEFINE_H_

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

typedef SemaphoreHandle_t JEE_MUTEX;
#define JEE_LOCK(JEE_MUTEX)         (xSemaphoreTake(JEE_MUTEX, portMAX_DELAY))           
#define JEE_UNLOCK(JEE_MUTEX)       (xSemaphoreGive(JEE_MUTEX))
#define JEE_LOCK_INIT(JEE_MUTEX)    (JEE_MUTEX = xSemaphoreCreateMutex())

#define JeeTrue (1)
#define JeeFalse (0)
typedef int8_t JeeBool;

#endif // _BSP_DEFINE_H_
