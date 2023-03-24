#ifndef __DHT20_H
#define __DHT20_H


#include "stdint.h"

void dht20_init(void);
uint8_t dht20_read_status(void);
void dht20_startup_init(void);
void dht20_read_humi_temp(uint32_t *pdata);
int dht20_read_humi_temp_crc(uint32_t *pdata);
void dht20_task_init(void);
void dht20_get_data(int32_t *data);






#endif
