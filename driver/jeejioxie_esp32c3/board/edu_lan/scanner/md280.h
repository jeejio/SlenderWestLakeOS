#ifndef __MD280_H
#define __MD280_H

#include "stdint.h"
#include "jeedef.h"
#include <string.h>

void md280Init(void);

uint16_t md280GetData(uint8_t *data);

#endif
