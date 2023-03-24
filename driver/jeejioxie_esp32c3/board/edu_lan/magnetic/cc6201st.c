#include "cc6201st.h"
#include <stdio.h>
#include "hal_gpio.h"
#include <string.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "auto_init.h"

#define MAG_IO1 (8)
#define MAG_IO2 (9)
#define MAG_IO3 (2)

void cc6201stInit(void)
{

    jee_pin_mode(MAG_IO1, PIN_MODE_INPUT);
    jee_pin_mode(MAG_IO2, PIN_MODE_INPUT);
    jee_pin_mode(MAG_IO3, PIN_MODE_INPUT);
}

jee_int32_t cc6201stRead(jee_uint8_t id)
{
    jee_int32_t value = 0;
    switch (id)
    {
    case MAG_ID1:

        value = jee_pin_read(MAG_IO1);
        break;
    case MAG_ID2:
        value = jee_pin_read(MAG_IO2);
        break;
    case MAG_ID3:
        value = jee_pin_read(MAG_IO3);
        break;
    default:
        value = -1;
        break;
    }
    return value;
}

jee_uint8_t cc6201stGetStatus(void)
{
    uint8_t status = 0;
    int val = 0;
    val = cc6201stRead(MAG_ID1);
    if (val == 0) // 低电平有效
    {
        status |= 0x01;
    }
    val = cc6201stRead(MAG_ID2);
    if (val == 0)
    {
        status |= 0x02;
    }
    val = cc6201stRead(MAG_ID3);
    if (val == 0)
    {
        status |= 0x04;
    }
    return status;
}
