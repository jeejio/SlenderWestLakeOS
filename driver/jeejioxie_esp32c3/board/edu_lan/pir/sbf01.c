#include "sbf01.h"
#include "hal_gpio.h"


void vSbf01Init(void)
{
    jee_pin_mode(PIR_PIN_NUM, PIN_MODE_INPUT_PULLUP);
}


jee_base_t lSbf01ReadStatus(void)
{
    return jee_pin_read(PIR_PIN_NUM);
}