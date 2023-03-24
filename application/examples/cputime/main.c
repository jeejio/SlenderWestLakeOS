#include <stdio.h>
#include "hal_cputime.h"
#include "esp_log.h"
#include "auto_init.h"

#define TAG "cputime"

void app_main(void)
{

#ifdef configUSING_COMPONENTS_INIT
    // Onboard components initialization.
    //rt_components_board_init();
    // components initialization.
    rt_components_init();
#endif
    for(int i = 0; i < 100; i++)
    {
        uint32_t tick = clock_cpu_gettime();
        LOGI(TAG, "current cpu time: %d ms", clock_cpu_microsecond(tick));
    }
}
