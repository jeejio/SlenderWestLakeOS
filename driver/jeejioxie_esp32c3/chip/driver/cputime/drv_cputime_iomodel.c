/*
 * Copyright (c) 2018-2023, jeejio
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <jeedef.h>
#include <auto_init.h>
#include <device.h>
#include <freertos/FreeRTOS.h>
#include <hal_cputime.h>

#include <esp_cpu.h>
#include <esp_rom_sys.h>

/* Use Cycle counter of Data Watchpoint and Trace Register for CPU time */
static uint32_t cortexm_cputime_getres(void)
{
    return esp_rom_get_cpu_ticks_per_us();
}

static uint32_t cortexm_cputime_gettime(void)
{
    return esp_cpu_get_cycle_count();
}

const static struct jee_clock_cputime_ops _cortexm_ops =
{
    cortexm_cputime_getres,
    cortexm_cputime_gettime
};

int cputime_init(void)
{
    clock_cpu_setops(&_cortexm_ops);
    return 0;
}
INIT_BOARD_EXPORT(cputime_init);