/*
 * Copyright (c) 2018-2023, jeejio
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#ifndef CPUTIME_H__
#define CPUTIME_H__

#include <stdint.h>

struct jee_clock_cputime_ops
{
    uint32_t (*cputime_getres) (void);
    uint32_t (*cputime_gettime)(void);
};

uint32_t clock_cpu_getres(void);
uint32_t clock_cpu_gettime(void);

uint32_t clock_cpu_microsecond(uint32_t cpu_tick);
uint32_t clock_cpu_millisecond(uint32_t cpu_tick);

int clock_cpu_setops(const struct jee_clock_cputime_ops *ops);

#endif
