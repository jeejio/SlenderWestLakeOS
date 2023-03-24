/*
 * Copyright (c) 2022, Jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-18     Monk         the first version
 */

#ifndef __AUTO_INIT_H__
#define __AUTO_INIT_H__

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdarg.h>

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Show the logs of auto init for debugging */
#define DEBUG_INIT 1

/* Automatic initialization switch */
#define configUSING_COMPONENTS_INIT


#undef RT_SECTION
#undef RT_USED
#undef RT_UNUSED

#define RT_UNUSED(x)                   ((void)x)

/* Compiler Related Definitions */
#if defined(__ARMCC_VERSION)           /* ARM Compiler */
#define RT_SECTION(x)               __attribute__((section(x)))
#define RT_USED                     __attribute__((used))
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
#define RT_SECTION(x)               @ x
#define RT_USED                     __root
#elif defined (__GNUC__)                /* GNU GCC Compiler */
#define RT_SECTION(x)               __attribute__((section(x)))
#define RT_USED                     __attribute__((used))
#elif defined (__ADSPBLACKFIN__)        /* for VisualDSP++ Compiler */
#define RT_SECTION(x)               __attribute__((section(x)))
#define RT_USED                     __attribute__((used))
#elif defined (_MSC_VER)
#define RT_SECTION(x)
#define RT_USED
#elif defined (__TI_COMPILER_VERSION__)
/* The way that TI compiler set section is different from other(at least
    * GCC and MDK) compilers. See ARM Optimizing C/C++ Compiler 5.9.3 for more
    * details. */
#define RT_SECTION(x)
#define RT_USED
#elif defined (__TASKING__)
#define RT_SECTION(x)               __attribute__((section(x)))
#define RT_USED                     __attribute__((used, protect))
#else
    #error not supported tool chain
#endif /* __ARMCC_VERSION */

#define _SECTION_ATTR_IMPL(SECTION, COUNTER) __attribute__((section(SECTION "." _COUNTER_STRINGIFY(COUNTER))))

/* initialization export */
#ifdef configUSING_COMPONENTS_INIT
typedef int (*initFn_t)(void);
#ifdef _MSC_VER
#pragma section("rti_fn$f",read)
    #if DEBUG_INIT
        struct rt_init_desc
        {
            const char* level;
            const initFn_t fn;
            const char* fn_name;
        };
        #define INIT_EXPORT(fn, level)                                  \
                                const char __rti_level_##fn[] = ".rti_fn." level;       \
                                const char __rti_##fn##_name[] = #fn;                   \
                                __declspec(allocate("rti_fn$f"))                        \
                                RT_USED const struct rt_init_desc __rt_init_msc_##fn =  \
                                {__rti_level_##fn, fn, __rti_##fn##_name};
    #else
        struct rt_init_desc
        {
            const char* level;
            const initFn_t fn;
        };
        #define INIT_EXPORT(fn, level)                                  \
                                const char __rti_level_##fn[] = ".rti_fn." level;       \
                                __declspec(allocate("rti_fn$f"))                        \
                                RT_USED const struct rt_init_desc __rt_init_msc_##fn =  \
                                {__rti_level_##fn, fn };
    #endif
#else
    #if DEBUG_INIT
        struct rt_init_desc
        {
            const char* fn_name;
            const initFn_t fn;
        };
        #define INIT_EXPORT(fn, level)                                                       \
            static RT_USED const char __rti_##fn##_name[] = #fn;                                            \
            static RT_USED const struct rt_init_desc __rt_init_desc_##fn RT_SECTION(".rti_fn." level) = \
            { __rti_##fn##_name, fn};
    #else
        #define INIT_EXPORT(fn, level)                                                       \
            static RT_USED const initFn_t __rt_init_##fn RT_SECTION(".rti_fn." level) = fn
    #endif
#endif
#else
#define INIT_EXPORT(fn, level)
#endif

/* board init routines will be called in board_init() function */
#define INIT_BOARD_EXPORT(fn)           INIT_EXPORT(fn, "1")

/* pre/device/component/env/app init routines will be called in init_thread */
/* components pre-initialization (pure software initialization) */
#define INIT_PREV_EXPORT(fn)            INIT_EXPORT(fn, "2")
/* device initialization */
#define INIT_DEVICE_EXPORT(fn)          INIT_EXPORT(fn, "3")
/* components initialization (dfs, lwip, ...) */
#define INIT_COMPONENT_EXPORT(fn)       INIT_EXPORT(fn, "4")
/* environment initialization (mount disk, ...) */
#define INIT_ENV_EXPORT(fn)             INIT_EXPORT(fn, "5")
/* application initialization (rtgui application etc ...) */
#define INIT_APP_EXPORT(fn)             INIT_EXPORT(fn, "6")


static __attribute__((unused)) const char *AUTO_INIT_TAG = "auto_init";
#define rt_kprintf(fmt, ...)   ESP_LOGI(AUTO_INIT_TAG, fmt, ##__VA_ARGS__)
//#define rt_kprintf(fmt, ...)    printf(fmt, ##__VA_ARGS__)

#ifdef configUSING_COMPONENTS_INIT
void rt_components_board_init(void);
void rt_components_init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __AUTO_INIT_H__ */
