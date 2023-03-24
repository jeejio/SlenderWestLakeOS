/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-29     zhengqian    the first version
*/

#ifndef __JEE_DEF_H__
#define __JEE_DEF_H__

#define JEE_USING_LIBC

#ifdef JEE_USING_LIBC
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include "esp_log.h"

#endif /* JEE_USING_LIBC */

#define JEE_USING_DEVICE
#define JEE_NAME_MAX 20

#define LogError(tag, pcformat, ...)      ESP_LOGE(tag, pcformat, ##__VA_ARGS__)
#define LogWarn(tag, pcformat, ...)       ESP_LOGW(tag, pcformat, ##__VA_ARGS__)
#define LogInfo(tag, pcformat, ...)       ESP_LOGI(tag, pcformat, ##__VA_ARGS__)
#define LogDebug(tag, pcformat, ...)      ESP_LOGD(tag, pcformat, ##__VA_ARGS__)


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup BasicDef
 */

/**@{*/


/* e.g. #if (RTTHREAD_VERSION >= JEE_VERSION_CHECK(4, 1, 0) */
#define JEE_VERSION_CHECK(major, minor, revise)          ((major * 10000) + \
                                                         (minor * 100) + revise)

/* RT-Thread basic data type definitions */
#ifndef JEE_USING_ARCH_DATA_TYPE
#ifdef JEE_USING_LIBC
typedef int8_t                          jee_int8_t;      /**<  8bit integer type */
typedef int16_t                         jee_int16_t;     /**< 16bit integer type */
typedef int32_t                         jee_int32_t;     /**< 32bit integer type */
typedef uint8_t                         jee_uint8_t;     /**<  8bit unsigned integer type */
typedef uint16_t                        jee_uint16_t;    /**< 16bit unsigned integer type */
typedef uint32_t                        jee_uint32_t;    /**< 32bit unsigned integer type */
typedef int64_t                         jee_int64_t;     /**< 64bit integer type */
typedef uint64_t                        jee_uint64_t;    /**< 64bit unsigned integer type */
typedef size_t                          jee_size_t;      /**< Type for size number */

#else
typedef signed   char                   jee_int8_t;      /**<  8bit integer type */
typedef signed   short                  jee_int16_t;     /**< 16bit integer type */
typedef signed   int                    jee_int32_t;     /**< 32bit integer type */
typedef unsigned char                   jee_uint8_t;     /**<  8bit unsigned integer type */
typedef unsigned short                  jee_uint16_t;    /**< 16bit unsigned integer type */
typedef unsigned int                    jee_uint32_t;    /**< 32bit unsigned integer type */

#ifdef ARCH_CPU_64BIT
typedef signed long                     jee_int64_t;     /**< 64bit integer type */
typedef unsigned long                   jee_uint64_t;    /**< 64bit unsigned integer type */
typedef unsigned long                   jee_size_t;      /**< Type for size number */
#else
typedef signed long long                jee_int64_t;     /**< 64bit integer type */
typedef unsigned long long              jee_uint64_t;    /**< 64bit unsigned integer type */
typedef unsigned int                    jee_size_t;      /**< Type for size number */
#endif /* ARCH_CPU_64BIT */
#endif /* JEE_USING_LIBC */
#endif /* JEE_USING_ARCH_DATA_TYPE */

typedef int                             jee_bool_t;      /**< boolean type */
typedef long                            jee_base_t;      /**< Nbit CPU related date type */
typedef unsigned long                   jee_ubase_t;     /**< Nbit unsigned CPU related data type */

typedef jee_base_t                       jee_err_t;       /**< Type for error number */
typedef jee_uint32_t                     jee_time_t;      /**< Type for time stamp */
typedef jee_uint32_t                     jee_tick_t;      /**< Type for tick count */
typedef jee_base_t                       jee_flag_t;      /**< Type for flags */
typedef jee_ubase_t                      jee_dev_t;       /**< Type for device */
typedef jee_base_t                       jee_off_t;       /**< Type for offset */

/* boolean type definitions */
#define JEE_TRUE                         1               /**< boolean true  */
#define JEE_FALSE                        0               /**< boolean fails */

/* null pointer definition */
#define JEE_NULL                         0

/**@}*/

/* maximum value of base type */
#ifdef JEE_USING_LIBC
#define JEE_UINT8_MAX                    UINT8_MAX       /**< Maximum number of UINT8 */
#define JEE_UINT16_MAX                   UINT16_MAX      /**< Maximum number of UINT16 */
#define JEE_UINT32_MAX                   UINT32_MAX      /**< Maximum number of UINT32 */
#else
#define JEE_UINT8_MAX                    0xff            /**< Maximum number of UINT8 */
#define JEE_UINT16_MAX                   0xffff          /**< Maximum number of UINT16 */
#define JEE_UINT32_MAX                   0xffffffff      /**< Maximum number of UINT32 */
#endif /* JEE_USING_LIBC */

#define JEE_TICK_MAX                     JEE_UINT32_MAX   /**< Maximum number of tick */

/* maximum value of ipc type */
#define JEE_SEM_VALUE_MAX                JEE_UINT16_MAX   /**< Maximum number of semaphore .value */
#define JEE_MUTEX_VALUE_MAX              JEE_UINT16_MAX   /**< Maximum number of mutex .value */
#define JEE_MUTEX_HOLD_MAX               JEE_UINT8_MAX    /**< Maximum number of mutex .hold */
#define JEE_MB_ENTRY_MAX                 JEE_UINT16_MAX   /**< Maximum number of mailbox .entry */
#define JEE_MQ_ENTRY_MAX                 JEE_UINT16_MAX   /**< Maximum number of message queue .entry */

#define JEE_UNUSED(x)                   ((void)x)

/* Compiler Related Definitions */
#if defined(__ARMCC_VERSION)           /* ARM Compiler */
#define JEE_SECTION(x)               __attribute__((section(x)))
#define JEE_USED                     __attribute__((used))
#define ALIGN(n)                    __attribute__((aligned(n)))
#define JEE_WEAK                     __attribute__((weak))
#define jee_inline                   static __inline
/* module compiling */
#ifdef JEE_USING_MODULE
#define RTT_API                     __declspec(dllimport)
#else
#define RTT_API                     __declspec(dllexport)
#endif /* JEE_USING_MODULE */
#elif defined (__IAR_SYSTEMS_ICC__)     /* for IAR Compiler */
#define JEE_SECTION(x)               @ x
#define JEE_USED                     __root
#define PRAGMA(x)                   _Pragma(#x)
#define ALIGN(n)                    PRAGMA(data_alignment=n)
#define JEE_WEAK                     __weak
#define jee_inline                   static inline
#define RTT_API
#elif defined (__GNUC__)                /* GNU GCC Compiler */
#ifndef JEE_USING_LIBC
/* the version of GNU GCC must be greater than 4.x */
typedef __builtin_va_list           __gnuc_va_list;
typedef __gnuc_va_list              va_list;
#define va_start(v,l)               __builtin_va_start(v,l)
#define va_end(v)                   __builtin_va_end(v)
#define va_arg(v,l)                 __builtin_va_arg(v,l)
#endif /* JEE_USING_LIBC */
#define JEE_SECTION(x)               __attribute__((section(x)))
#define JEE_USED                     __attribute__((used))
#define ALIGN(n)                    __attribute__((aligned(n)))
#define JEE_WEAK                     __attribute__((weak))
#define jee_inline                   static __inline
#define RTT_API
#elif defined (__ADSPBLACKFIN__)        /* for VisualDSP++ Compiler */
#define JEE_SECTION(x)               __attribute__((section(x)))
#define JEE_USED                     __attribute__((used))
#define ALIGN(n)                    __attribute__((aligned(n)))
#define JEE_WEAK                     __attribute__((weak))
#define jee_inline                   static inline
#define RTT_API
#elif defined (_MSC_VER)
#define JEE_SECTION(x)
#define JEE_USED
#define ALIGN(n)                    __declspec(align(n))
#define JEE_WEAK
#define jee_inline                   static __inline
#define RTT_API
#elif defined (__TI_COMPILER_VERSION__)
/* The way that TI compiler set section is different from other(at least
    * GCC and MDK) compilers. See ARM Optimizing C/C++ Compiler 5.9.3 for more
    * details. */
#define JEE_SECTION(x)
#define JEE_USED
#define PRAGMA(x)                   _Pragma(#x)
#define ALIGN(n)
#define JEE_WEAK
#define jee_inline                   static inline
#define RTT_API
#elif defined (__TASKING__)
#define JEE_SECTION(x)               __attribute__((section(x)))
#define JEE_USED                     __attribute__((used, protect))
#define PRAGMA(x)                   _Pragma(#x)
#define ALIGN(n)                    __attribute__((__align(n)))
#define JEE_WEAK                     __attribute__((weak))
#define jee_inline                   static inline
#define RTT_API
#else
    #error not supported tool chain
#endif /* __ARMCC_VERSION */


/* memory management option */
#define JEE_MM_PAGE_SIZE                 4096
#define JEE_MM_PAGE_MASK                 (JEE_MM_PAGE_SIZE - 1)
#define JEE_MM_PAGE_BITS                 12

/* kernel malloc definitions */
#ifndef JEE_KERNEL_MALLOC
#define JEE_KERNEL_MALLOC(sz)            jee_malloc(sz)
#endif

#ifndef JEE_KERNEL_FREE
#define JEE_KERNEL_FREE(ptr)             jee_free(ptr)
#endif

#ifndef JEE_KERNEL_REALLOC
#define JEE_KERNEL_REALLOC(ptr, size)    jee_realloc(ptr, size)
#endif

/**
 * @addtogroup Error
 */

/**@{*/

/* RT-Thread error code definitions */
#define JEE_EOK                          0               /**< There is no error */
#define JEE_ERROR                        1               /**< A generic error happens */
#define JEE_ETIMEOUT                     2               /**< Timed out */
#define JEE_EFULL                        3               /**< The resource is full */
#define JEE_EEMPTY                       4               /**< The resource is empty */
#define JEE_ENOMEM                       5               /**< No memory */
#define JEE_ENOSYS                       6               /**< No system */
#define JEE_EBUSY                        7               /**< Busy */
#define JEE_EIO                          8               /**< IO error */
#define JEE_EINTR                        9               /**< Interrupted system call */
#define JEE_EINVAL                       10              /**< Invalid argument */

/**@}*/

/**
 * @ingroup BasicDef
 *
 * @def JEE_ALIGN(size, align)
 * Return the most contiguous size aligned at specified width. JEE_ALIGN(13, 4)
 * would return 16.
 */
#define JEE_ALIGN(size, align)           (((size) + (align) - 1) & ~((align) - 1))

/**
 * @ingroup BasicDef
 *
 * @def JEE_ALIGN_DOWN(size, align)
 * Return the down number of aligned at specified width. JEE_ALIGN_DOWN(13, 4)
 * would return 12.
 */
#define JEE_ALIGN_DOWN(size, align)      ((size) & ~((align) - 1))

/**
 * Double List structure
 */
struct jee_list_node
{
    struct jee_list_node *next;                          /**< point to next node. */
    struct jee_list_node *prev;                          /**< point to prev node. */
};
typedef struct jee_list_node jee_list_t;                  /**< Type for lists. */

/**
 * Single List structure
 */
struct jee_slist_node
{
    struct jee_slist_node *next;                         /**< point to next node. */
};
typedef struct jee_slist_node jee_slist_t;                /**< Type for single list. */

/**
 * The information of the device
 */
struct jee_device_information
{
    jee_list_t                 device_list;              /**< device list */
    jee_size_t                 device_size;              /**< device size */
};

/**
 * The hook function call macro
 */
#ifndef JEE_USING_HOOK
    #define __ON_HOOK_ARGS(__hook, argv)
    #define JEE_OBJECT_HOOK_CALL(func, argv)
#else
    #define JEE_OBJECT_HOOK_CALL(func, argv)         __on_##func argv
    #ifdef JEE_HOOK_USING_FUNC_PTR
        #define __ON_HOOK_ARGS(__hook, argv)        do {if ((__hook) != JEE_NULL) __hook argv; } while (0)
    #else
        #define __ON_HOOK_ARGS(__hook, argv)
    #endif /* JEE_HOOK_USING_FUNC_PTR */
#endif /* JEE_USING_HOOK */

#ifndef __on_jee_interrupt_switch_hook
    #define __on_jee_interrupt_switch_hook()         __ON_HOOK_ARGS(jee_interrupt_switch_hook, ())
#endif
#ifndef __on_jee_malloc_hook
    #define __on_jee_malloc_hook(addr, size)         __ON_HOOK_ARGS(jee_malloc_hook, (addr, size))
#endif
#ifndef __on_jee_free_hook
    #define __on_jee_free_hook(rmem)                 __ON_HOOK_ARGS(jee_free_hook, (rmem))
#endif


/**@}*/

/**
 * @addtogroup IPC
 */

/**@{*/

/**
 * IPC flags and control command definitions
 */
#define JEE_IPC_FLAG_FIFO                0x00            /**< FIFOed IPC. @ref IPC. */
#define JEE_IPC_FLAG_PRIO                0x01            /**< PRIOed IPC. @ref IPC. */

#define JEE_IPC_CMD_UNKNOWN              0x00            /**< unknown IPC command */
#define JEE_IPC_CMD_RESET                0x01            /**< reset IPC object */

#define JEE_WAITING_FOREVER              -1              /**< Block forever until get resource. */
#define JEE_WAITING_NO                   0               /**< Non-block. */


#ifdef JEE_USING_DEVICE
/**
 * @addtogroup Device
 */

/**@{*/

/**
 * device (I/O) class type
 */
enum jee_device_class_type
{
    JEE_Device_Class_Char = 0,                           /**< character device */
    JEE_Device_Class_Block,                              /**< block device */
    JEE_Device_Class_NetIf,                              /**< net interface */
    JEE_Device_Class_MTD,                                /**< memory device */
    JEE_Device_Class_CAN,                                /**< CAN device */
    JEE_Device_Class_RTC,                                /**< RTC device */
    JEE_Device_Class_Sound,                              /**< Sound device */
    JEE_Device_Class_Graphic,                            /**< Graphic device */
    JEE_Device_Class_I2CBUS,                             /**< I2C bus device */
    JEE_Device_Class_USBDevice,                          /**< USB slave device */
    JEE_Device_Class_USBHost,                            /**< USB host bus */
    JEE_Device_Class_USBOTG,                             /**< USB OTG bus */
    JEE_Device_Class_SPIBUS,                             /**< SPI bus device */
    JEE_Device_Class_SPIDevice,                          /**< SPI device */
    JEE_Device_Class_SDIO,                               /**< SDIO bus device */
    JEE_Device_Class_PM,                                 /**< PM pseudo device */
    JEE_Device_Class_Pipe,                               /**< Pipe device */
    JEE_Device_Class_Portal,                             /**< Portal device */
    JEE_Device_Class_Timer,                              /**< Timer device */
    JEE_Device_Class_Miscellaneous,                      /**< Miscellaneous device */
    JEE_Device_Class_Sensor,                             /**< Sensor device */
    JEE_Device_Class_Touch,                              /**< Touch device */
    JEE_Device_Class_PHY,                                /**< PHY device */
    JEE_Device_Class_Security,                           /**< Security device */
    JEE_Device_Class_WLAN,                               /**< WLAN device */
    JEE_Device_Class_Pin,                                /**< Pin device */
    JEE_Device_Class_ADC,                                /**< ADC device */
    JEE_Device_Class_DAC,                                /**< DAC device */
    JEE_Device_Class_WDT,                                /**< WDT device */
    JEE_Device_Class_PWM,                                /**< PWM device */
    JEE_Device_Class_Unknown                             /**< unknown device */
};

/**
 * device flags definitions
 */
#define JEE_DEVICE_FLAG_DEACTIVATE       0x000           /**< device is not not initialized */

#define JEE_DEVICE_FLAG_RDONLY           0x001           /**< read only */
#define JEE_DEVICE_FLAG_WRONLY           0x002           /**< write only */
#define JEE_DEVICE_FLAG_RDWR             0x003           /**< read and write */

#define JEE_DEVICE_FLAG_REMOVABLE        0x004           /**< removable device */
#define JEE_DEVICE_FLAG_STANDALONE       0x008           /**< standalone device */
#define JEE_DEVICE_FLAG_ACTIVATED        0x010           /**< device is activated */
#define JEE_DEVICE_FLAG_SUSPENDED        0x020           /**< device is suspended */
#define JEE_DEVICE_FLAG_STREAM           0x040           /**< stream mode */

#define JEE_DEVICE_FLAG_INT_RX           0x100           /**< INT mode on Rx */
#define JEE_DEVICE_FLAG_DMA_RX           0x200           /**< DMA mode on Rx */
#define JEE_DEVICE_FLAG_INT_TX           0x400           /**< INT mode on Tx */
#define JEE_DEVICE_FLAG_DMA_TX           0x800           /**< DMA mode on Tx */

#define JEE_DEVICE_OFLAG_CLOSE           0x000           /**< device is closed */
#define JEE_DEVICE_OFLAG_RDONLY          0x001           /**< read only access */
#define JEE_DEVICE_OFLAG_WRONLY          0x002           /**< write only access */
#define JEE_DEVICE_OFLAG_RDWR            0x003           /**< read and write */
#define JEE_DEVICE_OFLAG_OPEN            0x008           /**< device is opened */
#define JEE_DEVICE_OFLAG_MASK            0xf0f           /**< mask of open flag */

/**
 * general device commands
 */
#define JEE_DEVICE_CTRL_RESUME           0x01            /**< resume device */
#define JEE_DEVICE_CTRL_SUSPEND          0x02            /**< suspend device */
#define JEE_DEVICE_CTRL_CONFIG           0x03            /**< configure device */
#define JEE_DEVICE_CTRL_CLOSE            0x04            /**< close device */

#define JEE_DEVICE_CTRL_SET_INT          0x10            /**< set interrupt */
#define JEE_DEVICE_CTRL_CLR_INT          0x11            /**< clear interrupt */
#define JEE_DEVICE_CTRL_GET_INT          0x12            /**< get interrupt status */

/**
 * device control
 */
#define JEE_DEVICE_CTRL_BASE(Type)        (JEE_Device_Class_##Type * 0x100)

/**
 * special device commands
 */
#define JEE_DEVICE_CTRL_CHAR_STREAM      (JEE_DEVICE_CTRL_BASE(Char) + 1)             /**< stream mode on char device */
#define JEE_DEVICE_CTRL_BLK_GETGEOME     (JEE_DEVICE_CTRL_BASE(Block) + 1)            /**< get geometry information   */
#define JEE_DEVICE_CTRL_BLK_SYNC         (JEE_DEVICE_CTRL_BASE(Block) + 2)            /**< flush data to block device */
#define JEE_DEVICE_CTRL_BLK_ERASE        (JEE_DEVICE_CTRL_BASE(Block) + 3)            /**< erase block on block device */
#define JEE_DEVICE_CTRL_BLK_AUTOREFRESH  (JEE_DEVICE_CTRL_BASE(Block) + 4)            /**< block device : enter/exit auto refresh mode */
#define JEE_DEVICE_CTRL_NETIF_GETMAC     (JEE_DEVICE_CTRL_BASE(NetIf) + 1)            /**< get mac address */
#define JEE_DEVICE_CTRL_MTD_FORMAT       (JEE_DEVICE_CTRL_BASE(MTD) + 1)              /**< format a MTD device */

typedef struct jee_device *jee_device_t;

#ifdef JEE_USING_DEVICE_OPS
/**
 * operations set for device object
 */
struct jee_device_ops
{
    /* common device interface */
    jee_err_t  (*init)   (jee_device_t dev);
    jee_err_t  (*open)   (jee_device_t dev, jee_uint16_t oflag);
    jee_err_t  (*close)  (jee_device_t dev);
    jee_size_t (*read)   (jee_device_t dev, jee_off_t pos, void *buffer, jee_size_t size);
    jee_size_t (*write)  (jee_device_t dev, jee_off_t pos, const void *buffer, jee_size_t size);
    jee_err_t  (*control)(jee_device_t dev, int cmd, void *args);
};
#endif /* JEE_USING_DEVICE_OPS */

/**
 * WaitQueue structure
 */
struct jee_wqueue
{
    jee_uint32_t flag;
    jee_list_t waiting_list;
};
typedef struct jee_wqueue jee_wqueue_t;

/**
 * Device structure
 */
struct jee_device
{
     char                      name[JEE_NAME_MAX];       /**< name of device */
     jee_list_t                list;                     /**< list node of device */


    enum jee_device_class_type type;                     /**< device type */
    jee_uint16_t               flag;                     /**< device flag */
    jee_uint16_t               open_flag;                /**< device open flag */

    jee_uint8_t                ref_count;                /**< reference count */
    jee_uint8_t                device_id;                /**< 0 - 255 */

    /* device call back */
    jee_err_t (*rx_indicate)(jee_device_t dev, jee_size_t size);
    jee_err_t (*tx_complete)(jee_device_t dev, void *buffer);

#ifdef JEE_USING_DEVICE_OPS
    const struct jee_device_ops *ops;
#else
    /* common device interface */
    jee_err_t  (*init)   (jee_device_t dev);
    jee_err_t  (*open)   (jee_device_t dev, jee_uint16_t oflag);
    jee_err_t  (*close)  (jee_device_t dev);
    jee_size_t (*read)   (jee_device_t dev, jee_off_t pos, void *buffer, jee_size_t size);
    jee_size_t (*write)  (jee_device_t dev, jee_off_t pos, const void *buffer, jee_size_t size);
    jee_err_t  (*control)(jee_device_t dev, int cmd, void *args);
#endif /* JEE_USING_DEVICE_OPS */

#ifdef JEE_USING_POSIX_DEVIO
    const struct dfs_file_ops *fops;
    struct jee_wqueue wait_queue;
#endif /* JEE_USING_POSIX_DEVIO */

    void                     *user_data;                /**< device private data */
};

/**
 * block device geometry structure
 */
struct jee_device_blk_geometry
{
    jee_uint32_t sector_count;                           /**< count of sectors */
    jee_uint32_t bytes_per_sector;                       /**< number of bytes per sector */
    jee_uint32_t block_size;                             /**< number of bytes to erase one block */
};

/**
 * sector arrange struct on block device
 */
struct jee_device_blk_sectors
{
    jee_uint32_t sector_begin;                           /**< begin sector */
    jee_uint32_t sector_end;                             /**< end sector   */
};

/**
 * cursor control command
 */
#define JEE_DEVICE_CTRL_CURSOR_SET_POSITION  0x10
#define JEE_DEVICE_CTRL_CURSOR_SET_TYPE      0x11

/**
 * graphic device control command
 */
#define RTGRAPHIC_CTRL_RECT_UPDATE      (JEE_DEVICE_CTRL_BASE(Graphic) + 0)
#define RTGRAPHIC_CTRL_POWERON          (JEE_DEVICE_CTRL_BASE(Graphic) + 1)
#define RTGRAPHIC_CTRL_POWEROFF         (JEE_DEVICE_CTRL_BASE(Graphic) + 2)
#define RTGRAPHIC_CTRL_GET_INFO         (JEE_DEVICE_CTRL_BASE(Graphic) + 3)
#define RTGRAPHIC_CTRL_SET_MODE         (JEE_DEVICE_CTRL_BASE(Graphic) + 4)
#define RTGRAPHIC_CTRL_GET_EXT          (JEE_DEVICE_CTRL_BASE(Graphic) + 5)
#define RTGRAPHIC_CTRL_SET_BRIGHTNESS   (JEE_DEVICE_CTRL_BASE(Graphic) + 6)
#define RTGRAPHIC_CTRL_GET_BRIGHTNESS   (JEE_DEVICE_CTRL_BASE(Graphic) + 7)
#define RTGRAPHIC_CTRL_GET_MODE         (JEE_DEVICE_CTRL_BASE(Graphic) + 8)
#define RTGRAPHIC_CTRL_GET_STATUS       (JEE_DEVICE_CTRL_BASE(Graphic) + 9)
#define RTGRAPHIC_CTRL_PAN_DISPLAY      (JEE_DEVICE_CTRL_BASE(Graphic) + 10)
#define RTGRAPHIC_CTRL_WAIT_VSYNC       (JEE_DEVICE_CTRL_BASE(Graphic) + 11)

/* graphic device */
enum
{
    RTGRAPHIC_PIXEL_FORMAT_MONO = 0,
    RTGRAPHIC_PIXEL_FORMAT_GRAY4,
    RTGRAPHIC_PIXEL_FORMAT_GRAY16,
    RTGRAPHIC_PIXEL_FORMAT_RGB332,
    RTGRAPHIC_PIXEL_FORMAT_RGB444,
    RTGRAPHIC_PIXEL_FORMAT_RGB565,
    RTGRAPHIC_PIXEL_FORMAT_RGB565P,
    RTGRAPHIC_PIXEL_FORMAT_BGR565 = RTGRAPHIC_PIXEL_FORMAT_RGB565P,
    RTGRAPHIC_PIXEL_FORMAT_RGB666,
    RTGRAPHIC_PIXEL_FORMAT_RGB888,
    RTGRAPHIC_PIXEL_FORMAT_BGR888,
    RTGRAPHIC_PIXEL_FORMAT_ARGB888,
    RTGRAPHIC_PIXEL_FORMAT_ABGR888,
    RTGRAPHIC_PIXEL_FORMAT_RESERVED,
};

/**
 * build a pixel position according to (x, y) coordinates.
 */
#define RTGRAPHIC_PIXEL_POSITION(x, y)  ((x << 16) | y)

/**
 * graphic device information structure
 */
struct jee_device_graphic_info
{
    jee_uint8_t  pixel_format;                           /**< graphic format */
    jee_uint8_t  bits_per_pixel;                         /**< bits per pixel */
    jee_uint16_t pitch;                                  /**< bytes per line */

    jee_uint16_t width;                                  /**< width of graphic device */
    jee_uint16_t height;                                 /**< height of graphic device */

    jee_uint8_t *framebuffer;                            /**< frame buffer */
    jee_uint32_t smem_len;                               /**< allocated frame buffer size */
};

/**
 * rectangle information structure
 */
struct jee_device_rect_info
{
    jee_uint16_t x;                                      /**< x coordinate */
    jee_uint16_t y;                                      /**< y coordinate */
    jee_uint16_t width;                                  /**< width */
    jee_uint16_t height;                                 /**< height */
};

/**
 * graphic operations
 */
struct jee_device_graphic_ops
{
    void (*set_pixel) (const char *pixel, int x, int y);
    void (*get_pixel) (char *pixel, int x, int y);

    void (*draw_hline)(const char *pixel, int x1, int x2, int y);
    void (*draw_vline)(const char *pixel, int x, int y1, int y2);

    void (*blit_line) (const char *pixel, int x, int y, jee_size_t size);
};
#define jee_graphix_ops(device)          ((struct jee_device_graphic_ops *)(device->user_data))

/**@}*/
#endif /* JEE_USING_DEVICE */

#ifdef __cplusplus
}
#endif

#endif /* __JEE_DEF_H__ */

