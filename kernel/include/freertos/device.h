/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-11-29     zhengqian    the first version
 */

#ifndef __JEE_DEVICE_H__
#define __JEE_DEVICE_H__

#include "jeedef.h"
#include "io_dev_mgt.h"


#ifdef __cplusplus
extern "C" {
#endif



#ifdef JEE_USING_DEVICE
/**
 * @addtogroup Device
 */

/**@{*/

/*
 * device (I/O) system interface
 */
jee_device_t jee_device_find(const char *name);

jee_err_t jee_device_register(jee_device_t dev,
                            const char *name,
                            jee_uint16_t flags);
jee_err_t jee_device_unregister(jee_device_t dev);

#ifdef jee_USING_HEAP
jee_device_t jee_device_create(int type, int attach_size);
void jee_device_destroy(jee_device_t device);
#endif

jee_err_t
jee_device_set_rx_indicate(jee_device_t dev,
                          jee_err_t (*rx_ind)(jee_device_t dev, jee_size_t size));
jee_err_t
jee_device_set_tx_complete(jee_device_t dev,
                          jee_err_t (*tx_done)(jee_device_t dev, void *buffer));

jee_err_t  jee_device_init (jee_device_t dev);
jee_err_t  jee_device_open (jee_device_t dev, jee_uint16_t oflag);
jee_err_t  jee_device_close(jee_device_t dev);
jee_size_t jee_device_read (jee_device_t dev,
                          jee_off_t    pos,
                          void       *buffer,
                          jee_size_t   size);
jee_size_t jee_device_write(jee_device_t dev,
                          jee_off_t    pos,
                          const void *buffer,
                          jee_size_t   size);
jee_err_t  jee_device_control(jee_device_t dev, int cmd, void *arg);

/**@}*/
#endif


#ifdef __cplusplus
}
#endif

#endif
