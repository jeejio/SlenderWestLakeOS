/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-3-12     Yi Qiu      first version
 * 2021-02-23     Leslie Lee  provide possibility for multi usb host
 */

#ifndef __HAL_USB_HOST_H__
#define __HAL_USB_HOST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <jeedef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <io_dev_mgt.h>
#include <string.h>
#include "hal_usb_common.h"

#define USB_MAX_DEVICE                  0x20
#define USB_MAX_INTERFACE               0x08
#define USB_HUB_PORT_NUM                0x04
#define SIZEOF_USB_REQUEST              0x08

#define DEV_STATUS_IDLE                 0x00
#define DEV_STATUS_BUSY                 0x01
#define DEV_STATUS_ERROR                0x02

#define UPIPE_STATUS_OK                 0x00
#define UPIPE_STATUS_STALL              0x01
#define UPIPE_STATUS_ERROR              0x02

#define USBH_PID_SETUP                  0x00
#define USBH_PID_DATA                   0x01

struct uhcd;
struct uhintf;
struct uhub;
struct upipe;

struct uclass_driver
{
    jee_list_t list;
    int class_code;
    int subclass_code;

    jee_err_t (*enable)(void* arg);
    jee_err_t (*disable)(void* arg);

    void* user_data;
};
typedef struct uclass_driver* ucd_t;

struct uprotocal
{
    jee_list_t list;
    int pro_id;

    jee_err_t (*init)(void* arg);
    jee_err_t (*callback)(void* arg);
};
typedef struct uprotocal* uprotocal_t;

struct uinstance
{
    struct jee_device parent;

    struct udevice_descriptor dev_desc;
    ucfg_desc_t cfg_desc;
    struct uhcd *hcd;

    struct upipe * pipe_ep0_out;
    struct upipe * pipe_ep0_in;
    jee_list_t pipe;

    jee_uint8_t status;
    jee_uint8_t type;
    jee_uint8_t index;
    jee_uint8_t address;
    jee_uint8_t speed;
    jee_uint8_t max_packet_size;
    jee_uint8_t port;

    struct uhub* parent_hub;
    struct uhintf* intf[USB_MAX_INTERFACE];
};
typedef struct uinstance* uinst_t;

struct uhintf
{
    struct uinstance* device;
    uintf_desc_t intf_desc;

    ucd_t drv;
    void *user_data;
};

struct upipe
{
    jee_list_t list;
    jee_uint8_t pipe_index;
    jee_uint32_t status;
    struct uendpoint_descriptor ep;
    uinst_t inst;
    func_callback callback;
    void* user_data;
};
typedef struct upipe* upipe_t;

struct uhub
{
    struct uhub_descriptor hub_desc;
    jee_uint8_t num_ports;
    jee_uint32_t port_status[USB_HUB_PORT_NUM];
    struct uinstance* child[USB_HUB_PORT_NUM];

    jee_bool_t is_roothub;

    jee_uint8_t buffer[8];
    struct uinstance* self;
    struct uhcd *hcd;
};
typedef struct uhub* uhub_t;

struct uhcd_ops
{
    jee_err_t    (*reset_port)   (jee_uint8_t port);
    int         (*pipe_xfer)    (upipe_t pipe, jee_uint8_t token, void* buffer, int nbytes, int timeout);
    jee_err_t    (*open_pipe)    (upipe_t pipe);
    jee_err_t    (*close_pipe)   (upipe_t pipe);
};
typedef struct uhcd_ops* uhcd_ops_t;
struct uhcd
{
    struct jee_device parent;
    uhcd_ops_t ops;
    jee_uint8_t num_ports;
    uhub_t roothub;
    QueueHandle_t usb_mq;
};
typedef struct uhcd* uhcd_t;

enum uhost_msg_type
{
    USB_MSG_CONNECT_CHANGE,
    USB_MSG_CALLBACK,
};
typedef enum uhost_msg_type uhost_msg_type;

struct uhost_msg
{
    uhost_msg_type type;
    union
    {
        struct uhub* hub;
        struct
        {
            func_callback function;
            void *context;
        }cb;
    }content;
};
typedef struct uhost_msg* uhost_msg_t;

/* usb host system interface */
jee_err_t jee_usb_host_init(const char *name);
void jee_usbh_hub_init(struct uhcd *hcd);

/* usb host core interface */
struct uinstance* jee_usbh_alloc_instance(uhcd_t uhcd);
jee_err_t jee_usbh_attatch_instance(struct uinstance* device);
jee_err_t jee_usbh_detach_instance(struct uinstance* device);
jee_err_t jee_usbh_get_descriptor(struct uinstance* device, jee_uint8_t type, void* buffer, int nbytes);
jee_err_t jee_usbh_set_configure(struct uinstance* device, int config);
jee_err_t jee_usbh_set_address(struct uinstance* device);
jee_err_t jee_usbh_set_interface(struct uinstance* device, int intf);
jee_err_t jee_usbh_clear_feature(struct uinstance* device, int endpoint, int feature);
jee_err_t jee_usbh_get_interface_descriptor(ucfg_desc_t cfg_desc, int num, uintf_desc_t* intf_desc);
jee_err_t jee_usbh_get_endpoint_descriptor(uintf_desc_t intf_desc, int num, uep_desc_t* ep_desc);

/* usb class driver interface */
jee_err_t jee_usbh_class_driver_init(void);
jee_err_t jee_usbh_class_driver_register(ucd_t drv);
jee_err_t jee_usbh_class_driver_unregister(ucd_t drv);
jee_err_t jee_usbh_class_driver_enable(ucd_t drv, void* args);
jee_err_t jee_usbh_class_driver_disable(ucd_t drv, void* args);
ucd_t jee_usbh_class_driver_find(int class_code, int subclass_code);

/* usb class driver implement */
ucd_t jee_usbh_class_driver_hub(void);
ucd_t jee_usbh_class_driver_storage(void);



/* usb hub interface */
jee_err_t jee_usbh_hub_get_descriptor(struct uinstance* device, jee_uint8_t *buffer,
    jee_size_t size);
jee_err_t jee_usbh_hub_get_status(struct uinstance* device, jee_uint32_t* buffer);
jee_err_t jee_usbh_hub_get_port_status(uhub_t uhub, jee_uint16_t port,
    jee_uint32_t* buffer);
jee_err_t jee_usbh_hub_clear_port_feature(uhub_t uhub, jee_uint16_t port,
    jee_uint16_t feature);
jee_err_t jee_usbh_hub_set_port_feature(uhub_t uhub, jee_uint16_t port,
    jee_uint16_t feature);
jee_err_t jee_usbh_hub_reset_port(uhub_t uhub, jee_uint16_t port);
jee_err_t jee_usbh_event_signal(uhcd_t uhcd, struct uhost_msg* msg);


void jee_usbh_root_hub_connect_handler(struct uhcd *hcd, jee_uint8_t port, jee_bool_t isHS);
void jee_usbh_root_hub_disconnect_handler(struct uhcd *hcd, jee_uint8_t port);

/* usb host controller driver interface */
jee_inline jee_err_t jee_usb_instance_add_pipe(uinst_t inst, upipe_t pipe)
{
    configASSERT(inst != JEE_NULL);
    configASSERT(pipe != JEE_NULL);
    jee_list_insert_before(&inst->pipe, &pipe->list);
    return JEE_EOK;
}
jee_inline upipe_t jee_usb_instance_find_pipe(uinst_t inst,jee_uint8_t ep_address)
{
    jee_list_t * l;
    for(l = inst->pipe.next;l != &inst->pipe;l = l->next)
    {
        if(jee_list_entry(l,struct upipe,list)->ep.bEndpointAddress == ep_address)
        {
            return jee_list_entry(l,struct upipe,list);
        }
    }
    return JEE_NULL;
}
jee_inline jee_err_t jee_usb_hcd_alloc_pipe(uhcd_t hcd, upipe_t* pipe, uinst_t inst, uep_desc_t ep)
{
    *pipe = (upipe_t)malloc(sizeof(struct upipe));
    if(*pipe == JEE_NULL)
    {
        return JEE_ERROR;
    }
    memset(*pipe,0,sizeof(struct upipe));
    (*pipe)->inst = inst;
    memcpy(&(*pipe)->ep,ep,sizeof(struct uendpoint_descriptor));
    return hcd->ops->open_pipe(*pipe);
}
jee_inline void jee_usb_pipe_add_callback(upipe_t pipe, func_callback callback)
{
    pipe->callback = callback;
}

jee_inline jee_err_t jee_usb_hcd_free_pipe(uhcd_t hcd, upipe_t pipe)
{
    configASSERT(pipe != JEE_NULL);
    hcd->ops->close_pipe(pipe);
    free(pipe);
    return JEE_EOK;
}

int jee_usb_hcd_pipe_xfer(uhcd_t hcd, upipe_t pipe, void* buffer, int nbytes, int timeout);
jee_inline int jee_usb_hcd_setup_xfer(uhcd_t hcd, upipe_t pipe, ureq_t setup, int timeout)
{
    return hcd->ops->pipe_xfer(pipe, USBH_PID_SETUP, (void *)setup, 8, timeout);
}

#ifdef __cplusplus
}
#endif

#endif
