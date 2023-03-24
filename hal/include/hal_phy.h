/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-14     wangqiang    the first version
 * 2022-08-17     xjy198903    add 1000M definition
 */

#include "hal_phy_mdio.h"

#ifndef __PHY_H__
#define __PHY_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Defines the PHY link speed. This is align with the speed for MAC. */
#define PHY_SPEED_10M   0U     /* PHY 10M speed. */
#define PHY_SPEED_100M  1U     /* PHY 100M speed. */
#define PHY_SPEED_1000M 2U     /* PHY 1000M speed. */

/* Defines the PHY link duplex. */
#define PHY_HALF_DUPLEX 0U     /* PHY half duplex. */
#define PHY_FULL_DUPLEX 1U     /* PHY full duplex. */

/*! @brief Defines the PHY loopback mode. */
#define PHY_LOCAL_LOOP  0U     /* PHY local loopback. */
#define PHY_REMOTE_LOOP 1U     /* PHY remote loopback. */

#define PHY_STATUS_OK      0U
#define PHY_STATUS_FAIL    1U
#define PHY_STATUS_TIMEOUT 2U

typedef struct jee_phy_msg
{
    jee_uint32_t reg;
    jee_uint32_t value;
}jee_phy_msg_t;

typedef struct jee_phy_device
{
    struct jee_device parent;
    struct jee_mdio_bus *bus;
    jee_uint32_t addr;
    struct jee_phy_ops *ops;
}jee_phy_t;

typedef jee_int32_t jee_phy_status;

struct jee_phy_ops
{
    jee_phy_status (*init)(void *object, jee_uint32_t phy_addr, jee_uint32_t src_clock_hz);
    jee_phy_status (*read)(jee_uint32_t reg, jee_uint32_t *data);
    jee_phy_status (*write)(jee_uint32_t reg, jee_uint32_t data);
    jee_phy_status (*loopback)(jee_uint32_t mode, jee_uint32_t speed, jee_bool_t enable);
    jee_phy_status (*get_link_status)(jee_bool_t *status);
    jee_phy_status (*get_link_speed_duplex)(jee_uint32_t *speed, jee_uint32_t *duplex);
};

jee_err_t jee_hw_phy_register(struct jee_phy_device *phy, const char *name);

#ifdef __cplusplus
}
#endif

#endif /* __PHY_H__*/
