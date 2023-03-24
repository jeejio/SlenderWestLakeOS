/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-04-23     tyx          the first version
 */

#ifndef __HW_HASH_H__
#define __HW_HASH_H__

#include <hwcrypto.h>

#ifdef __cplusplus
extern "C" {
#endif

struct hwcrypto_hash;

struct hwcrypto_hash_ops
{
    jee_err_t (*update)(struct hwcrypto_hash *hash_ctx,
                       const jee_uint8_t *in, jee_size_t length);     /**< Processing a packet of data */
    jee_err_t (*finish)(struct hwcrypto_hash *hash_ctx,
                       jee_uint8_t *out, jee_size_t length);          /**< Get the final hash value */
};

/**
 * @brief           hash context. Hardware driver usage
 */
struct hwcrypto_hash
{
    struct jee_hwcrypto_ctx parent;              /**< Inheritance from hardware crypto context */
    const struct hwcrypto_hash_ops *ops;        /**< !! Hardware initializes this value when creating context !! */
};

/**
 * @brief           Creating hash Context
 *
 * @param device    Hardware crypto device
 * @param type      Type of hash context
 *
 * @return          Hash context
 */
struct jee_hwcrypto_ctx *jee_hwcrypto_hash_create(struct jee_hwcrypto_device *device,
                                                hwcrypto_type type);

/**
 * @brief           Destroy hash Context
 *
 * @param ctx       Hash context
 */
void jee_hwcrypto_hash_destroy(struct jee_hwcrypto_ctx *ctx);

/**
 * @brief           Get the final hash value
 *
 * @param ctx       Hash context
 * @param output    Hash value buffer
 * @param length    Hash value buffer length
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_hash_finish(struct jee_hwcrypto_ctx *ctx, jee_uint8_t *output, jee_size_t length);

/**
 * @brief           Processing a packet of data
 *
 * @param ctx       Hash context
 * @param input     Data buffer to be Processed
 * @param length    Data Buffer length
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_hash_update(struct jee_hwcrypto_ctx *ctx, const jee_uint8_t *input, jee_size_t length);

/**
 * @brief           This function copy hash context
 *
 * @param des       The destination hash context
 * @param src       The hash context to be copy
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_hash_cpy(struct jee_hwcrypto_ctx *des, const struct jee_hwcrypto_ctx *src);

/**
 * @brief           Reset hash context
 *
 * @param ctx       Hash context
 */
void jee_hwcrypto_hash_reset(struct jee_hwcrypto_ctx *ctx);

/**
 * @brief           Setting hash context type
 *
 * @param ctx       Hash context
 * @param type      Types of settings
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_hash_set_type(struct jee_hwcrypto_ctx *ctx, hwcrypto_type type);

#ifdef __cplusplus
}
#endif

#endif
