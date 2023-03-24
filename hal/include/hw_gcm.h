/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-14     tyx          the first version
 */

#ifndef __HW_GCM_H__
#define __HW_GCM_H__

#include "hw_symmetric.h"

#ifdef __cplusplus
extern "C" {
#endif

struct hwcrypto_gcm;

struct hwcrypto_gcm_ops
{
    jee_err_t (*start)(struct hwcrypto_gcm *gcm_ctx,
                      const unsigned char *add, jee_size_t add_len);    /**< Set additional data. start GCM operation */
    jee_err_t (*finish)(struct hwcrypto_gcm *gcm_ctx,
                       const unsigned char *tag, jee_size_t tag_len);   /**< finish GCM operation. get tag */
};

/**
 * @brief           GCM context. Hardware driver usage
 */
struct hwcrypto_gcm
{
    struct hwcrypto_symmetric parent;       /**< Inheritance from hardware symmetric crypto context */
    hwcrypto_type crypt_type;               /**< symmetric crypto type. eg: AES/DES */
    const struct hwcrypto_gcm_ops *ops;     /**< !! Hardware initializes this value when creating context !! */
};

/**
 * @brief           Creating GCM Context
 *
 * @param device    Hardware crypto device
 * @param type      Type of symmetric crypto context
 *
 * @return          GCM context
 */
struct jee_hwcrypto_ctx *jee_hwcrypto_gcm_create(struct jee_hwcrypto_device *device,
                                               hwcrypto_type crypt_type);

/**
 * @brief           Destroy GCM Context
 *
 * @param ctx       GCM context
 */
void jee_hwcrypto_gcm_destroy(struct jee_hwcrypto_ctx *ctx);

/**
 * @brief           This function starts a GCM encryption or decryption operation
 *
 * @param ctx       GCM context
 * @param add       The buffer holding the additional data
 * @param add_len   The length of the additional data
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_gcm_start(struct jee_hwcrypto_ctx *ctx, const jee_uint8_t *add,
                               jee_size_t add_len);

/**
 * @brief           This function finishes the GCM operation and generates the authentication tag
 *
 * @param ctx       GCM context
 * @param tag       The buffer for holding the tag
 * @param tag_len   The length of the tag to generate
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_gcm_finish(struct jee_hwcrypto_ctx *ctx, const jee_uint8_t *tag,
                                jee_size_t tag_len);

/**
 * @brief           This function performs a symmetric encryption or decryption operation
 *
 * @param ctx       GCM context
 * @param mode      Operation mode. HWCRYPTO_MODE_ENCRYPT or HWCRYPTO_MODE_DECRYPT
 * @param length    The length of the input data in Bytes. This must be a multiple of the block size
 * @param in        The buffer holding the input data
 * @param out       The buffer holding the output data
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_gcm_crypt(struct jee_hwcrypto_ctx *ctx, hwcrypto_mode mode,
                               jee_size_t length, const jee_uint8_t *in, jee_uint8_t *out);

/**
 * @brief           Set Symmetric Encryption and Decryption Key
 *
 * @param ctx       GCM context
 * @param key       The crypto key
 * @param bitlen    The crypto key bit length
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_gcm_setkey(struct jee_hwcrypto_ctx *ctx,
                                const jee_uint8_t *key, jee_uint32_t bitlen);

/**
 * @brief           Get Symmetric Encryption and Decryption Key
 *
 * @param ctx       GCM context
 * @param key       The crypto key buffer
 * @param bitlen    The crypto key bit length
 *
 * @return          Key length of copy
 */
jee_err_t jee_hwcrypto_gcm_getkey(struct jee_hwcrypto_ctx *ctx,
                                jee_uint8_t *key, jee_uint32_t bitlen);

/**
 * @brief           Set Symmetric Encryption and Decryption initialization vector
 *
 * @param ctx       GCM context
 * @param iv        The crypto initialization vector
 * @param len       The crypto initialization vector length
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_gcm_setiv(struct jee_hwcrypto_ctx *ctx,
                               const jee_uint8_t *iv, jee_size_t len);

/**
 * @brief           Get Symmetric Encryption and Decryption initialization vector
 *
 * @param ctx       GCM context
 * @param iv        The crypto initialization vector buffer
 * @param len       The crypto initialization vector buffer length
 *
 * @return          IV length of copy
 */
jee_err_t jee_hwcrypto_gcm_getiv(struct jee_hwcrypto_ctx *ctx,
                               jee_uint8_t *iv, jee_size_t len);

/**
 * @brief           Set offset in initialization vector
 *
 * @param ctx       GCM context
 * @param iv_off    The offset in IV
 */
void jee_hwcrypto_gcm_set_ivoff(struct jee_hwcrypto_ctx *ctx, jee_int32_t iv_off);

/**
 * @brief           Get offset in initialization vector
 *
 * @param ctx       GCM context
 * @param iv_off    It must point to a valid memory
 */
void jee_hwcrypto_gcm_get_ivoff(struct jee_hwcrypto_ctx *ctx, jee_int32_t *iv_off);

/**
 * @brief           This function copy GCM context
 *
 * @param des       The destination GCM context
 * @param src       The GCM context to be copy
 *
 * @return          JEE_EOK on success.
 */
jee_err_t jee_hwcrypto_gcm_cpy(struct jee_hwcrypto_ctx *des,
                             const struct jee_hwcrypto_ctx *src);

/**
 * @brief           Reset GCM context
 *
 * @param ctx       GCM context
 */
void jee_hwcrypto_gcm_reset(struct jee_hwcrypto_ctx *ctx);

#ifdef __cplusplus
}
#endif

#endif
