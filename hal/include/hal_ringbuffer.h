/*
 * Copyright (c) 2022, Jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-03     Monk         First version.
 */
#ifndef HAL_RINGBUFFER_H__
#define HAL_RINGBUFFER_H__

#include "jeedef.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JEE_USING_HEAP
#define JEE_ALIGN_SIZE 4

/* ring buffer */
struct jee_ringbuffer
{
    jee_uint8_t *buffer_ptr;
    /* use the msb of the {read,write}_index as mirror bit. You can see this as
     * if the buffer adds a virtual mirror and the pointers point either to the
     * normal or to the mirrored buffer. If the write_index has the same value
     * with the read_index, but in a different mirror, the buffer is full.
     * While if the write_index and the read_index are the same and within the
     * same mirror, the buffer is empty. The ASCII art of the ringbuffer is:
     *
     *          mirror = 0                    mirror = 1
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Full
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     *  read_idx-^                   write_idx-^
     *
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Empty
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * read_idx-^ ^-write_idx
     *
     * The tradeoff is we could only use 32KiB of buffer for 16 bit of index.
     * But it should be enough for most of the cases.
     *
     * Ref: http://en.wikipedia.org/wiki/Circular_buffer#Mirroring */
    jee_uint16_t read_mirror : 1;
    jee_uint16_t read_index : 15;
    jee_uint16_t write_mirror : 1;
    jee_uint16_t write_index : 15;
    /* as we use msb of index as mirror bit, the size should be signed and
     * could only be positive. */
    jee_int16_t buffer_size;
};

enum jee_ringbuffer_state
{
    JEE_RINGBUFFER_EMPTY,
    JEE_RINGBUFFER_FULL,
    /* half full is neither full nor empty */
    JEE_RINGBUFFER_HALFFULL,
};

/**
 * RingBuffer for DeviceDriver
 *
 * Please note that the ring buffer implementation of RT-Thread
 * has no thread wait or resume feature.
 */
void jee_ringbuffer_init(struct jee_ringbuffer *rb, jee_uint8_t *pool, jee_int16_t size);
void jee_ringbuffer_reset(struct jee_ringbuffer *rb);
jee_size_t jee_ringbuffer_put(struct jee_ringbuffer *rb, const jee_uint8_t *ptr, jee_uint16_t length);
jee_size_t jee_ringbuffer_put_force(struct jee_ringbuffer *rb, const jee_uint8_t *ptr, jee_uint16_t length);
jee_size_t jee_ringbuffer_putchar(struct jee_ringbuffer *rb, const jee_uint8_t ch);
jee_size_t jee_ringbuffer_putchar_force(struct jee_ringbuffer *rb, const jee_uint8_t ch);
jee_size_t jee_ringbuffer_get(struct jee_ringbuffer *rb, jee_uint8_t *ptr, jee_uint16_t length);
jee_size_t jee_ringbuffer_peek(struct jee_ringbuffer *rb, jee_uint8_t **ptr);
jee_size_t jee_ringbuffer_getchar(struct jee_ringbuffer *rb, jee_uint8_t *ch);
jee_size_t jee_ringbuffer_data_len(struct jee_ringbuffer *rb);

#ifdef JEE_USING_HEAP
struct jee_ringbuffer* jee_ringbuffer_create(jee_uint16_t length);
void jee_ringbuffer_destroy(struct jee_ringbuffer *rb);
#endif

/**
 * @brief Get the buffer size of the ring buffer object.
 *
 * @param rb        A pointer to the ring buffer object.
 *
 * @return  Buffer size.
 */
jee_inline jee_uint16_t jee_ringbuffer_get_size(struct jee_ringbuffer *rb)
{
    configASSERT(rb != NULL);
    return rb->buffer_size;
}

/** return the size of empty space in rb */
#define jee_ringbuffer_space_len(rb) ((rb)->buffer_size - jee_ringbuffer_data_len(rb))


#ifdef __cplusplus
}
#endif

#endif
