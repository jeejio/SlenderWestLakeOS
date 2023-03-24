/*
 * Copyright (c) 2022, jeejio Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-1-10      zhengqian    first version
 */



#ifndef __SPI_H__
#define __SPI_H__

#include <stdlib.h>
#include <freertos/device.h>
#include <freertos/semphr.h>


#ifdef __cplusplus
extern "C"{
#endif

/**
 * At CPOL=0 the base value of the clock is zero
 *  - For CPHA=0, data are captured on the clock's rising edge (low->high transition)
 *    and data are propagated on a falling edge (high->low clock transition).
 *  - For CPHA=1, data are captured on the clock's falling edge and data are
 *    propagated on a rising edge.
 * At CPOL=1 the base value of the clock is one (inversion of CPOL=0)
 *  - For CPHA=0, data are captured on clock's falling edge and data are propagated
 *    on a rising edge.
 *  - For CPHA=1, data are captured on clock's rising edge and data are propagated
 *    on a falling edge.
 */
#define JEE_SPI_CPHA     (1<<0)                             /* bit[0]:CPHA, clock phase */
#define JEE_SPI_CPOL     (1<<1)                             /* bit[1]:CPOL, clock polarity */

#define JEE_SPI_LSB      (0<<2)                             /* bit[2]: 0-LSB */
#define JEE_SPI_MSB      (1<<2)                             /* bit[2]: 1-MSB */

#define JEE_SPI_MASTER   (0<<3)                             /* SPI master device */
#define JEE_SPI_SLAVE    (1<<3)                             /* SPI slave device */

#define JEE_SPI_CS_HIGH  (1<<4)                             /* Chipselect active high */
#define JEE_SPI_NO_CS    (1<<5)                             /* No chipselect */
#define JEE_SPI_3WIRE    (1<<6)                             /* SI/SO pin shared */
#define JEE_SPI_READY    (1<<7)                             /* Slave pulls low to pause */

#define JEE_SPI_MODE_MASK    (JEE_SPI_CPHA | JEE_SPI_CPOL | JEE_SPI_MSB | JEE_SPI_SLAVE | JEE_SPI_CS_HIGH | JEE_SPI_NO_CS | JEE_SPI_3WIRE | JEE_SPI_READY)

#define JEE_SPI_MODE_0       (0 | 0)                        /* CPOL = 0, CPHA = 0 */
#define JEE_SPI_MODE_1       (0 | JEE_SPI_CPHA)              /* CPOL = 0, CPHA = 1 */
#define JEE_SPI_MODE_2       (JEE_SPI_CPOL | 0)              /* CPOL = 1, CPHA = 0 */
#define JEE_SPI_MODE_3       (JEE_SPI_CPOL | JEE_SPI_CPHA)    /* CPOL = 1, CPHA = 1 */

#define JEE_SPI_BUS_MODE_SPI         (1<<0)
#define JEE_SPI_BUS_MODE_QSPI        (1<<1)

/**
 * SPI message structure
 */
struct jee_spi_message
{
    const void *send_buf;
    void *recv_buf;
    jee_size_t length;
    struct jee_spi_message *next;

    unsigned cs_take    : 1;
    unsigned cs_release : 1;
};

/**
 * SPI configuration structure
 */
struct jee_spi_configuration
{
    jee_uint8_t mode;
    jee_uint8_t data_width;
    jee_uint16_t reserved;

    jee_uint32_t max_hz;
    void *pConfig;
};

struct jee_spi_ops;
struct jee_spi_bus
{
    struct jee_device parent;
    jee_uint8_t mode;
    const struct jee_spi_ops *ops;
    SemaphoreHandle_t lock;
    struct jee_spi_device *owner;
};

/**
 * SPI operators
 */
struct jee_spi_ops
{
    jee_err_t (*configure)(struct jee_spi_device *device, struct jee_spi_configuration *configuration);
    jee_uint32_t (*xfer)(struct jee_spi_device *device, struct jee_spi_message *message);
};

/**
 * SPI Virtual BUS, one device must connected to a virtual BUS
 */
struct jee_spi_device
{
    struct jee_device parent;
    struct jee_spi_bus *bus;

    struct jee_spi_configuration config;
    void   *user_data;
};

struct jee_qspi_message
{
    struct jee_spi_message parent;

    /* instruction stage */
    struct
    {
        jee_uint8_t content;
        jee_uint8_t qspi_lines;
    } instruction;

    /* address and alternate_bytes stage */
    struct
    {
        jee_uint32_t content;
        jee_uint8_t size;
        jee_uint8_t qspi_lines;
    } address, alternate_bytes;

    /* dummy_cycles stage */
    jee_uint32_t dummy_cycles;

    /* number of lines in qspi data stage, the other configuration items are in parent */
    jee_uint8_t qspi_data_lines;
};

struct jee_qspi_configuration
{
    struct jee_spi_configuration parent;
    /* The size of medium */
    jee_uint32_t medium_size;
    /* double data rate mode */
    jee_uint8_t ddr_mode;
    /* the data lines max width which QSPI bus supported, such as 1, 2, 4 */
    jee_uint8_t qspi_dl_width ;
};

struct jee_qspi_device
{
    struct jee_spi_device parent;

    struct jee_qspi_configuration config;

    void (*enter_qspi_mode)(struct jee_qspi_device *device);

    void (*exit_qspi_mode)(struct jee_qspi_device *device);
};

#define SPI_DEVICE(dev) ((struct jee_spi_device *)(dev))

/* register a SPI bus */
jee_err_t jee_spi_bus_register(struct jee_spi_bus       *bus,
                             const char              *name,
                             const struct jee_spi_ops *ops);

/* attach a device on SPI bus */
jee_err_t jee_spi_bus_attach_device(struct jee_spi_device *device,
                                  const char           *name,
                                  const char           *bus_name,
                                  void                 *user_data);

/**
 * This function takes SPI bus.
 *
 * @param device the SPI device attached to SPI bus
 *
 * @return JEE_EOK on taken SPI bus successfully. others on taken SPI bus failed.
 */
jee_err_t jee_spi_take_bus(struct jee_spi_device *device);

/**
 * This function releases SPI bus.
 *
 * @param device the SPI device attached to SPI bus
 *
 * @return JEE_EOK on release SPI bus successfully.
 */
jee_err_t jee_spi_release_bus(struct jee_spi_device *device);

/**
 * This function take SPI device (takes CS of SPI device).
 *
 * @param device the SPI device attached to SPI bus
 *
 * @return JEE_EOK on release SPI bus successfully. others on taken SPI bus failed.
 */
jee_err_t jee_spi_take(struct jee_spi_device *device);

/**
 * This function releases SPI device (releases CS of SPI device).
 *
 * @param device the SPI device attached to SPI bus
 *
 * @return JEE_EOK on release SPI device successfully.
 */
jee_err_t jee_spi_release(struct jee_spi_device *device);

/* set configuration on SPI device */
jee_err_t jee_spi_configure(struct jee_spi_device        *device,
                          struct jee_spi_configuration *cfg);

/* send data then receive data from SPI device */
jee_err_t jee_spi_send_then_recv(struct jee_spi_device *device,
                               const void           *send_buf,
                               jee_size_t             send_length,
                               void                 *recv_buf,
                               jee_size_t             recv_length);

jee_err_t jee_spi_send_then_send(struct jee_spi_device *device,
                               const void           *send_buf1,
                               jee_size_t             send_length1,
                               const void           *send_buf2,
                               jee_size_t             send_length2);

/**
 * This function transmits data to SPI device.
 *
 * @param device the SPI device attached to SPI bus
 * @param send_buf the buffer to be transmitted to SPI device.
 * @param recv_buf the buffer to save received data from SPI device.
 * @param length the length of transmitted data.
 *
 * @return the actual length of transmitted.
 */
jee_size_t jee_spi_transfer(struct jee_spi_device *device,
                          const void           *send_buf,
                          void                 *recv_buf,
                          jee_size_t             length);

/**
 * This function transfers a message list to the SPI device.
 *
 * @param device the SPI device attached to SPI bus
 * @param message the message list to be transmitted to SPI device
 *
 * @return JEE_NULL if transmits message list successfully,
 *         SPI message which be transmitted failed.
 */
struct jee_spi_message *jee_spi_transfer_message(struct jee_spi_device  *device,
                                               struct jee_spi_message *message);

jee_inline jee_size_t jee_spi_recv(struct jee_spi_device *device,
                                void                 *recv_buf,
                                jee_size_t             length)
{
    return jee_spi_transfer(device, JEE_NULL, recv_buf, length);
}

jee_inline jee_size_t jee_spi_send(struct jee_spi_device *device,
                                const void           *send_buf,
                                jee_size_t             length)
{
    return jee_spi_transfer(device, send_buf, JEE_NULL, length);
}

jee_inline jee_uint8_t jee_spi_sendrecv8(struct jee_spi_device *device,
                                      jee_uint8_t            data)
{
    jee_uint8_t value = 0;

    jee_spi_send_then_recv(device, &data, 1, &value, 1);

    return value;
}

jee_inline jee_uint16_t jee_spi_sendrecv16(struct jee_spi_device *device,
                                        jee_uint16_t           data)
{
    jee_uint16_t value = 0;

    jee_spi_send_then_recv(device, &data, 2, &value, 2);

    return value;
}

/**
 * This function appends a message to the SPI message list.
 *
 * @param list the SPI message list header.
 * @param message the message pointer to be appended to the message list.
 */
jee_inline void jee_spi_message_append(struct jee_spi_message *list,
                                     struct jee_spi_message *message)
{
    configASSERT(list != JEE_NULL);
    if (message == JEE_NULL)
        return; /* not append */

    while (list->next != JEE_NULL)
    {
        list = list->next;
    }

    list->next = message;
    message->next = JEE_NULL;
}

/**
 * This function can set configuration on QSPI device.
 *
 * @param device the QSPI device attached to QSPI bus.
 * @param cfg the configuration pointer.
 *
 * @return the actual length of transmitted.
 */
jee_err_t jee_qspi_configure(struct jee_qspi_device *device, struct jee_qspi_configuration *cfg);

/**
 * This function can register a SPI bus for QSPI mode.
 *
 * @param bus the SPI bus for QSPI mode.
 * @param name The name of the spi bus.
 * @param ops the SPI bus instance to be registered.
 *
 * @return the actual length of transmitted.
 */
jee_err_t jee_qspi_bus_register(struct jee_spi_bus *bus, const char *name, const struct jee_spi_ops *ops);

/**
 * This function transmits data to QSPI device.
 *
 * @param device the QSPI device attached to QSPI bus.
 * @param message the message pointer.
 *
 * @return the actual length of transmitted.
 */
jee_size_t jee_qspi_transfer_message(struct jee_qspi_device  *device, struct jee_qspi_message *message);

/**
 * This function can send data then receive data from QSPI device
 *
 * @param device the QSPI device attached to QSPI bus.
 * @param send_buf the buffer to be transmitted to QSPI device.
 * @param send_length the number of data to be transmitted.
 * @param recv_buf the buffer to be recivied from QSPI device.
 * @param recv_length the data to be recivied.
 *
 * @return the status of transmit.
 */
jee_err_t jee_qspi_send_then_recv(struct jee_qspi_device *device, const void *send_buf, jee_size_t send_length,void *recv_buf, jee_size_t recv_length);

/**
 * This function can send data to QSPI device
 *
 * @param device the QSPI device attached to QSPI bus.
 * @param send_buf the buffer to be transmitted to QSPI device.
 * @param send_length the number of data to be transmitted.
 *
 * @return the status of transmit.
 */
jee_err_t jee_qspi_send(struct jee_qspi_device *device, const void *send_buf, jee_size_t length);

#ifdef __cplusplus
}
#endif

#endif
