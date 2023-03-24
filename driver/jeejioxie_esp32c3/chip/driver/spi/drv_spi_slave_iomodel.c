#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <driver/spi_slave.h>
#include <hal_spi.h>
#include <driver/drv_spi_slave_iomodel.h>
#include "esp_log.h"
#include "auto_init.h"



#define TAG "drv_spi_iomodel"

struct esp_spi_cs
{
    jee_int16_t pin;
};


jee_err_t esp_spi_bus_attach_device(const char *bus_name, const char *device_name, jee_uint32_t pin);

static jee_err_t spi_configure(struct jee_spi_device *device, struct jee_spi_configuration *cfg)
{
    struct esp_spi_cs *cs_pin = (struct esp_spi_cs *)device->parent.user_data;
    spi_bus_config_t bus_cfg={
        .mosi_io_num=PIN_NUM_MOSI,
        .miso_io_num=PIN_NUM_MISO,
        .sclk_io_num=PIN_NUM_CLK
    };
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=cs_pin->pin,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=0,
        .post_trans_cb=0
    };
    spi_slave_initialize(SLAVE_HOST, &bus_cfg, &slvcfg, SPI_DMA_CH_AUTO);
    LogInfo(TAG, "spi_configure slave");
    return JEE_EOK;
}

static jee_dev_t spixfer(struct jee_spi_device *device, struct jee_spi_message *message)
{
    jee_err_t ret = JEE_ERROR;
    struct esp_spi_cs *cs = device->parent.user_data;

    spi_slave_transaction_t t={
        .tx_buffer = message->send_buf,
        .rx_buffer = message->recv_buf,
        .length=message->length*8,
    };
    ret = spi_slave_transmit(SLAVE_HOST, &t, portMAX_DELAY);

    return !ret;
}

jee_err_t esp_spi_bus_attach_device(const char *bus_name, const char *device_name, jee_uint32_t pin)
{
    jee_err_t ret;
    struct jee_spi_device *spi_device;
    struct esp_spi_cs *cs_pin;

    spi_device = (struct jee_spi_device *)malloc(sizeof(struct jee_spi_device));
    configASSERT(spi_device != JEE_NULL);

    cs_pin = (struct esp_spi_cs *)malloc(sizeof(struct esp_spi_cs));
    configASSERT(cs_pin != JEE_NULL);

    cs_pin->pin = pin;
    ret = jee_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs_pin);

    return ret;
}

static struct jee_spi_ops esp_spi_ops =
{
    .configure = spi_configure,
    .xfer = spixfer
};

struct jee_spi_bus spi2_bus;


int jee_spi_slave_init(void)
{ 
    return jee_spi_bus_register(&spi2_bus, SPI_BUS_NAME, &esp_spi_ops);
}
INIT_PREV_EXPORT(jee_spi_slave_init);