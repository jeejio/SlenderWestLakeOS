#include <driver/spi_common.h>

#include <driver/spi_slave.h>
#include <hal_spi.h>
#include <driver/drv_spi_master_iomodel.h>
#include "esp_log.h"
#include "auto_init.h"


#define TAG "drv_spi_iomodel"

struct esp_spi_cs
{
    jee_int16_t pin;
};

static jee_err_t spi_master_init(struct jee_spi_device *device,spi_bus_device_configuration_t *param);

jee_err_t esp_spi_bus_attach_device(const char *bus_name, const char *device_name, jee_uint32_t pin);
static jee_err_t spi_master_init(struct jee_spi_device *device,spi_bus_device_configuration_t *param)
{
    esp_err_t ret = JEE_EOK;
	spi_device_handle_t spi_handle ;
       
    if(! &(param->spi_bus))
    {
        LogInfo(TAG, " bus config is not initialized ");
    }
    spi_bus_config_t bus_cfg=SPI_BUS_TEST_DEFAULT_CONFIG();
    bus_cfg.miso_io_num = param->spi_bus->miso_io_num;  //miso_io_num,//PIN_NUM_MISO,
    bus_cfg.mosi_io_num = param->spi_bus->mosi_io_num;//PIN_NUM_MOSI,
    bus_cfg.sclk_io_num = param->spi_bus->sclk_io_num;//PIN_NUM_CLK,
    bus_cfg.quadwp_io_num = param->spi_bus->quadwp_io_num;
    bus_cfg.quadhd_io_num = param->spi_bus->quadhd_io_num;
    bus_cfg.max_transfer_sz = param->spi_bus->max_transfer_sz;
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    if(! &(param->spi_device))
    {
        LogInfo(TAG, " device config is not initialized ");
    }

    spi_device_interface_config_t devcfg=SPI_DEVICE_TEST_DEFAULT_CONFIG();
    devcfg.command_bits=param->spi_device->command_bits;
    devcfg.address_bits=param->spi_device->address_bits;
    devcfg.dummy_bits=param->spi_device->dummy_bits;
    devcfg.clock_speed_hz=param->spi_device->clock_speed_hz;  
    devcfg.duty_cycle_pos=param->spi_device->duty_cycle_pos;  
    devcfg.mode=param->spi_device->mode;
    devcfg.spics_io_num= param->spi_device->spics_io_num;
    devcfg.queue_size=param->spi_device->queue_size;
    devcfg.flags=param->spi_device->flags;
    devcfg.pre_cb=param->spi_device->pre_cb;
    devcfg.post_cb=param->spi_device->post_cb;
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);

    device->user_data = spi_handle;  
    return ret;     
}

static jee_err_t spi_configure(struct jee_spi_device *device, struct jee_spi_configuration *cfg)
{
    esp_err_t ret = JEE_EOK;
    struct esp_spi_cs *cs_pin = (struct esp_spi_cs *)device->parent.user_data;
    if(cfg == NULL)
    {
        LogInfo(TAG,"--------------------->CFG is NULL");
    }
    ret = spi_master_init(device,(spi_bus_device_configuration_t *)(cfg->pConfig));      
    LogInfo(TAG, "spi_configure master");

    return ret;
}

static jee_dev_t spixfer(struct jee_spi_device *device, struct jee_spi_message *message)
{
    jee_err_t ret = JEE_ERROR;

    struct esp_spi_cs *cs = device->parent.user_data;

    spi_device_handle_t spi_handle = (spi_device_handle_t)device->user_data;
    spi_device_acquire_bus(spi_handle,portMAX_DELAY);
    spi_transaction_t t = {
        .tx_buffer = message->send_buf,
        .rx_buffer = message->recv_buf,
        .length = message->length*8,
        // .cmd = CMD_READ | (0 & ADDR_MASK),
    };
    if(message->send_buf)
    {
        if(message->length > 0)
        {
            ret = spi_device_polling_transmit(spi_handle,&t);
            //ret=spi_device_transmit(spi_handle, &t);
            spi_device_release_bus(spi_handle);
        }
    }else{
        ret = spi_device_polling_transmit(spi_handle,&t);
        spi_device_release_bus(spi_handle);
    }

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

int jee_spi_bus_init(void)
{ 
    return jee_spi_bus_register(&spi2_bus, SPI_BUS_NAME, &esp_spi_ops);
}
INIT_PREV_EXPORT(jee_spi_bus_init);