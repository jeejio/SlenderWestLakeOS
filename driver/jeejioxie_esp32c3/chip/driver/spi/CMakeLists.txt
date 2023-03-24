idf_component_register(SRCS "drv_spi_master_iomodel.c"
                       INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos" "${jeejio_path}/hal/include" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include/driver"
                       REQUIRES "driver" "hal"
                       PRIV_REQUIRES  kernel)


jee_auto_init_list_register()