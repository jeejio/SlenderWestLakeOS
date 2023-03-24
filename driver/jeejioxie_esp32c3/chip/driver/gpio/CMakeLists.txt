idf_component_register(SRCS "drv_gpio_iomodel.c"
                       INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos" "${jeejio_path}/hal/include" 
                       REQUIRES "driver" "hal"
                       PRIV_REQUIRES  kernel)

jee_auto_init_list_register()