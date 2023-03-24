idf_component_register(SRCS "drv_uart_iomodel.c"
                       INCLUDE_DIRS "."
                       REQUIRES "driver" "hal"
                       PRIV_REQUIRES  kernel)

jee_auto_init_list_register()