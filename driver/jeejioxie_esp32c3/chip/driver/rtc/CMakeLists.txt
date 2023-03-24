idf_component_register(SRCS "drv_rtc_iomodel.c"
                       INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos" "${jeejio_path}/hal/include" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/esp_common/newlib/platform_include"
                       REQUIRES "driver" "hal"
                       PRIV_REQUIRES  kernel)

jee_auto_init_list_register()
