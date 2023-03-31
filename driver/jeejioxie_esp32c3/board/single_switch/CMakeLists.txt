idf_component_register(SRCS  "single_switch.c" "drv_sensor_single_switch.c"

                       INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos" "${jeejio_path}/hal/include"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include/driver"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/vfs/include" 
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/fatfs/vfs"
                                        "${jeejio_path}/kernel/lib/fatfs"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/fatfs/wear_levelling/include"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/spi_flash/include"
                       REQUIRES "driver" "hal"
                       PRIV_REQUIRES  kernel)

jee_auto_init_list_register()