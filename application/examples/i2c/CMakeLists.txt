idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."  "${jeejio_path}/hal/include" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include" "${jeejio_path}/kernel/include/freertos"
                    REQUIRES "driver" "i2c"
                    PRIV_REQUIRES kernel)
