idf_component_register(SRCS "main_io_slave.c"
                    INCLUDE_DIRS "${jeejio_path}/hal/include" "${jeejio_path}/kernel/include/freertos" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include/driver"
                    REQUIRES "driver"
                    PRIV_REQUIRES kernel)
