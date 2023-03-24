idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "${jeejio_path}/hal/include" "${jeejio_path}/kernel/include/freertos"
                    REQUIRES "driver"
                    PRIV_REQUIRES kernel)
