idf_component_register(SRCS "main.c" "net_socket.c" "smt_connect.c"
                    INCLUDE_DIRS "." "${jeejio_path}/hal/include" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include" "${jeejio_path}/kernel/include/freertos"
                    REQUIRES esp_wifi nvs_flash)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

jee_auto_init_list_register()

