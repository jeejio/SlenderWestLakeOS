#hal： all submodel derived from device model

idf_build_get_property(jeejio_path  JEEJIO_PATH)


set(includes  "include"
               "${jeejio_path}/kernel/include/freertos"
			"${jeejio_path}/framework/lwip/port/esp32"
			"${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/deprecated"
               "${jeejio_path}/driver/jeejioxie_esp32c3/chip/esp_wireless/bt/common/osi/include"
               "${jeejio_path}/hal/include"
               "${jeejio_path}/driver/jeejioxie_esp32c3/chip/esp_wireless/esp_wifi/include"
     )


idf_component_register(SRCS  
    INCLUDE_DIRS  ${includes}
    PRIV_REQUIRES lwip kernel esp_netif)


set(LIBS libhal)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})


target_compile_options(${COMPONENT_LIB} INTERFACE "-Wno-format")
jee_auto_init_list_register()


