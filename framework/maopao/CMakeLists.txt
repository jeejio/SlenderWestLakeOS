idf_component_register(SRCS
    INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos"
        "${jeejio_path}/hal/include" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include"  
        "${jeejio_path}/external/wifi"
        "${jeejio_path}/external/MQTTClient-RT"
        "${jeejio_path}/external/MQTTPacket"
        "${jeejio_path}/external/json"
        "${jeejio_path}/external/smart_config"
        "${jeejio_path}/external/mson"
    REQUIRES external
)

set(LIBS libmaopao)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})	
