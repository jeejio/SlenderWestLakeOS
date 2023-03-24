idf_component_register(SRCS 
                       INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos" "${jeejio_path}/hal/include"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include/driver"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/vfs/include" 
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/fatfs/vfs"
                                        "${jeejio_path}/kernel/lib/fatfs"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/fatfs/wear_levelling/include"
                                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/storage/spi_flash/include"
                       REQUIRES driver hal
                       PRIV_REQUIRES  kernel hal)
					   
set(LIBS libfactory_test)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})
					   
jee_auto_init_list_register()
