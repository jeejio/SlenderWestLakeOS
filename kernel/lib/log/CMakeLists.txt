idf_build_get_property(target JEEJIO_TARGET)
#set(srcs "log.c" "log_buffers.c")
set(priv_requires "")

list(APPEND priv_requires soc esp_hal esp_hw_support)

idf_component_register(SRCS 
                    INCLUDE_DIRS "include"
                    LDFRAGMENTS linker.lf
                    PRIV_REQUIRES ${priv_requires})




 
set(LIBS liblog)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})



# Ideally, FreeRTOS shouldn't be included into bootloader build, so the 2nd check should be unnecessary
#if(kernel IN_LIST BUILD_COMPONENTS AND NOT BOOTLOADER_BUILD)
 #   target_sources(${COMPONENT_TARGET} PRIVATE log_freertos.c)
#else()
#    target_sources(${COMPONENT_TARGET} PRIVATE log_noos.c)
#endif()
