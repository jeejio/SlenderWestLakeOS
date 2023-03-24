idf_component_register(SRCS

    INCLUDE_DIRS "." 
        "${jeejio_path}/external/json"
        "${jeejio_path}/external/webclient"
        "${jeejio_path}/framework/maopao"
    REQUIRES external
)

set(LIBS libcloud)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})


