idf_build_get_property(target JEEJIO_TARGET)
idf_build_get_property(arch JEEJIO_TARGET_ARCH)

if(NOT "${arch}" STREQUAL "riscv")
    return()
endif()


if(BOOTLOADER_BUILD)
    set(priv_requires soc)
else()
    set(priv_requires soc)
    set(srcs
        "instruction_decode.c"
        "interrupt.c"
        "vectors.S")
endif()

idf_component_register(SRCS 
                       LDFRAGMENTS linker.lf
                       INCLUDE_DIRS "include"
#                       PRIV_REQUIRES ${priv_requires}
)



set(LIBS libriscv)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})
