idf_build_get_property(target JEEJIO_TARGET)

if(NOT "${target}" STREQUAL "esp32c2") # TODO: IDF-4135
idf_component_register(SRCS "src/gdbstub.c" "src/packet.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       LDFRAGMENTS "linker.lf"
                       REQUIRES "kernel"
                       PRIV_REQUIRES "soc" "esp_system")
endif()

target_include_directories(${COMPONENT_LIB} PUBLIC "riscv" "${target}")
target_sources(${COMPONENT_LIB} PRIVATE "riscv/gdbstub_riscv.c" "${target}/gdbstub_${target}.c")
