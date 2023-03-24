set(include_dirs
    include
    include/apps
    include/apps/sntp
    lwip/src/include
    port/esp32/include
    port/esp32/include/arch
    port/esp32/netif
    )


					
idf_component_register(SRCS 
                    INCLUDE_DIRS "${include_dirs}"
                    LDFRAGMENTS linker.lf
                    PRIV_REQUIRES vfs hal)					
					
					
					
set(LIBS liblwip)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})					
					
					
				

# lots of LWIP source files evaluate macros that check address of stack variables
#target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-address)

set_source_files_properties(
    lwip/src/netif/ppp/pppos.c
    PROPERTIES COMPILE_FLAGS
    -Wno-type-limits
    )
# "comparison is always false due to limited range of data type" warning
# when setting CONFIG_LWIP_TCP_WND_DEFAULT to 65535
set_source_files_properties(
    lwip/src/core/tcp.c
    PROPERTIES COMPILE_FLAGS
    -Wno-type-limits
)

# ignore some declaration mismatches
set_source_files_properties(
    lwip/src/netif/ppp/chap_ms.c
    PROPERTIES COMPILE_FLAGS
    -Wno-array-parameter
)

if(CONFIG_OPENTHREAD_ENABLED)
    idf_component_optional_requires(PRIVATE openthread)
endif()

if(CONFIG_ETH_ENABLED)
    idf_component_optional_requires(PRIVATE esp_eth)
endif()

if(CONFIG_LWIP_DHCP_RESTORE_LAST_IP)
    idf_component_optional_requires(PRIVATE nvs_flash)
endif()

target_compile_options(${COMPONENT_LIB} INTERFACE "-Wno-format")

