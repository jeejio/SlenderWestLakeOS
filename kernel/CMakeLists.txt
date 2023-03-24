if(BOOTLOADER_BUILD)
    # bootloader only needs FreeRTOS for config, not for anything else
    idf_component_register()
    return()
endif()

idf_build_get_property(target JEEJIO_TARGET)
idf_build_get_property(jeejio_path JEEJIO_PATH)

set(kernel_dir "core")
set(portable_dir "${jeejio_path}/driver/jeejioxie_esp32c3/chip/esp_common/esp_addition")

set(ldfragments ${portable_dir}/linker.lf)


if(CONFIG_JEEJIO_TARGET_ARCH_XTENSA)

    set(include_dirs
        include
        ${portable_dir}/include/freertos          # For files with #include "FreeRTOSConfig.h"
        ${portable_dir}/portable/xtensa/include # For arch-specific FreeRTOSConfig_arch.h in portable/<arch>/include
        ${portable_dir}/include)                  # For files with #include "freertos/FreeRTOSConfig.h"
    set(private_include_dirs
        ${portable_dir}/portable/xtensa/include/freertos
        ${portable_dir}/portable/xtensa
        ${portable_dir}/portable/priv_include
        .)
elseif(CONFIG_JEEJIO_TARGET_ARCH_RISCV)

    set(include_dirs
        include
        ${portable_dir}/include/freertos          # For files with #include "FreeRTOSConfig.h"
        ${portable_dir}/portable/riscv/include  # For arch-specific FreeRTOSConfig_arch.h in portable/<arch>/include
        ${portable_dir}/include)                  # For files with #include "freertos/FreeRTOSConfig.h"
    set(private_include_dirs
        ${portable_dir}/portable/riscv/include/freertos
        ${portable_dir}/portable/riscv
        ${portable_dir}/portable/priv_include
        .)
endif()

list(APPEND include_dirs
    "auto_init")                              # For file with include "auto_init.h"
list(APPEND private_include_dirs
    "include/freertos"
    "${portable_dir}/private_include")    # For include "freertos_tasks_c_additions.h"


idf_component_register(SRCS 
                    INCLUDE_DIRS ${include_dirs}
                    PRIV_INCLUDE_DIRS  ${private_include_dirs}
                    LDFRAGMENTS "${ldfragments}"
                    PRIV_REQUIRES soc)


set(LIBS libkernel)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})






#idf_component_set_property(freertos ORIG_INCLUDE_PATH "${jeejio_path}/kernel/include/freertos/")

if(CONFIG_FREERTOS_DEBUG_OCDAWARE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=uxTopUsedPriority") #will be removed
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=FreeRTOS_openocd_params")
    idf_build_set_property(COMPILE_OPTIONS "-DconfigENABLE_FREERTOS_DEBUG_OCDAWARE=1" APPEND)
endif()

set_source_files_properties(
    tasks.c
    event_groups.c
    timers.c
    queue.c
    stream_buffer.c
    PROPERTIES COMPILE_DEFINITIONS
    _ESP_FREERTOS_INTERNAL
    )

# The freertos component provides the `start_app` and `start_app_other_cores`
# if it is included in the build. It then calls `app_main`
# from the main task created, which must be provided by the user.
# Like for `start_app` and `start_app_other_cores`,
# we can't establish dependency on what we don't yet know, so we force the
# linker to not drop this symbol.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u app_main")

if(CONFIG_APPTRACE_SV_ENABLE)
    # FreeRTOS headers have a dependency on app_trace when SystemView tracing is enabled
    idf_component_optional_requires(PUBLIC app_trace)
elseif(CONFIG_APPTRACE_ENABLE)
    # [refactor-todo]: port.c has a dependency on esp_apptrace_init, for running it on APP CPU.
    # this should be resolved when link-time registration of startup functions is added.
    idf_component_optional_requires(PRIVATE app_trace)
endif()

if(CONFIG_ESP_SYSTEM_GDBSTUB_RUNTIME)
    # [refactor-todo]: port.c esp_startup_start_app_common() calls esp_gdbstub_init()
    idf_component_optional_requires(PRIVATE esp_gdbstub)
endif()

if(CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER)
    # [refactor-todo]: esp_timer is required by FreeRTOS when we use esp_timer_get_time() to do profiling
    # Introduce a port wrapper function to avoid including esp_timer.h into the public header
    idf_component_optional_requires(PUBLIC esp_system)
endif()

if(CONFIG_SPIRAM)
    idf_component_optional_requires(PRIVATE esp_psram)
endif()
