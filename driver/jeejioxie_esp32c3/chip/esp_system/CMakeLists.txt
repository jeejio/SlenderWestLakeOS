idf_build_get_property(target JEEJIO_TARGET)

set(include_dirs "include" "include/${target}")

set(srcs "esp_err.c")

if(CONFIG_IDF_ENV_FPGA)
    list(APPEND srcs "fpga_overrides.c")
endif()

function(rom_linker_script name)
    target_linker_script(${COMPONENT_LIB} INTERFACE "ld/${target}/${target}.rom.${name}.ld")
endfunction()

list(APPEND srcs
        "rom/esp_rom_crc.c"
        "rom/esp_rom_sys.c"
        "rom/esp_rom_uart.c"
        "rom/esp_rom_spiflash.c"
        "rom/esp_rom_regi2c.c"
        "rom/esp_rom_efuse.c")

 if(CONFIG_HEAP_TLSF_USE_ROM_IMPL AND CONFIG_ESP_ROM_TLSF_CHECK_PATCH)
     # This file shall be included in the build if TLSF in ROM is activated
     list(APPEND srcs "rom/esp_rom_tlsf.c")
 endif()
 if(CONFIG_JEEJIO_TARGET_ARCH_XTENSA)
     list(APPEND srcs "rom/esp_rom_longjmp.S")
 endif()

if(BOOTLOADER_BUILD)
        # "_esp_error_check_failed()" requires spi_flash module
        # Bootloader relies on some Kconfig options defined in esp_system.
        idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "${include_dirs}" REQUIRES spi_flash LDFRAGMENTS "rom.lf")
else()
    list(APPEND srcs "crosscore_int.c"
            "esp_ipc.c"
            "esp_err.c"
            "freertos_hooks.c"
            "int_wdt.c"
            "panic.c"
            "esp_system.c"
            "startup.c"
            "system_time.c"
            "stack_check.c"
            "task_wdt.c"
            "ubsan.c"
            "xt_wdt.c"
            "debug_stubs.c"
            "timer/esp_timer.c"
            "timer/ets_timer_legacy.c"
            "timer/system_time.c"
            "pm/pm_locks.c"
            "pm/pm_trace.c"
            "pm/pm_impl.c")

    if(CONFIG_SOC_SYSTIMER_SUPPORTED)
        list(APPEND srcs "rom/esp_rom_systimer.c")
    endif()
    if(CONFIG_ESP_TIMER_IMPL_TG0_LAC)
        list(APPEND srcs "timer/esp_timer_impl_lac.c")
    elseif(CONFIG_ESP_TIMER_IMPL_SYSTIMER)
        list(APPEND srcs "timer/esp_timer_impl_systimer.c")
    endif()
    if(CONFIG_ESP_SYSTEM_USE_EH_FRAME)
        list(APPEND srcs "eh_frame_parser.c")
    endif()

    idf_component_register(SRCS "${srcs}"
                        INCLUDE_DIRS "${include_dirs}"
                        PRIV_INCLUDE_DIRS private_include
                        PRIV_REQUIRES spi_flash
                                    # [refactor-todo] requirements due to init code,
                                    # should be removable once using component init functions
                                    # link-time registration is used.
                                    # [refactor-todo] requires "driver" for headers:
                                    # - spi_common_internal.h
                                    # [refactor-todo] required for virtual efuse
                                    # init code. Move to esp_efuse component.
                                    bootloader_support efuse driver kernel
                        LDFRAGMENTS "linker.lf" "app.lf" "pm.lf" "rom.lf")

    add_subdirectory(port)

    # After system initialization, `start_app` (and its other cores variant) is called.
    # This is provided by the user or from another component. Since we can't establish
    # dependency on what we don't know, force linker to not drop the symbol regardless
    # of link line order.
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u start_app")

    if(NOT CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-u start_app_other_cores")
    endif()

    # Disable stack protection in files which are involved in initialization of that feature
    set_source_files_properties(
        "startup.c" "stack_check.c"
        PROPERTIES COMPILE_FLAGS
        -fno-stack-protector)

    include(${CMAKE_CURRENT_LIST_DIR}/ld/ld.cmake)
endif()

target_linker_script(${COMPONENT_LIB} INTERFACE "ld/${target}/${target}.rom.ld")
rom_linker_script("api")
if(CONFIG_COMPILER_FLOAT_LIB_FROM_GCCLIB)
    rom_linker_script("libgcc")
else()
    rom_linker_script("rvfp")
endif()
idf_build_get_property(time_t_size TIME_T_SIZE)

if(BOOTLOADER_BUILD)
if(target STREQUAL "esp32")
    rom_linker_script("newlib-funcs")
    if(NOT CONFIG_SPI_FLASH_ROM_DRIVER_PATCH)
        rom_linker_script("spiflash")
    endif()
    if(CONFIG_ESP32_REV_MIN_3)
        rom_linker_script("eco3")
    endif()

    elseif(target STREQUAL "esp32s2")
        rom_linker_script("newlib-funcs")
        rom_linker_script("spiflash")

    elseif(target STREQUAL "esp32s3")
        rom_linker_script("newlib")

    elseif(target STREQUAL "esp32c3")
        rom_linker_script("newlib")

    elseif(target STREQUAL "esp32h2")
        rom_linker_script("newlib")

    elseif(target STREQUAL "esp32c2")
        rom_linker_script("newlib")
        rom_linker_script("mbedtls")
    endif()
else() # Regular app build
    if(target STREQUAL "esp32")
        rom_linker_script("newlib-data")
        rom_linker_script("syscalls")
        if(NOT CONFIG_SPIRAM_CACHE_WORKAROUND)
            # ESP32 only: these ROM functions may only be used if PSRAM cache workaround is disabled.
            # Otherwise we need to link to a multilib version of libc compiled with PSRAM workaround.
            rom_linker_script("newlib-funcs")

            if(time_t_size EQUAL 4)
                # The ROM functions listed in this linker script depend on sizeof(time_t).
                # Since ROM for ESP32 was compiled for 32-bit time_t, only link these functions
                # if the toolchain is also using 32-bit time_t.
                rom_linker_script("newlib-time")

                if(CONFIG_NEWLIB_NANO_FORMAT)
                    # nano formatting functions in ROM are also built for 32-bit time_t.
                    rom_linker_script("newlib-nano")
                endif()
            endif()
        endif()

        if(NOT CONFIG_SPI_FLASH_ROM_DRIVER_PATCH)
            rom_linker_script("spiflash")
        endif()

        if(CONFIG_ESP32_REV_MIN_3)
            rom_linker_script("eco3")
        endif()

    elseif(target STREQUAL "esp32s2")
        rom_linker_script("newlib-funcs")
        rom_linker_script("newlib-data")
        rom_linker_script("spiflash")

        if(time_t_size EQUAL 4)
            # The ROM functions listed in this linker script depend on sizeof(time_t).
            # Since ROM for ESP32-S2 was compiled for 32-bit time_t, only link these functions
            # if the toolchain is also using 32-bit time_t.
            rom_linker_script("newlib-time")

            if(CONFIG_NEWLIB_NANO_FORMAT)
                # nano formatting functions in ROM are also built for 32-bit time_t.
                rom_linker_script("newlib-nano")
            endif()
        endif()

    elseif(target STREQUAL "esp32s3")
        rom_linker_script("newlib")
        rom_linker_script("version")

        if(time_t_size EQUAL 4)
            # The ROM functions listed in this linker script depend on sizeof(time_t).
            # Since ROM for ESP32-S3 was compiled for 32-bit time_t, only link these functions
            # if the toolchain is also using 32-bit time_t.
            rom_linker_script("newlib-time")

            if(CONFIG_NEWLIB_NANO_FORMAT)
                # nano formatting functions in ROM are also built for 32-bit time_t.
                rom_linker_script("newlib-nano")
            endif()
        endif()

    elseif(target STREQUAL "esp32c3")
        rom_linker_script("newlib")
        rom_linker_script("version")

        if(time_t_size EQUAL 4)
            # The ROM functions listed in this linker script depend on sizeof(time_t).
            # Since ROM for ESP32-C3 was compiled for 32-bit time_t, only link these functions
            # if the toolchain is also using 32-bit time_t.
            rom_linker_script("newlib-time")

            if(CONFIG_NEWLIB_NANO_FORMAT)
                # nano formatting functions in ROM are also built for 32-bit time_t.
                rom_linker_script("newlib-nano")
            endif()
        endif()

        if(CONFIG_ESP32C3_REV_MIN_3 OR CONFIG_ESP32C3_REV_MIN_4)
            rom_linker_script("eco3")
        endif()

    elseif(target STREQUAL "esp32h2")
        rom_linker_script("newlib")
        rom_linker_script("version")

        if(CONFIG_NEWLIB_NANO_FORMAT AND time_t_size EQUAL 4)
            # nano formatting functions in ROM are built for 32-bit time_t,
            # only link them if the toolchain is also using 32-bit time_t and nano formatting was requested.
            rom_linker_script("newlib-nano")
        endif()

    elseif(target STREQUAL "esp32c2")
        rom_linker_script("newlib")
        rom_linker_script("version")
        rom_linker_script("mbedtls")

        if(time_t_size EQUAL 8)
            # The ROM functions listed in this linker script depend on sizeof(time_t).
            # Since ROM for ESP32-C2 was compiled for 64-bit time_t, only link these functions
            # if the toolchain is also using 64-bit time_t.
            rom_linker_script("newlib-time")

            if(CONFIG_NEWLIB_NANO_FORMAT)
                # nano formatting functions in ROM are also built for 64-bit time_t.
                rom_linker_script("newlib-nano")
            endif()
        endif()

        if(CONFIG_HEAP_TLSF_USE_ROM_IMPL)
            # After registering the component, set the tlsf_set_rom_patches symbol as undefined
            # to force the linker to integrate the whole `esp_rom_tlsf.c` object file inside the
            # final binary. This is necessary because tlsf_set_rom_patches is a constructor, thus,
            # there as no explicit reference/call to it in IDF.
            if(CONFIG_ESP_ROM_TLSF_CHECK_PATCH)
                target_link_libraries(${COMPONENT_LIB} PRIVATE "-u tlsf_set_rom_patches")
            endif()

            rom_linker_script("heap")
        endif()
    endif()

    if(CONFIG_JEEJIO_TARGET_ARCH_XTENSA)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=longjmp")
    endif()
endif()

if(target STREQUAL "esp32s2")
    target_sources(${COMPONENT_LIB} PRIVATE "esp32s2/usb_descriptors.c")
endif()

if(CONFIG_IDF_ENV_FPGA)
    # Forces the linker to include fpga stubs from this component
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u esp_errno_include_fpga_overrides")
endif()

# Force linking UBSAN hooks. If UBSAN is not enabled, the hooks will ultimately be removed
# due to -ffunction-sections -Wl,--gc-sections options.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u __ubsan_include")


# [refactor-todo] requirements due to init code, should be removable
# once link-time registration of component init functions is used.
if(CONFIG_APPTRACE_ENABLE)
    idf_component_optional_requires(PRIVATE app_trace)
endif()

if(CONFIG_ESP_COREDUMP_ENABLE)
    idf_component_optional_requires(PRIVATE espcoredump)
endif()

# [refactor-todo] requirement from the panic handler,
# need to introduce panic "event" concept to remove this dependency (IDF-2194)
idf_component_optional_requires(PRIVATE esp_gdbstub)

if(CONFIG_PM_ENABLE)
    idf_component_optional_requires(PRIVATE pm)
endif()

if(CONFIG_VFS_SUPPORT_IO)
    idf_component_optional_requires(PRIVATE vfs)
endif()

if(CONFIG_SW_COEXIST_ENABLE OR CONFIG_EXTERNAL_COEX_ENABLE)
    idf_component_optional_requires(PRIVATE esp_wifi)
endif()

if(NOT BOOTLOADER_BUILD)
    if(CONFIG_SPIRAM)
        idf_component_optional_requires(PRIVATE esp_psram)
    endif()
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
