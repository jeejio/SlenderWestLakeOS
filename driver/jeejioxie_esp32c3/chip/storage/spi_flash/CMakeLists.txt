idf_build_get_property(target JEEJIO_TARGET)
if(${target} STREQUAL "linux")
    return()
endif()

set(srcs "esp_app_desc.c")

if(BOOTLOADER_BUILD)
    set(cache_srcs "")
    set(priv_requires bootloader_support soc)
else()
    set(cache_srcs
        "cache_utils.c"
        "flash_mmap.c"
        "flash_ops.c"
        "${target}/flash_ops_${target}.c"
    )

    list(APPEND srcs "flash_brownout_hook.c")

    if(CONFIG_ESPTOOLPY_OCT_FLASH)
        list(APPEND srcs "${target}/spi_flash_oct_flash_init.c")
    endif()

    if(CONFIG_JEEJIO_TARGET_ESP32S3)
        list(APPEND srcs
            "esp32s3/spi_timing_config.c"
            "spi_flash_timing_tuning.c"
            "spi_flash_hpm_enable.c")
    endif()

    # New implementation after IDF v4.0
    list(APPEND srcs
        "spi_flash_chip_drivers.c"
        "spi_flash_chip_generic.c"
        "spi_flash_chip_issi.c"
        "spi_flash_chip_mxic.c"
        "spi_flash_chip_gd.c"
        "spi_flash_chip_winbond.c"
        "spi_flash_chip_boya.c"
        "spi_flash_chip_mxic_opi.c"
        "spi_flash_chip_th.c"
        "memspi_host_driver.c"
        "partition.c"
        "partition_target.c")

    list(APPEND cache_srcs
        "esp_flash_api.c"
        "esp_flash_spi_init.c"
        "spi_flash_os_func_app.c"
        "spi_flash_os_func_noos.c")

    list(APPEND srcs ${cache_srcs})
    set(priv_requires bootloader_support app_update soc driver)
endif()

idf_component_register(SRCS "${srcs}"
                    REQUIRES esp_hal
                    PRIV_REQUIRES "${priv_requires}"
                    INCLUDE_DIRS include
                    PRIV_INCLUDE_DIRS include/spi_flash
                    LDFRAGMENTS linker.lf)

# Avoid cache miss by unexpected inlineing when built by -Os
set_source_files_properties(${cache_srcs} PROPERTIES COMPILE_FLAGS "-fno-inline-functions")
if(CMAKE_C_COMPILER_ID MATCHES "GNU")
    # These flags are GCC specific
    set_property(SOURCE ${cache_srcs} APPEND_STRING PROPERTY COMPILE_FLAGS
        " -fno-inline-small-functions -fno-inline-functions-called-once")
endif()

if(NOT BOOTLOADER_BUILD)
    if(CONFIG_SPIRAM)
        # [refactor-todo]: requires "esp_psram" for few MMU usages in `flash_mmap.c`
        # will be replaced with MMU requirements
        idf_component_optional_requires(PRIVATE esp_psram)
    endif()
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-u esp_app_desc")
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
endif()

if(CONFIG_APP_PROJECT_VER_FROM_CONFIG)
    # Ignore current PROJECT_VER (which was set in __project_get_revision()).
    # Gets the version from the CONFIG_APP_PROJECT_VER.
    idf_build_set_property(PROJECT_VER "${CONFIG_APP_PROJECT_VER}")
endif()

# cut PROJECT_VER and PROJECT_NAME to required 32 characters.
idf_build_get_property(project_ver PROJECT_VER)
idf_build_get_property(project_name PROJECT_NAME)
string(SUBSTRING "${project_ver}" 0 31 PROJECT_VER_CUT)
string(SUBSTRING "${project_name}" 0 31 PROJECT_NAME_CUT)
message(STATUS "App \"${PROJECT_NAME_CUT}\" version: ${PROJECT_VER_CUT}")

set_source_files_properties(
    SOURCE "esp_app_desc.c"
    PROPERTIES COMPILE_DEFINITIONS
    "PROJECT_VER=\"${PROJECT_VER_CUT}\"; PROJECT_NAME=\"${PROJECT_NAME_CUT}\"")