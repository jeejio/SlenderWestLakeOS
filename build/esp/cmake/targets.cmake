#
# Set the target used for the standard project build.
#
macro(__target_init)
    # Input is JEEJIO_TARGET environement variable
    set(env_jeejio_target $ENV{JEEJIO_TARGET})

    if(NOT env_jeejio_target)
        # JEEJIO_TARGET not set in environment, see if it is set in cache
        if(JEEJIO_TARGET)
            set(env_jeejio_target ${JEEJIO_TARGET})
        else()
            set(env_jeejio_target esp32c3)
            message(STATUS "JEEJIO_TARGET not set, using default target: ${env_jeejio_target}")
        endif()
    else()
        # JEEJIO_TARGET set both in environment and in cache, must be the same
        if(NOT ${JEEJIO_TARGET} STREQUAL ${env_jeejio_target})
            message(FATAL_ERROR "JEEJIO_TARGET in CMake cache does not match "
                            "JEEJIO_TARGET environment variable. To change the target, clear "
                            "the build directory and sdkconfig file, and build the project again")
        endif()
    endif()

    # JEEJIO_TARGET will be used by Kconfig, make sure it is set
    set(ENV{JEEJIO_TARGET} ${env_jeejio_target})

    # Finally, set JEEJIO_TARGET in cache
    set(JEEJIO_TARGET ${env_jeejio_target} CACHE STRING "JEEJIO Build Target")
endmacro()

#
# Check that the set build target and the config target matches.
#
function(__target_check)
    # Should be called after sdkconfig CMake file has been included.
    idf_build_get_property(jeejio_target JEEJIO_TARGET)
    if(NOT ${jeejio_target} STREQUAL ${CONFIG_JEEJIO_TARGET})
        message(FATAL_ERROR "CONFIG_JEEJIO_TARGET in sdkconfig does not match "
            "JEEJIO_TARGET environment variable. To change the target, delete "
            "sdkconfig file and build the project again.")
    endif()
endfunction()

#
# Used by the project CMake file to set the toolchain before project() call.
#
macro(__target_set_toolchain)
    idf_build_get_property(jeejio_path JEEJIO_PATH)

    # See if Clang toolchain should be used
    set(env_jeejio_toolchain $ENV{JEEJIO_TOOLCHAIN})
    if(NOT env_jeejio_toolchain)
        # IDF_TOOLCHAIN not set in environment, see if it is set in cache
        if(JEEJIO_TOOLCHAIN)
            set(env_jeejio_toolchain ${JEEJIO_TOOLCHAIN})
        else()
            set(env_jeejio_toolchain gcc)
        endif()
    else()
        # IDF_TOOLCHAIN set both in environment and in cache, must be the same
        if(NOT ${JEEJIO_TOOLCHAIN} STREQUAL ${env_jeejio_toolchain})
            message(FATAL_ERROR "JEEJIO_TOOLCHAIN in CMake cache does not match "
                    "JEEJIO_TOOLCHAIN environment variable. To change the toolchain, clear "
                    "the build directory and sdkconfig file, and build the project again")
        endif()
    endif()

    # Finally, set IDF_TOOLCHAIN in cache
    set(JEEJIO_TOOLCHAIN ${env_jeejio_toolchain} CACHE STRING "JEEJIO Build Toolchain Type")

    # First try to load the toolchain file from the build/esp/cmake/directory of IDF
    set(toolchain_file_global ${jeejio_path}/build/esp/cmake/toolchain-${JEEJIO_TARGET}.cmake)
    if(EXISTS ${toolchain_file_global})
        set(CMAKE_TOOLCHAIN_FILE ${toolchain_file_global})
    else()
        message(FATAL_ERROR "Toolchain file ${toolchain_file_global} not found")
    endif()
endmacro()
