get_property(__jeejio_env_set GLOBAL PROPERTY __JEEJIO_ENV_SET)
if(NOT __jeejio_env_set)

    # Infer an JEEJIO_PATH relative to the cmake directory
    get_filename_component(_jeejio_path "${CMAKE_CURRENT_LIST_DIR}/../../.." REALPATH)
    file(TO_CMAKE_PATH "${_jeejio_path}" _jeejio_path)

    # Get the path set in environment
    set(jeejio_path $ENV{JEEJIO_PATH})

    # Environment JEEJIO_PATH should match the inferred JEEJIO_PATH. If not, warn the user.
    # (Note: REALPATH is needed in both above steps to account for case on case
    # insensitive filesystems, or relative paths)
    if(jeejio_path)
        get_filename_component(jeejio_path "${jeejio_path}" REALPATH)
        file(TO_CMAKE_PATH "${jeejio_path}" jeejio_path)

        if(NOT jeejio_path STREQUAL _jeejio_path)
            message(WARNING "JEEJIO_PATH environment variable is different from inferred JEEJIO_PATH.
                            Check if your project's top-level CMakeLists.txt includes the right
                            CMake files. Environment JEEJIO_PATH will be used for the build:
                            ${jeejio_path}")
        endif()
    else()
        message(WARNING "JEEJIO_PATH environment variable not found. Setting JEEJIO_PATH to '${_jeejio_path}'.")
        set(jeejio_path ${_jeejio_path})
        set(ENV{JEEJIO_PATH} ${_jeejio_path})
    endif() 

    # Include other CMake modules required
    set(CMAKE_MODULE_PATH
        "${jeejio_path}/build/esp/cmake"
        ${CMAKE_MODULE_PATH})
    include(build)

    set(JEEJIO_PATH ${jeejio_path})

    include(CheckCCompilerFlag)
    include(CheckCXXCompilerFlag)
    include(kconfig)
    include(component)
    include(utilities)
    include(targets)
    include(ldgen)
    include(dfu)
    include(uf2)
    include(version)

    __build_init("${jeejio_path}")

    # Check if IDF_ENV_FPGA environment is set
    set(env_idf_env_fpga $ENV{IDF_ENV_FPGA})
    if(${env_idf_env_fpga})
        idf_build_set_property(__IDF_ENV_FPGA "y")
        message(NOTICE "IDF_ENV_FPGA is set, building for FPGA environment")
    endif()

    set_property(GLOBAL PROPERTY __JEEJIO_ENV_SET 1)
endif()
