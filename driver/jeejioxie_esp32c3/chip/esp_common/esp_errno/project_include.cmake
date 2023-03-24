#
# Warn if the toolchain version doesn't match
#

if(NOT ${target} STREQUAL "linux" AND CMAKE_C_COMPILER_ID MATCHES "Clang")
    message(WARNING "Building ESP-IDF with clang is an experimental feature and is not yet officially supported.")
endif()
