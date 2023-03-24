# Note: we can't expand these environment variables in the main IDF CMake build,
# because we want to expand them at flashing time not at CMake runtime (so they can change
# without needing a CMake re-run)
set(JEEJIOPORT $ENV{JEEJIOPORT})
if(NOT JEEJIOPORT)
    message("Note: ${SERIAL_TOOL} will search for a serial port. "
            "To specify a port, set the JEEJIOPORT environment variable.")
else()
    set(port_arg "-p ${JEEJIOPORT}")
endif()

set(JEEJIOBAUD $ENV{JEEJIOBAUD})
if(NOT JEEJIOBAUD)
    message("Note: ${SERIAL_TOOL} will attempt to set baud rate automatically. "
            "To specify a baud rate, set the JEEJIOBAUD environment variable.")
else()
    set(baud_arg "-b ${JEEJIOBAUD}")
endif()
