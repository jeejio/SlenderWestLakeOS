

idf_component_register(SRCS
                    INCLUDE_DIRS include)



set(LIBS libpthread)
add_library(${LIBS} STATIC IMPORTED)
set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${LIBS}.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})



idf_build_set_property(COMPILE_DEFINITIONS "-D_POSIX_READER_WRITER_LOCKS" APPEND)

set(extra_link_flags "-u pthread_include_pthread_impl")
list(APPEND extra_link_flags "-u pthread_include_pthread_cond_impl")
list(APPEND extra_link_flags "-u pthread_include_pthread_local_storage_impl")
list(APPEND extra_link_flags "-u pthread_include_pthread_rwlock_impl")

if(CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP AND NOT CONFIG_FREERTOS_SMP) # See IDF-4955
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=vPortCleanUpTCB")
endif()

if(extra_link_flags)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "${extra_link_flags}")
endif()

target_compile_options(${COMPONENT_LIB} INTERFACE "-Wno-format")



