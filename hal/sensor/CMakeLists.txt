#hal： all submodel derived from device model

idf_build_get_property(jeejio_path  JEEJIO_PATH)

set(src_class
     "hal_class.c"
   )





idf_component_register(SRCS  ${src_class}
    PRIV_REQUIRES   )

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
jee_auto_init_list_register()




