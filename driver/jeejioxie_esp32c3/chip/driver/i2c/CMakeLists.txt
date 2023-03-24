set(src   "drv_i2c_iomodel.c"
          "i2c.c"
)

set(includes   "${jeejio_path}/kernel/include/freertos"
               "${jeejio_path}/hal/include"
               "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include"
	       "${jeejio_path}/kernel/lib/ringbuf/include"
)

idf_component_register(SRCS ${src}
     INCLUDE_DIRS ${includes}
)
jee_auto_init_list_register()
