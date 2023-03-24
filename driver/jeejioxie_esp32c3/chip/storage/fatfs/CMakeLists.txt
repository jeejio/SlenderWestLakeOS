idf_build_get_property(jeejio_path JEEJIO_PATH)

set(fatfs_path "${jeejio_path}/kernel/lib/fatfs")

set(srcs "diskio/diskio.c"
         "diskio/diskio_rawflash.c"
         "diskio/diskio_sdmmc.c"
         "diskio/diskio_wl.c"
         "ff.c"
         "ffsystem.c"
         "ffunicode.c"
         "vfs/vfs_fat.c"
         "vfs/vfs_fat_sdmmc.c"
         "vfs/vfs_fat_spiflash.c"
         "wear_levelling/Partition.cpp"
         "wear_levelling/SPI_Flash.cpp"
         "wear_levelling/WL_Ext_Perf.cpp"
         "wear_levelling/WL_Ext_Safe.cpp"
         "wear_levelling/WL_Flash.cpp"
         "wear_levelling/crc32.cpp"
         "wear_levelling/wear_levelling.cpp")

#LIST(APPEND srcs ${wear_level_src})

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS diskio vfs ${fatfs_path} "wear_levelling/include"
                       REQUIRES sdmmc spi_flash
					   PRIV_INCLUDE_DIRS "wear_levelling/private_include"
                       PRIV_REQUIRES vfs spi_flash
                      )

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
