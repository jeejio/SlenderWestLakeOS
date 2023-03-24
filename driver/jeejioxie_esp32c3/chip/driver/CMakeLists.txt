idf_build_get_property(target JEEJIO_TARGET)

set(srcs
    "adc/adc_cali.c"
    "adc/adc_cali_curve_fitting.c"
    "adc/adc_oneshot.c"
    "adc/adc_common.c"
    "adc/deprecated/esp_adc_cal_common_legacy.c"
    "gpio/gpio.c"
    "gpio/rtc_io.c"
    "gptimer.c"
    "pwm/ledc.c"
    "spi/sdspi_crc.c"
    "spi/sdspi_host.c"
    "spi/sdspi_transaction.c"
    "spi/spi_common.c"
    "spi/spi_master.c"
    "spi/spi_slave.c"
    "spi/spi_bus_lock.c"
    "uart/uart.c"
    )

if(CONFIG_SOC_ADC_DMA_SUPPORTED)
    list(APPEND srcs "adc/adc_continuous.c")
endif()

# line fitting scheme
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/adc/${target}/adc_cali_line_fitting.c")
    list(APPEND srcs "adc/${target}/adc_cali_line_fitting.c")
endif()

# curve fitting scheme coefficients
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/adc/${target}/curve_fitting_coefficients.c")
    list(APPEND srcs "adc/${target}/curve_fitting_coefficients.c")
endif()

# legacy calibration driver
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/adc/deprecated/${target}/esp_adc_cal_legacy.c")
    list(APPEND srcs "adc/deprecated/${target}/esp_adc_cal_legacy.c")
endif()

# deprecated source files
list(APPEND srcs "deprecated/timer_legacy.c")

set(includes "include" "deprecated" "${jeejio_path}/hal/include" "${jeejio_path}/kernel/include/freertos" "adc/interface" "adc/${target}/include" "adc/deprecated/include")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${target}/include")
    list(APPEND includes "${target}/include")
endif()

if(CONFIG_SOC_ADC_SUPPORTED)
    list(APPEND srcs "deprecated/adc_legacy.c")
endif()

if(CONFIG_SOC_ADC_DMA_SUPPORTED)
    list(APPEND srcs "deprecated/adc_dma_legacy.c")
endif()

if(CONFIG_SOC_DEDICATED_GPIO_SUPPORTED)
    list(APPEND srcs "gpio/dedic_gpio.c")
endif()

if(CONFIG_SOC_SDM_SUPPORTED)
    list(APPEND srcs "sdm.c" "deprecated/sigma_delta_legacy.c")
endif()

if(CONFIG_SOC_RMT_SUPPORTED)
    list(APPEND srcs "rmt/rmt_common.c" "rmt/rmt_encoder.c" "rmt/rmt_rx.c" "rmt/rmt_tx.c" "deprecated/rmt_legacy.c")
endif()

if(CONFIG_SOC_PCNT_SUPPORTED)
    list(APPEND srcs "pulse_cnt.c" "deprecated/pcnt_legacy.c")
endif()

if(CONFIG_SOC_SDMMC_HOST_SUPPORTED)
    list(APPEND srcs "sdmmc_transaction.c" "sdmmc_host.c")
endif()

if(CONFIG_SOC_I2S_SUPPORTED)
    list(APPEND srcs "i2s/i2s_common.c"
                     "i2s/i2s_std.c"
                     "deprecated/i2s_legacy.c")
    if(CONFIG_SOC_I2S_SUPPORTS_PDM)
        list(APPEND srcs "i2s/i2s_pdm.c")
    endif()
    if(CONFIG_SOC_I2S_SUPPORTS_TDM)
        list(APPEND srcs "i2s/i2s_tdm.c")
    endif()
endif()

if(CONFIG_SOC_TEMP_SENSOR_SUPPORTED)
    list(APPEND srcs "temperature_sensor.c"
                     "deprecated/rtc_temperature_legacy.c")
endif()

if(CONFIG_SOC_TWAI_SUPPORTED)
    list(APPEND srcs "twai.c")
endif()

if(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED)
    list(APPEND srcs "usb_serial_jtag.c")
endif()

if(CONFIG_SOC_SPI_SUPPORT_SLAVE_HD_VER2)
    list(APPEND srcs "spi/spi_slave_hd.c")
endif()

if(CONFIG_SOC_TOUCH_SENSOR_SUPPORTED)
    list(APPEND srcs "touch_sensor_common.c" "${target}/touch_sensor.c")
endif()

if(CONFIG_SOC_SDIO_SLAVE_SUPPORTED)
    list(APPEND srcs "sdio_slave.c")
endif()

if(${target} STREQUAL "esp32")
    list(APPEND srcs "dac_common.c"
                     "deprecated/adc_i2s_deprecated.c"
                     "esp32/dac.c")
endif()

if(${target} STREQUAL "esp32s2")
    list(APPEND srcs "dac_common.c"
                     "esp32s2/dac.c")
endif()

if(BOOTLOADER_BUILD)
    # Bootloader shall NOT depend on the drivers
    idf_component_register()
else()
    # (REQUIRES cannot hide soc headers, since many arguments in the driver headers are chip-dependent)
    # (Legacy drivers requires `esp_adc`, due to ADC HW resource mutex logics are there.
    # Can be removed together with legacy drivers)
    idf_component_register(SRCS "${srcs}"
                        INCLUDE_DIRS ${includes}
                        PRIV_INCLUDE_DIRS "include/driver"
                        PRIV_REQUIRES efuse
                        REQUIRES ringbuf kernel soc esp_hal esp_hw_support
                        LDFRAGMENTS linker.lf adc/adc.lf)
endif()
