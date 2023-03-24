idf_component_register(SRCS "main.c"
                        "edu_lan/EduLanMain.c"
                        "edu_lan/AppBloodOxygen.c"
                        "edu_lan/AppButtonPanel.c"
                        "edu_lan/AppCodeScanner.c"
                        "edu_lan/AppEnvSensor.c"
                        "edu_lan/AppGestureSensor.c"
                        "edu_lan/AppImuSensor.c"
                        "edu_lan/AppJoystick.c"
                        "edu_lan/AppLCD.c"
                        "edu_lan/AppLEDArray.c"
                        "edu_lan/AppMagneticField.c"
                        "edu_lan/AppMotor.c"
                        "edu_lan/AppPIR.c"
                        "edu_lan/AppRainSensor.c"
                        "edu_lan/AppRFID.c"
                        "edu_lan/AppRotaryEncoder.c"
                        "edu_lan/AppSteeringGear.c"
                        "edu_lan/AppWaterPump.c"

                    INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos"
                        "${jeejio_path}/application/edu_lan"
		                "${jeejio_path}/hal/include"
                        "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include"
                        "${jeejio_path}/external/wifi"
                        "${jeejio_path}/external/MQTTClient-RT"
                        "${jeejio_path}/external/MQTTPacket"
                        "${jeejio_path}/external/json"
                        "${jeejio_path}/external/smart_config"
                        "${jeejio_path}/external/mson"
                        "${jeejio_path}/external/webclient"
                        "${jeejio_path}/framework/maopao"
                        "${jeejio_path}/framework/factory_test"
                        "${jeejio_path}/framework/cloud"

                    REQUIRES tal driver fatfs vfs
                    REQUIRES kernel esp_wifi nvs_flash)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
set(image "${jeejio_path}/external/fatfs")
fatfs_create_spiflash_image(storage ${image} FLASH_IN_PROJECT PRESERVE_TIME)
jee_auto_init_list_register()

