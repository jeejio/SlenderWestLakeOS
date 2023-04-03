idf_component_register(SRCS
                        "wifi/net_socket.c"
                        "MQTTClient-RT/paho_mqtt_pipe.c"

                        "MQTTPacket/MQTTConnectClient.c"
                        "MQTTPacket/MQTTConnectServer.c"
                        "MQTTPacket/MQTTDeserializePublish.c"
                        "MQTTPacket/MQTTFormat.c"
                        "MQTTPacket/MQTTPacket.c"
                        "MQTTPacket/MQTTSerializePublish.c"
                        "MQTTPacket/MQTTSubscribeClient.c"
                        "MQTTPacket/MQTTSubscribeServer.c"
                        "MQTTPacket/MQTTUnsubscribeClient.c"
                        "MQTTPacket/MQTTUnsubscribeServer.c"

                        "json/cJSON.c"

                        "smart_config/smt_connect.c"

                        "mson/mson.c"

                        "webclient/webclient.c"
                        


                    INCLUDE_DIRS "." "${jeejio_path}/kernel/include/freertos"
                        "${jeejio_path}/hal/include" "${jeejio_path}/driver/jeejioxie_esp32c3/chip/driver/include"  
                        "${jeejio_path}/external/wifi"
                        "${jeejio_path}/external/MQTTClient-RT"
                        "${jeejio_path}/external/MQTTPacket"
                        "${jeejio_path}/external/json"
			            "${jeejio_path}/external/smart_config"
                        "${jeejio_path}/external/mson"
                        "${jeejio_path}/external/webclient"

                    REQUIRES driver
                    REQUIRES kernel esp_wifi)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
