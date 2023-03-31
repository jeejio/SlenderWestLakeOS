idf_build_get_property(jeejio_path  JEEJIO_PATH)

set(src  "pir/tal_pir.c"
     "motor/tal_motor.c"
     "rotary_encoder/tal_rotary_encoder.c"
     "steering_gear/tal_steering_gear.c"
     "magnetic/tal_magnetic.c"
     "water_pump/tal_waterpump.c"
     "six_axis/tal_six_axis.c"
     "rfid/tal_rfid.c"
     "hyetometer/tal_hyetometer.c"
     "button_panel/tal_button_panel.c"
     "led_array/tal_led_array.c"
     "getsture/tal_getsture.c"
     "spo2/tal_spo2.c"
     "scanner/tal_scanner.c"
      "joystick/tal_joystick.c"
      "environment/tal_environment.c"
      "display/tal_display.c"
      "single_switch/tal_single_switch.c"
     )

set(includes  "pir"
     "motor"
     "rotary_encoder"
     "steering_gear"
     "magnetic"
     "water_pump"
     "six_axis"
     "rfid"
     "hyetometer"
     "button_panel"
     "led_array"
     "getsture"
     "spo2"
     "scanner"
     "joystick"
     "environment"
     "display"
     "single_switch"
     "${jeejio_path}/driver/jeejioxie_esp32c3/board/edu_lan/pir"
     "${jeejio_path}/driver/jeejioxie_esp32c3/board/edu_lan/motor"
     "${jeejio_path}/driver/jeejioxie_esp32c3/board/edu_lan/magnetic"
     "${jeejio_path}/driver/jeejioxie_esp32c3/board/edu_lan/display"
     )
idf_component_register(SRCS  ${src}
    INCLUDE_DIRS  ${includes}
    PRIV_REQUIRES hal kernel)

