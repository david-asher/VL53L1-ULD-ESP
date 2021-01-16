set(COMPONENT_PRIV_REQUIRES log)

set(COMPONENT_SRCS 
  "core/VL53L1X_api.c"
  "core/VL53L1X_calibration.c"
  "platform/vl53l1_platform.c"
  "platform/i2c_platform_esp.c"
  "platform/gpio_platform_esp.c"
)

set(COMPONENT_ADD_INCLUDEDIRS "." "core" "platform")

register_component()
