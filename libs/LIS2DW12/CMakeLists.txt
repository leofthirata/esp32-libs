set(LIS2DW12_srcs 
    src/lis2dw12_reg.c
    src/LIS2DW12Sensor.cpp
)

idf_component_register(SRCS ${LIS2DW12_srcs}
                      INCLUDE_DIRS "src"
                      REQUIRES arduino driver
                      )
#https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#controlling-component-compilation
#target_compile_options(${COMPONENT_LIB} PRIVATE -DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_DEBUG)
