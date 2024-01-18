idf_component_register(SRCS "main.cpp"
"drone_hardware_layer/drone_hardware_layer.cpp" 
"drone_hardware_layer/tfmini_lidar.cpp"
"drone_hardware_layer/mpu6050_abs.cpp"
"attitude_determination/filters/gyro/gyro_highpassFilter.cpp"
"attitude_determination/filters/accel/accel_lowpassFilter.cpp"
"attitude_determination/attitude_determination.cpp"
                    INCLUDE_DIRS "." "./include")


target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23)
