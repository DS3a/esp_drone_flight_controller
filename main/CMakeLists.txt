idf_component_register(SRCS "main.cpp"
"uros_layer.cpp"
"drone_hardware_layer/drone_hardware_layer.cpp" 
"drone_hardware_layer/tfmini_lidar.cpp"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23)
