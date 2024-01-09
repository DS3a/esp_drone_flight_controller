# _Flight Controller based on ESP32, MPU6050, Infrared Ranger(height), and Motion Capture_

This is the firmware to be flashed onto the Esp32
Before building this project, clone https://github.com/micro-ROS/micro_ros_espidf_component.git to the `components` directory (create it if non-existent).

After cloning that, go into the cloned folder and clone https://github.com/DS3a/drone_flight_controller_ros_msgs.git into `extra_packages` (this should exist, if it doesn't you're probably in the wrong directory)

## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
