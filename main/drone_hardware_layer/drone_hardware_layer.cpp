#include "drone_hardware_layer.hpp"

#include <stdio.h>

namespace DroneHardwareLayer {

    Motor::Motor(uint16_t pin_num) {
        printf("initialized motor\n");
        this->pin_num_ = pin_num;
    }

    void Motor::set_value(uint32_t pwm_value) {
        // TODO implement this
    }

    DroneSensors::DroneSensors() {
        printf("initializing the TFMini-Lidar\n");
        this->lidar_ranger = std::make_shared<TFMiniLidar::TFMiniLidar>();
    }

    double DroneSensors::read_lidar_distance() {
        this->lidar_ranger->read_lidar();
        return this->lidar_ranger->get_distance();        
    }

}
