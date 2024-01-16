#ifndef DRONE_HARDWARE_LAYER_H_
#define DRONE_HARDWARE_LAYER_H_

#include <stdint.h>
#include "tfmini_lidar.hpp"
#include "mpu6050_abs.hpp"
#include <memory>

namespace DroneHardwareLayer {
    enum DroneState {IDLE, TAKING_OF, IN_FLIGHT, LANDING};

    class Motor {
    private:
        uint16_t pin_num_;

    public:
        Motor(uint16_t pin_num);
        void set_value(uint32_t pwm_value);
    };

    class DroneSensors {
    private:
        std::shared_ptr<MPU6050Abs::MPU6050Abs> imu;
        std::shared_ptr<TFMiniLidar::TFMiniLidar> lidar_ranger;        
    public:
        DroneSensors();
        std::shared_ptr<MPU6050Abs::MPU6050Abs> get_imu();
        double read_lidar_distance();
    };
}


#endif // DRONE_HARDWARE_LAYER_H_
