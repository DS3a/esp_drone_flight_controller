#ifndef ATTITUDE_DETERMINATION_H 
#define ATTITUDE_DETERMINATION_H

#include "../drone_hardware_layer/drone_hardware_layer.hpp"
#include <eigen3/Eigen/Eigen>
#include <memory>

namespace AttitudeDetermination {
    class AttitudeDetermination {
    private:
        std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors_; 
    public:
        AttitudeDetermination();
        void set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors);

        void read_gyro_filtered(Eigen::Vector3d *const gyro_filtered);
        void read_accel_filtered(Eigen::Vector3d *const accel_filtered);
    };
}

#endif