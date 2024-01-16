#include "attitude_determination.hpp"

namespace AttitudeDetermination {
    AttitudeDetermination::AttitudeDetermination() {

    }

    void AttitudeDetermination::set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors) {
        this->drone_sensors_ = drone_sensors;
    }

    void AttitudeDetermination::read_accel_filtered(Eigen::Vector3d *const gyro_filtered) {

    }

    void AttitudeDetermination::read_gyro_filtered(Eigen::Vector3d *const accel_filtered) {

    }
}