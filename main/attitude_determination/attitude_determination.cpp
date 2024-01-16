#include "attitude_determination.hpp"

namespace AttitudeDetermination {
    AttitudeDetermination::AttitudeDetermination() {

    }

    void AttitudeDetermination::set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors) {
        this->drone_sensors_ = drone_sensors;
    }
}