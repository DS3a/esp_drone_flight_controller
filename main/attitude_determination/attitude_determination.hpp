#ifndef ATTITUDE_DETERMINATION_H 
#define ATTITUDE_DETERMINATION_H

#include "../drone_hardware_layer/drone_hardware_layer.hpp"
#include <memory>

namespace AttitudeDetermination {
    class AttitudeDetermination {
    private:
        std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors_; 
    public:
        AttitudeDetermination();
        void set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors);
    };
}

#endif