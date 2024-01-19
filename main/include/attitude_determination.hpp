#ifndef ATTITUDE_DETERMINATION_H 
#define ATTITUDE_DETERMINATION_H

#include "drone_hardware_layer.hpp"
#include "gyro_highpassFilter.h"
#include "accel_lowpassFilter.h"
// #include "filters/gyro/gyro_highpassFilter.c"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>

namespace AttitudeDetermination {
    class AttitudeDetermination {
    private:
        float Kp = 0.95;
        float Ki = 1.1;
        float mean_error = 10.0;
        bool calculated_orientation = false;
        Eigen::Quaternion<float> orientation;
        Eigen::Quaternion<float> velocity_quat;
        Eigen::Vector3f gyro_values;
        Eigen::Vector3f gyro_filtered_values;
        gyro_highpassFilter gyro_filter[3];
        
        Eigen::Vector3f g_direction;
        // orientation errors for the mahony filter
        
        Eigen::Vector3f e;
        Eigen::Vector3f e_i;

        bool initialized=false;
        
        accel_lowpassFilter accel_filter[3];
        Eigen::Vector3f accel_values;
        Eigen::Vector3f accel_filtered_values;
        std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors_; 
    public:
        AttitudeDetermination();
        void set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors);

        void read_gyro_filtered(Eigen::Vector3f *const gyro_filtered);
        void read_accel_filtered(Eigen::Vector3f *const accel_filtered);
    
        const Eigen::Quaternion<float> * get_orientation();
        void update_orientation(double dT);

    };
}

#endif