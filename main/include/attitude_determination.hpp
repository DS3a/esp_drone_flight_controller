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
        double Kp = 0.95;
        double Ki = 0.1;
        double Kd = 0.00;
        double mean_error = 10.0;
        bool calculated_orientation = false;
        Eigen::Quaternion<double> orientation;
        Eigen::Quaternion<double> velocity_quat;
        Eigen::Vector3d gyro_values;
        Eigen::Vector3d gyro_filtered_values;
        gyro_highpassFilter gyro_filter[3];
        
        Eigen::Vector3d g_direction;
        // orientation errors for the mahony filter
        
        Eigen::Vector3d e_old;
        Eigen::Vector3d e_d;
        Eigen::Vector3d e;
        Eigen::Vector3d e_i;

        bool initialized=false;
        
        accel_lowpassFilter accel_filter[3];
        Eigen::Vector3d accel_values;
        Eigen::Vector3d accel_filtered_values;
        std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors_; 
    public:
        AttitudeDetermination();
        void set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors);

        void read_gyro_filtered(Eigen::Vector3d *const gyro_filtered);
        void read_accel_filtered(Eigen::Vector3d *const accel_filtered);
    
        const Eigen::Quaternion<double> * get_orientation();
        void update_orientation(double dT);

    };
}

#endif