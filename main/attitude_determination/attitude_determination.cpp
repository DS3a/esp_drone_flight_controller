#include "attitude_determination.hpp"

#define MU 0.99

#define MAHONY

namespace AttitudeDetermination {
    AttitudeDetermination::AttitudeDetermination() {
        gyro_highpassFilter_init(&(this->gyro_filter[0])); // for x axis
        gyro_highpassFilter_init(&(this->gyro_filter[1])); // for y axis
        gyro_highpassFilter_init(&(this->gyro_filter[2])); // for z axis

        accel_lowpassFilter_init(&(this->accel_filter[0]));
        accel_lowpassFilter_init(&(this->accel_filter[1]));
        accel_lowpassFilter_init(&(this->accel_filter[2]));

        this->orientation.w() = 1;
        this->orientation.x() = 0;
        this->orientation.y() = 0;
        this->orientation.z() = 0;   
    }

    void AttitudeDetermination::set_drone_sensors(std::shared_ptr<DroneHardwareLayer::DroneSensors> drone_sensors) {
        this->drone_sensors_ = drone_sensors;
   }

    void AttitudeDetermination::read_accel_filtered(Eigen::Vector3d *const accel_filtered) {
        this->drone_sensors_->get_imu()->read_accel_values(&(this->accel_values));

        accel_lowpassFilter_put(&this->accel_filter[0], accel_values.x());
        accel_lowpassFilter_put(&this->accel_filter[1], accel_values.y());
        accel_lowpassFilter_put(&this->accel_filter[2], accel_values.z());
    
        accel_filtered->x() = accel_lowpassFilter_get(&this->accel_filter[0]);
        accel_filtered->y() = accel_lowpassFilter_get(&this->accel_filter[1]);
        accel_filtered->z() = accel_lowpassFilter_get(&this->accel_filter[2]);
    }

    void AttitudeDetermination::read_gyro_filtered(Eigen::Vector3d *const gyro_filtered) {
        this->drone_sensors_->get_imu()->read_gyro_values(&gyro_values);
        gyro_highpassFilter_put(&(this->gyro_filter[0]), gyro_values.x());
        gyro_highpassFilter_put(&(this->gyro_filter[1]), gyro_values.y());
        gyro_highpassFilter_put(&(this->gyro_filter[2]), gyro_values.z());

        gyro_filtered->x() = gyro_highpassFilter_get(&(this->gyro_filter[0]));
        gyro_filtered->y() = gyro_highpassFilter_get(&(this->gyro_filter[1]));
        gyro_filtered->z() = gyro_highpassFilter_get(&(this->gyro_filter[2]));

        // if (!initialized) {
        //     gyro_filtered->x() = gyro_values.x();
        //     gyro_filtered->y() = gyro_values.y();
        //     gyro_filtered->z() = gyro_values.z();
        // } else {
        //     gyro_filtered->x() = MU * gyro_filtered->x() + (1 - MU) * gyro_values.x();
        //     gyro_filtered->y() = MU * gyro_filtered->y() + (1 - MU) * gyro_values.y();
        //     gyro_filtered->z() = MU * gyro_filtered->z() + (1 - MU) * gyro_values.z();
        // } 
    }

    void AttitudeDetermination::update_orientation(double dT=0.001) {
        // this->read_gyro_filtered(&(this->gyro_filtered_values));

        this->drone_sensors_->get_imu()->read_gyro_values(&(this->gyro_filtered_values));

        this->gyro_filtered_values *= this->mean_error; // mean error
        // this->gyro_filtered_values = Eigen::Vector3d(0, 0, 0);

        if (calculated_orientation) {
            // update orientation error using accelerometer measurements

#ifdef MAHONY
            // this->read_accel_filtered(&this->accel_filtered_values);
            this->drone_sensors_->get_imu()->read_accel_values(&(this->accel_filtered_values));

            printf("x %f\ty: %f\tz: %f\n", accel_filtered_values.x(), accel_filtered_values.y(), accel_filtered_values.z());
            g_direction.x() = 2 * (orientation.x()*orientation.z() - orientation.w()*orientation.y());
            g_direction.y() = 2 * (orientation.w()*orientation.x() + orientation.y()*orientation.z());
            g_direction.z() = orientation.w()*orientation.w() -
                            orientation.x() * orientation.x() -
                            orientation.y() * orientation.y() +
                            orientation.z() * orientation.z();

            this->e = this->accel_filtered_values.cross(this->g_direction);
            this->e_i += this->e * dT;
            if (this->e_i.norm() >= 2) {
                // integral windup
                this->e_i = Eigen::Vector3d(0, 0, 0);
            }
            this->e_d = (this->e_old - this->e) / dT;

            this->gyro_filtered_values += this->Kp * this->e + this->Ki * this->e_i + this->Kd * this->e_d;
            this->e_old = this->e;
#endif

#ifdef MADGWICK
        this->orientation.conjugate();
#endif

        }

        this->velocity_quat.w() = 0;
        this->velocity_quat.x() = this->gyro_filtered_values.x();
        this->velocity_quat.y() =  this->gyro_filtered_values.y();
        this->velocity_quat.z() = this->gyro_filtered_values.z();

        this->velocity_quat.coeffs() *= 0.5;
        this->velocity_quat = (this->velocity_quat * this->orientation);
        this->velocity_quat.coeffs() *= dT;
        this->orientation.coeffs() += this->velocity_quat.coeffs();
        this->orientation.coeffs().normalize();

        if (!calculated_orientation) {
            calculated_orientation = true;
            this->e = Eigen::Vector3d(0, 0, 0);
            this->e_i = Eigen::Vector3d(0, 0, 0);
        }
    }

    const Eigen::Quaternion<double> * AttitudeDetermination::get_orientation() {
        return &(this->orientation);
    }
}