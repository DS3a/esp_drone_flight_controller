#ifndef MPU6050_ABST_H
#define MPU6050_ABST_H


#include <driver/i2c.h>
#include <mpu6050.h>
#include "esp_system.h"
#include "unity.h"
#include <stdint.h>
#include <stdio.h>

#include "mpu6050_registers.hpp"
#include <eigen3/Eigen/Eigen>

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define CALIBRATION_BUFFER 250


namespace MPU6050Abs {
    class MPU6050Abs {
    private:
        i2c_config_t conf;
        mpu6050_handle_t mpu6050 = NULL;

    public:
        MPU6050Abs();
        void i2c_bus_init();
        void i2c_sensor_mpu6050_init();
        uint8_t read_accel_values(Eigen::Vector3d *const accel);
        uint8_t read_gyro_values(Eigen::Vector3d *const gyro);

        uint8_t mean_gyro(Eigen::Vector3d *const gyro_buf);
        uint8_t calibrate_gyro();
    
    };
}

#endif // MPU6050_ABST_H