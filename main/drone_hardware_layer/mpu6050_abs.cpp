#include "mpu6050_abs.hpp"
#include <math.h>


/*
 TODO set offset values

Sensor readings with offsets:	8	-10	16384	1	2	0
Your offsets:	-1589	-771	1343	106	-21	16

Data is printed as: acelX acelY acelZ giroX giroY giroZ
Check that your sensor readings are close to 0 0 16384 0 0 0

*/


namespace MPU6050Abs {

    void MPU6050Abs::i2c_bus_init() {
        this->conf.mode = I2C_MODE_MASTER;
        this->conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
        this->conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        this->conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
        this->conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        this->conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        this->conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

        esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

        ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");

    }

    void MPU6050Abs::i2c_sensor_mpu6050_init() {
        esp_err_t ret;

        accel_x_offset = -1589;
        accel_y_offset = -771;
        accel_z_offset = 1343;
        gyro_x_offset = 106;
        gyro_y_offset = -21;
        gyro_z_offset = 16;
        

        memcpy(accel_x_offset_buf, &accel_x_offset, sizeof(int));
        memcpy(accel_y_offset_buf, &accel_y_offset, sizeof(int));
        memcpy(accel_z_offset_buf, &accel_z_offset, sizeof(int));
        memcpy(gyro_x_offset_buf, &gyro_x_offset, sizeof(int));
        memcpy(gyro_y_offset_buf, &gyro_y_offset, sizeof(int));
        memcpy(gyro_z_offset_buf, &gyro_z_offset, sizeof(int));

        // i2c_bus_init();
        this->mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
        TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

        ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = mpu6050_wake_up(mpu6050);
        TEST_ASSERT_EQUAL(ESP_OK, ret);   

        mpu6050_write_ext(mpu6050, MPU6050_REG_ACCEL_XOFFS_H, &accel_x_offset_buf[1], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_ACCEL_YOFFS_H, &accel_y_offset_buf[1], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_ACCEL_ZOFFS_H, &accel_z_offset_buf[1], sizeof(uint8_t));

        mpu6050_write_ext(mpu6050, MPU6050_REG_ACCEL_XOFFS_L, &accel_x_offset_buf[0], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_ACCEL_YOFFS_L, &accel_y_offset_buf[0], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_ACCEL_ZOFFS_L, &accel_z_offset_buf[0], sizeof(uint8_t));

        mpu6050_write_ext(mpu6050, MPU6050_REG_GYRO_XOFFS_H, &gyro_x_offset_buf[1], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_GYRO_YOFFS_H, &gyro_y_offset_buf[1], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_GYRO_ZOFFS_H, &gyro_z_offset_buf[1], sizeof(uint8_t));

        mpu6050_write_ext(mpu6050, MPU6050_REG_GYRO_XOFFS_L, &gyro_x_offset_buf[0], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_GYRO_YOFFS_L, &gyro_y_offset_buf[0], sizeof(uint8_t));
        mpu6050_write_ext(mpu6050, MPU6050_REG_GYRO_ZOFFS_L, &gyro_z_offset_buf[0], sizeof(uint8_t));

    }

    MPU6050Abs::MPU6050Abs() {
        this->z_axis_270_rot << 
            0, -1, 0,
            1, 0, 0,
            0, 0, 1
        ;

        printf("initializing the i2c bus\n");
        this->i2c_bus_init();
        printf("done initializing the bus, initializing the sensor\n");
        this->i2c_sensor_mpu6050_init();
        printf("done initializing imu\n");
    }


    uint8_t MPU6050Abs::read_accel_values(Eigen::Vector3f *const accel) {
        esp_err_t ret;
        mpu6050_acce_value_t acce;

        ret = mpu6050_get_acce(this->mpu6050, &acce);
        if (ESP_OK != ret) {
            return 0;
        }

        accel->x() = acce.acce_x;
        accel->y() = acce.acce_y;
        accel->z() = acce.acce_z;

        *accel = this->z_axis_270_rot * (*accel);

        *accel *= 9.8; 
        return 1;
    }

    uint8_t MPU6050Abs::read_gyro_values(Eigen::Vector3f *const gyro) {
        esp_err_t ret;
        mpu6050_gyro_value_t gyro_reading;

        ret = mpu6050_get_gyro(mpu6050, &gyro_reading);
        if (ESP_OK != ret) {
            return 0;
        }

        gyro->x() = gyro_reading.gyro_x * M_PI / 180.0;
        gyro->y() = gyro_reading.gyro_y * M_PI / 180.0;
        gyro->z() = gyro_reading.gyro_z * M_PI / 180.0;


        *gyro = this->z_axis_270_rot * (*gyro);

        // gyro->x() = gyro_reading.gyro_x / 131.0;
        // gyro->y() = gyro_reading.gyro_y / 131.0;
        // gyro->z() = gyro_reading.gyro_z / 131.0;

        // returns in radian per second
        return 1;
    }


    uint8_t MPU6050Abs::mean_gyro(Eigen::Vector3f *const gyro_buf) {
        Eigen::Vector3f gyro_readings(0.0, 0.0, 0.0);

        int i=0;
        while(i++ < CALIBRATION_BUFFER + 100) {
#ifdef DEBUG
            printf("[%d] getting readings to calibrate\n", i);
#endif
            this->read_gyro_values(&gyro_readings);
            if (i < 100)
                continue; // skip the first 100 readings for vibes
            else
                (*gyro_buf) += gyro_readings;

            usleep(2);
        }

        (*gyro_buf) /= CALIBRATION_BUFFER;

        return 1;
    }

    uint8_t MPU6050Abs::calibrate_gyro() {
        // read mean values to get offset
        Eigen::Vector3f buffer_readings(0.0, 0.0, 0.0);
        this->mean_gyro(&buffer_readings);
        printf("the gyro offset is x: %f\t y: %f z: %f\n", buffer_readings.x(), buffer_readings.y(), buffer_readings.z());
        
        // mpu6050_write_ext(this->mpu6050, MPU6050_REG_GYRO_XOFFS_H, gx_offset_, 4);
        // mpu6050_write_ext(this->mpu6050, MPU6050_REG_GYRO_YOFFS_H, (uint8_t*)gy_offset, sizeof(int));
        // mpu6050_write_ext(this->mpu6050, MPU6050_REG_GYRO_ZOFFS_H, (uint8_t*)gz_offset, sizeof(int));

        buffer_readings.x() = 0;
        buffer_readings.y() = 0;
        buffer_readings.z() = 0;
        this->mean_gyro(&buffer_readings);
        printf("the gyro offset is x: %f\t y: %f z: %f \n", buffer_readings.x(), buffer_readings.y(), buffer_readings.z());
 

        return 1;
    }

}
