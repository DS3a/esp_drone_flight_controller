#include "mpu6050_abs.hpp"

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

        i2c_bus_init();
        this->mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
        TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

        ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = mpu6050_wake_up(mpu6050);
        TEST_ASSERT_EQUAL(ESP_OK, ret);        
    }

    MPU6050Abs::MPU6050Abs() {
        this->i2c_bus_init();
        this->i2c_sensor_mpu6050_init();
    }

    
}