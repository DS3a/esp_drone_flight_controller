#include "tfmini_lidar.hpp"
#include <stdio.h>

namespace TFMiniLidar {

    TFMiniLidar::TFMiniLidar() {
#ifdef DEBUG
        printf("initializing serial comms with the TFMini-Lidar\n");
#endif  

        // check these settings here https://cdn.sparkfun.com/assets/8/a/f/a/c/16977-TFMini-S_-_Micro_LiDAR_Module-Product_Manual.pdf page 11
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
            .rx_flow_ctrl_thresh = 122,
        };

        // Configure UART parameters
        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

        // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD2, RXD2, 18, 19));
        // Setup UART buffered IO with event queue
        const int uart_buffer_size = (1024 * 2);
        QueueHandle_t uart_queue;
        // Install UART driver using an event queue here
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                                uart_buffer_size, 10, &uart_queue, 0));

#ifdef DEBUG
        printf("serial comms initialized\n");
#endif 
    }

    void TFMiniLidar::read_lidar() {
        int length = uart_read_bytes(this->uart_num, this->data, 9, 100);
        this->distance = this->data[2] + this->data[3]*256;
    }


    double TFMiniLidar::get_distance() {
        return this->distance;
    }

    float TFMiniLidar::get_strength() {
        return this->strength;
    }

    float TFMiniLidar::get_temperature() {
        return this->temperature;
    }
}
