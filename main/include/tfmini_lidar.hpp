#ifndef TFMINI_LIDAR_H_
#define TFMINI_LIDAR_H_

#include <driver/uart.h>
#include <stdint.h>

#define RXD2 16
#define TXD2 17

namespace TFMiniLidar {
    class TFMiniLidar {
    private:
        const uart_port_t uart_num = UART_NUM_2;
        uint8_t data[9];
        double distance;
        float strength;
        float temperature;
        const int header = 89; 
    public:
        TFMiniLidar();

        void read_lidar();
        double get_distance();
        float get_strength();
        float get_temperature();
    };
}

#endif // TFMINI_LIDAR_H_
