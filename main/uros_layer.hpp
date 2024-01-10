// #ifndef UROS_LAYER_H_
// #define UROS_LAYER_H_


// #include <uros_network_interfaces.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <std_msgs/msg/int32.h>
// #include <nav_msgs/msg/odometry.h>
// #include <std_msgs/msg/int32_multi_array.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <drone_controller_messages/msg/pwm_message.h>
// #include <drone_controller_messages/msg/attitude_setpoint.h>


// namespace UROSLayer {
//     rcl_publisher_t publisher;
//     rcl_subscription_t pwm_subscription;

//     std_msgs__msg__Int32 msg;
//     drone_controller_messages__msg__PwmMessage pwm_recv_msg;
//     drone_controller_messages__msg__AttitudeSetpoint attitude_recv_msg;
//     std_msgs__msg__Int32 recv_int_msg;


//     void pwm_subscription_callback(const void * msgin) {
//     #ifdef DEBUG
//         static int num_calls = 0;
//         num_calls++;
//         printf("[%d: %ld] received pwm values\n", num_calls, *last_value_time);
//     #endif

//         motor_pwm_value_mutex.lock();

//         const drone_controller_messages__msg__PwmMessage *pwm_values_msg = (const drone_controller_messages__msg__PwmMessage *)msgin;
//         motor_pwm_values->front_left = pwm_values_msg->front_left;
//         motor_pwm_values->front_right = pwm_values_msg->front_right;
//         motor_pwm_values->back_right = pwm_values_msg->back_right;
//         motor_pwm_values->back_left = pwm_values_msg->back_left;
//         *last_value_time = esp_timer_get_time() / 1000;

//         printf("fl: %ld\t fr: %ld\t br: %ld\t bl: %ld\n",
//             motor_pwm_values->front_left,
//             motor_pwm_values->front_right,
//             motor_pwm_values->back_right,
//             motor_pwm_values->back_left
//         );
//         motor_pwm_value_mutex.unlock();
//     }


//     void pwm_subscription_callback(const void * msgin) {
//     #ifdef DEBUG
//         static int num_calls = 0;
//         num_calls++;
//         printf("[%d: %ld] received pwm values\n", num_calls, *last_value_time);
//     #endif

//         motor_pwm_value_mutex.lock();

//         const drone_controller_messages__msg__PwmMessage *pwm_values_msg = (const drone_controller_messages__msg__PwmMessage *)msgin;
//         motor_pwm_values->front_left = pwm_values_msg->front_left;
//         motor_pwm_values->front_right = pwm_values_msg->front_right;
//         motor_pwm_values->back_right = pwm_values_msg->back_right;
//         motor_pwm_values->back_left = pwm_values_msg->back_left;
//         *last_value_time = esp_timer_get_time() / 1000;

//         printf("fl: %ld\t fr: %ld\t br: %ld\t bl: %ld\n",
//             motor_pwm_values->front_left,
//             motor_pwm_values->front_right,
//             motor_pwm_values->back_right,
//             motor_pwm_values->back_left
//         );
//         motor_pwm_value_mutex.unlock();
//     }



// }

// #endif // UROS_LAYER_H_
