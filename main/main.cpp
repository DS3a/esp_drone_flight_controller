#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <drone_controller_messages/msg/pwm_message.h>
#include <drone_controller_messages/msg/attitude_setpoint.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define DRONE_NAME "drone_0"

rcl_publisher_t publisher;
rcl_subscription_t pwm_subscription;

std_msgs__msg__Int32MultiArray recv_msg;
std_msgs__msg__Int32 msg;

drone_controller_messages__msg__PwmMessage pwm_recv_msg;
drone_controller_messages__msg__AttitudeSetpoint attitude_recv_msg;

rcl_subscription_t subscriber;
std_msgs__msg__Int32 recv_int_msg;

typedef struct {
	uint32_t front_left;
	uint32_t front_right;
	uint32_t back_right;
	uint32_t back_left;
} motor_pwm_values_t;


std::shared_ptr<motor_pwm_values_t> motor_pwm_values;
std::shared_ptr<uint32_t> last_value_time;
std::mutex motor_pwm_value_mutex;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		// printf("Publishing: %d\n", (int) msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n",  (int)  msg->data);
}

void pwm_subscription_callback(const void * msgin) {
	printf("received pwm values\n");
	motor_pwm_value_mutex.lock();
	// const std_msgs__msg__Int32MultiArray *pwm_values_msg = (const std_msgs__msg__Int32MultiArray *)msgin;

	const drone_controller_messages__msg__PwmMessage *pwm_values_msg = (const drone_controller_messages__msg__PwmMessage *)msgin;
	motor_pwm_values->front_left = pwm_values_msg->front_left;
	motor_pwm_values->front_right = pwm_values_msg->front_right;
	motor_pwm_values->back_right = pwm_values_msg->back_right;
	motor_pwm_values->back_left = pwm_values_msg->back_left;
	*last_value_time = esp_timer_get_time() / 1000;

	printf("fl: %ld\t fr: %ld\t br: %ld\t bl: %ld\n",
		motor_pwm_values->front_left,
		motor_pwm_values->front_right,
		motor_pwm_values->back_right,
		motor_pwm_values->back_left
	);
	// motor_pwm_value_mutex.unlock();
}

void micro_ros_task(void * arg)
{
    printf("starting the task\n");
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "drone_node", DRONE_NAME, &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"drone_status"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_sub"));

	RCCHECK(rclc_subscription_init_default(
		&pwm_subscription,
		&node,
		// rosidl_typesupport_c__get_message_type_support_handle__drone_controller_messages__msg__PwmMessage,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drone_controller_messages, msg, PwmMessage),
		// ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		"pwm_values"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 0500;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_int_msg, &subscription_callback, ON_NEW_DATA));
	printf("creating pwm subscription\n");

	recv_msg.data.capacity = 4;
	recv_msg.data.size = 0;
	recv_msg.data.data = (int32_t*) malloc(recv_msg.data.capacity * sizeof(int32_t));

	// RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscription, &recv_msg, &pwm_subscription_callback, ON_NEW_DATA));

	msg.data = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
		usleep(1000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&pwm_subscription, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	// TODO create a task to write pwm values

	// TODO task to read imu and height sensor

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            16000,
            NULL,
            5,
            NULL);
}