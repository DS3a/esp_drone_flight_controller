#define UPDATE_RATE_HZ 100
#define DRONE_NAME "drone_0"


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


#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <drone_hardware_layer.hpp>
#include <attitude_determination.hpp>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define DEBUG



rcl_publisher_t odom_publisher;
rcl_subscription_t attitude_setpoint_subscription;

std_msgs__msg__Int32 msg;
drone_controller_messages__msg__AttitudeSetpoint attitude_recv_msg;
nav_msgs__msg__Odometry odometry_dbg_msg;
std_msgs__msg__Int32 recv_int_msg;


const Eigen::Quaternion<float> *orientation;

typedef struct {
	uint32_t front_left;
	uint32_t front_right;
	uint32_t back_right;
	uint32_t back_left;
} motor_pwm_values_t;

std::shared_ptr<motor_pwm_values_t> motor_pwm_values;
std::shared_ptr<uint32_t> last_value_time;
std::mutex motor_pwm_value_mutex;
std::shared_ptr<DroneHardwareLayer::DroneSensors> sensors;

void attitude_subscription_callback(const void * msgin) {
#ifdef DEBUG
	static int num_calls = 0;
	num_calls++;
	printf("[%d: %ld] received attitude values\n", num_calls, *last_value_time);
#endif

	// motor_pwm_value_mutex.lock();

	// const drone_controller_messages__msg__PwmMessage *pwm_values_msg = (const drone_controller_messages__msg__PwmMessage *)msgin;
	const drone_controller_messages__msg__AttitudeSetpoint *attitude_setpoint_msg = (const drone_controller_messages__msg__AttitudeSetpoint *)msgin;
	// motor_pwm_values->front_left = pwm_values_msg->front_left;
	// motor_pwm_values->front_right = pwm_values_msg->front_right;
	// motor_pwm_values->back_right = pwm_values_msg->back_right;
	// motor_pwm_values->back_left = pwm_values_msg->back_left;
	*last_value_time = esp_timer_get_time() / 1000;

	printf("roll: %f\t pitch: %f\t yawrate: %f\t thrust: %f\n",
		attitude_setpoint_msg->roll,
		attitude_setpoint_msg->pitch,
		attitude_setpoint_msg->yawrate,
		attitude_setpoint_msg->thrust
	);
	// motor_pwm_value_mutex.unlock();
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);

	odometry_dbg_msg.child_frame_id.capacity = 8;
	odometry_dbg_msg.header.frame_id.capacity = 8;
	odometry_dbg_msg.child_frame_id.data = (char*) malloc(8 * sizeof(char));
	odometry_dbg_msg.header.frame_id.data = (char*) malloc(8 * sizeof(char));
	strcpy(odometry_dbg_msg.child_frame_id.data, "imu\0");
	strcpy(odometry_dbg_msg.header.frame_id.data, "odom\0");

	odometry_dbg_msg.pose.pose.orientation.w = orientation->w();
	odometry_dbg_msg.pose.pose.orientation.x = orientation->x();
	odometry_dbg_msg.pose.pose.orientation.y = orientation->y();
	odometry_dbg_msg.pose.pose.orientation.z = orientation->z();

	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&odom_publisher, &odometry_dbg_msg, NULL));
		msg.data++;
	}
}


void micro_ros_task(void * arg) {
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

	RCCHECK(rclc_publisher_init_default(
		&odom_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"/odom_drone"
	));

	rcl_timer_t timer;
	const unsigned int timer_timeout = 1;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));


	RCCHECK(rclc_subscription_init_default(
		&attitude_setpoint_subscription,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drone_controller_messages, msg, AttitudeSetpoint),
		"attitude_setpoint"));

// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

	// Add timer and subscriber to executor.
	printf("creating attitude subscription\n");
	RCCHECK(rclc_executor_add_subscription(&executor, &attitude_setpoint_subscription, &attitude_recv_msg, &attitude_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&attitude_setpoint_subscription, &node));
	RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void write_pwm_values(void * arg) {
	while(1) {

		/* TODO check drone stat
		 * only write pwm values to the motors if it is IN_FLIGHT
		 */
		motor_pwm_value_mutex.lock();
		uint32_t current_time = esp_timer_get_time() / 1000;
		if (current_time - (*last_value_time) >= 1000/UPDATE_RATE_HZ) { // if the pwm values aren't being given at 100Hz
			// TODO Initiate landing sequence
		} else {
			// TODO write pwm values to motor
		}

		motor_pwm_value_mutex.unlock();
		usleep(1);
	}

	vTaskDelete(NULL);
}

void attitude_determination_task(void * arg) {
	Eigen::Vector3f gyro_readings;
	bool gyro_calibrated = true;

	AttitudeDetermination::AttitudeDetermination ahrs;
	ahrs.set_drone_sensors(sensors);
	
	while(1) {
		double dist = sensors->read_lidar_distance();
#ifndef DEBUG
		printf("the lidar distance is %f\n", dist);
#endif

		if (!gyro_calibrated) {
			printf("calibrating the gyro\n");
			sensors->get_imu()->calibrate_gyro();
			gyro_calibrated = true;
			printf("done calibrating the gyro\n");
		}
		// sensors->get_imu()->read_gyro_values(&gyro_readings);
		// ahrs.read_gyro_filtered(&gyro_readings);
		ahrs.update_orientation(0.001);
		orientation = ahrs.get_orientation();
		// printf("the gyro readings\nx: %f\n y: %f\n z: %f\n", gyro_readings.x(), gyro_readings.y(), gyro_readings.z());
		// printf("%f, %f, %f\n", gyro_readings.x(), gyro_readings.y(), gyro_readings.z());

		vTaskDelay(1 /  portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void initialize_drone_hardware() {
	DroneHardwareLayer::Motor(12);
	printf("initializing the sensors\n");
	sensors = std::make_shared<DroneHardwareLayer::DroneSensors>();
}


extern "C" void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	motor_pwm_values = std::make_shared<motor_pwm_values_t>();
	last_value_time = std::make_shared<uint32_t>();

	initialize_drone_hardware();

	/* TODO create a task to write pwm values to the motors
	 * Before reading the motor values from `motor_pwm_values`, check the time when the last commands came in
	 * If the time is more than 1/pingrate then there is a connection error
	 * In this case use the IMUs magnetometer, gyro, and depth sensor to land
	 */

	// TODO task to read imu and height sensor and determine attitude
	xTaskCreate(attitude_determination_task,
			"attitude_determination_task",
			16000,
			NULL,
			5,
			NULL);

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            16000,
            NULL,
            5, // Low priority numbers denote low priority tasks
			// see https://www.freertos.org/RTOS-task-priority.html
            NULL);
}
