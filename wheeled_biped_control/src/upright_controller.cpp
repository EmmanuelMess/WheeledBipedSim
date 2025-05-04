#include "wheeled_biped_control/upright_controller.hpp"

#include <numeric>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace upright_controller {
UprightController::UprightController() : errorSum(0.0) {

}

controller_interface::InterfaceConfiguration UprightController::command_interface_configuration() const {
	controller_interface::InterfaceConfiguration command_interfaces_config;
	command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
	return command_interfaces_config;
}

controller_interface::InterfaceConfiguration UprightController::state_interface_configuration() const {
	controller_interface::InterfaceConfiguration state_interfaces_config;
	state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	state_interfaces_config.names = { "velocity" };
	return state_interfaces_config;
}

controller_interface::CallbackReturn UprightController::on_init() {
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_configure(const rclcpp_lifecycle::State & previous_state) {
	this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_node()->get_clock());
	this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);

	auto subscribers_qos = rclcpp::SystemDefaultsQoS();
	subscribers_qos.keep_last(1);
	subscribers_qos.best_effort();

	this->imuSubscriber = this->get_node()->create_subscription<sensor_msgs::msg::Imu>("/imu/head", subscribers_qos,
		[this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { this->imu_callback(msg); });

	this->diffTwistPublisher = this->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
	this->anglePublisher = this->get_node()->create_publisher<std_msgs::msg::Float64>("/debug/angle", 10);
	this->pidPublisher = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/debug/pid", 10);

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_activate(const rclcpp_lifecycle::State & previous_state) {
	return controller_interface::CallbackReturn::SUCCESS;
}

// Chainable controller replaces update() with the following two functions
controller_interface::return_type UprightController::update_reference_from_subscribers(const rclcpp::Time & time,
	const rclcpp::Duration & period) {
	return controller_interface::return_type::OK;
}

controller_interface::return_type UprightController::update_and_write_commands(const rclcpp::Time & time,
	 const rclcpp::Duration & period) {
	return controller_interface::return_type::OK;
}


controller_interface::CallbackReturn UprightController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_cleanup(const rclcpp_lifecycle::State & previous_state) {
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_error(const rclcpp_lifecycle::State & previous_state) {
	return controller_interface::CallbackReturn::SUCCESS;
}

void UprightController::imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
	// TODO add rolling window average to the IMU data

	const std::string& baselinkFrame = "base_link";
	const std::string& imuFrame = "imu_link";

	geometry_msgs::msg::TransformStamped t;
	try {
		t = tfBuffer->lookupTransform(baselinkFrame, imuFrame, tf2::TimePointZero);
	} catch (const tf2::TransformException& ex) {
		RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), "Could not transform "
					<< imuFrame << " to " << baselinkFrame << ":" << ex.what());
		return;
	}

	tf2::Quaternion orientation;
	tf2::convert(msg->orientation, orientation);

	const double deltaTime = 1.0 / 400.0;
	const double targetAngle = 0.0;

	// Get the angle with the z direction,
	const auto angle = orientation.y();

	const auto error = angle - targetAngle;

	this->errorSum = std::clamp(error * deltaTime + errorSum, -1.0, 1.0);

	const auto p = 31.0 * error;
	const auto i = 22.0 * errorSum;
	const auto d = 0.075 * (angle - lastAngle) / deltaTime;
	const auto pid = p + i + d;

	lastAngle = angle;

	{
		// TODO actually use the states
	    auto message = geometry_msgs::msg::TwistStamped();
	    message.header.stamp = this->get_node()->get_clock()->now();
	    message.header.frame_id = baselinkFrame;
	    message.twist.linear.x = pid;
	    message.twist.linear.y = 0.0;
	    message.twist.linear.z = 0.0;
	    message.twist.angular.x = 0.0;
	    message.twist.angular.y = 0.0;
	    message.twist.angular.z = 0.0;

	    this->diffTwistPublisher->publish(message);
	}
	RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "Angle: " << angle << " Error: " << error << " PID: " << pid << " P: " << p
																 << " I: " << i << " D: " << d);

    {
    	auto message = std_msgs::msg::Float64();
    	message.data = angle;

    	this->anglePublisher->publish(message);
    }

    {
    	auto message = std_msgs::msg::Float64MultiArray();
    	message.data = { p, i, d };

    	this->pidPublisher->publish(message);
    }
}

}


#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(upright_controller::UprightController, controller_interface::ChainableControllerInterface)