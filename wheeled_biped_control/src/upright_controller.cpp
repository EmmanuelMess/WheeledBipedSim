#include "wheeled_biped_control/upright_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

namespace upright_controller {
UprightController::UprightController() {

}

controller_interface::InterfaceConfiguration UprightController::command_interface_configuration() const {

}

controller_interface::InterfaceConfiguration UprightController::state_interface_configuration() const {

}

controller_interface::CallbackReturn UprightController::on_init() {

}

controller_interface::CallbackReturn UprightController::on_configure(const rclcpp_lifecycle::State & previous_state) {
	// topics QoS
	auto subscribers_qos = rclcpp::SystemDefaultsQoS();
	subscribers_qos.keep_last(1);
	subscribers_qos.best_effort();

	// Reference Subscriber
	this->imuSubscriber = get_node()->create_subscription<sensor_msgs::msg::Imu>("/imu/head", subscribers_qos,
		[this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { this->imu_callback(msg); });
}

controller_interface::CallbackReturn UprightController::on_activate(const rclcpp_lifecycle::State & previous_state) {

}

// Chainable controller replaces update() with the following two functions
controller_interface::return_type UprightController::update_reference_from_subscribers(const rclcpp::Time & time,
	const rclcpp::Duration & period) {

}

controller_interface::return_type UprightController::update_and_write_commands(const rclcpp::Time & time,
	 const rclcpp::Duration & period) {

}


controller_interface::CallbackReturn UprightController::on_deactivate(const rclcpp_lifecycle::State & previous_state) {


}

controller_interface::CallbackReturn UprightController::on_cleanup(const rclcpp_lifecycle::State & previous_state) {

}

controller_interface::CallbackReturn UprightController::on_error(const rclcpp_lifecycle::State & previous_state) {


}

void UprightController::imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg) const {
	RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "" << msg);
}

}


#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(upright_controller::UprightController, controller_interface::ChainableControllerInterface)