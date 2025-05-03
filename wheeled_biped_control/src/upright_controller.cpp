#include "wheeled_biped_control/upright_controller.hpp"

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
}


#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(upright_controller::UprightController, controller_interface::ChainableControllerInterface)