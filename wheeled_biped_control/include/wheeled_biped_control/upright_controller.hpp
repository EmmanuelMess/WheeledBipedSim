#ifndef UPRIGHT_CONTROLLER_HPP
#define UPRIGHT_CONTROLLER_HPP

#include "controller_interface/chainable_controller_interface.hpp"

namespace upright_controller {
class UprightController : public controller_interface::ChainableControllerInterface {
public:
	UprightController();

	controller_interface::InterfaceConfiguration command_interface_configuration() const override;

	controller_interface::InterfaceConfiguration state_interface_configuration() const override;

	controller_interface::CallbackReturn on_init() override;

	controller_interface::CallbackReturn on_configure(
	  const rclcpp_lifecycle::State & previous_state) override;

	controller_interface::CallbackReturn on_activate(
	  const rclcpp_lifecycle::State & previous_state) override;

	// Chainable controller replaces update() with the following two functions
	controller_interface::return_type update_reference_from_subscribers(
	  const rclcpp::Time & time, const rclcpp::Duration & period) override;

	controller_interface::return_type update_and_write_commands(
	  const rclcpp::Time & time, const rclcpp::Duration & period) override;

	controller_interface::CallbackReturn on_deactivate(
	  const rclcpp_lifecycle::State & previous_state) override;

	controller_interface::CallbackReturn on_cleanup(
	  const rclcpp_lifecycle::State & previous_state) override;

	controller_interface::CallbackReturn on_error(
	  const rclcpp_lifecycle::State & previous_state) override;
};
}

#endif //UPRIGHT_CONTROLLER_HPP
