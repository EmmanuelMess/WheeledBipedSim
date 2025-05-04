#ifndef UPRIGHT_CONTROLLER_HPP
#define UPRIGHT_CONTROLLER_HPP

#include <memory>

#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/exceptions.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
private:
	double lastAngle;
	double errorSum;

	std::shared_ptr<tf2_ros::TransformListener> tfListener;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer;

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber;

	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr diffTwistPublisher;

	void imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
};
}

#endif //UPRIGHT_CONTROLLER_HPP
