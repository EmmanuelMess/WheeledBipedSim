#include "wheeled_biped_control/upright_controller.hpp"

#include <numeric>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace upright_controller {
UprightController::UprightController()
	: isActive(false)
	, lastAngle(0.0)
	, errorSum(0.0)
	, imuRealtimeBuffer(nullptr)
	, speedRealtimeBuffer(nullptr) {

}

controller_interface::InterfaceConfiguration UprightController::command_interface_configuration() const {
	std::vector<std::string> names;
	// TODO this is wrong
	//for (const auto & wheelName : params.wheel_names) {
	//	names.push_back(wheelName + "/" + hardware_interface::HW_IF_VELOCITY);
	//}
	return controller_interface::InterfaceConfiguration {
		controller_interface::interface_configuration_type::INDIVIDUAL,
		names
 	};
}

controller_interface::InterfaceConfiguration UprightController::state_interface_configuration() const {
	std::vector<std::string> names;
	//for (const auto & wheelName : params.wheel_names) {
	//	names.push_back(wheelName + "/" + hardware_interface::HW_IF_VELOCITY);
	//}
	//TODO add the IMU angle as a state
	return controller_interface::InterfaceConfiguration {
		controller_interface::interface_configuration_type::INDIVIDUAL,
		names
 	};
}

controller_interface::CallbackReturn UprightController::on_init() {
	const auto paramListener = std::make_shared<ParamListener>(this->get_node());
	this->params = paramListener->get_params();
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_configure(const rclcpp_lifecycle::State & /* previous_state */) {
	// Allocate reference interfaces if needed
	reference_interfaces_.resize(UprightController::INTERAFACE_SIZE, std::numeric_limits<double>::quiet_NaN());

	this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_node()->get_clock());
	this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);

	this->imuSubscriber = this->get_node()->create_subscription<sensor_msgs::msg::Imu>("/imu/head", rclcpp::SystemDefaultsQoS(),
		[this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
			if (!this->isActive) {
				return;
			}

			// TODO check if we need a stale message check
			this->imuRealtimeBuffer.writeFromNonRT(msg);
        });
	this->speedSubscriber = this->get_node()->create_subscription<geometry_msgs::msg::TwistStamped>("/speed", rclcpp::SystemDefaultsQoS(),
		[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
			if (!this->isActive) {
				RCLCPP_WARN(get_node()->get_logger(), "Can't accept new messages. Controller is inactive");
				return;
			}

			// TODO check if we need a stale message check
			this->speedRealtimeBuffer.writeFromNonRT(msg);
		});

	this->diffTwistPublisher = this->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", rclcpp::SystemDefaultsQoS());
	this->diffTwistRealtimePublisher = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(this->diffTwistPublisher);
	this->anglePublisher = this->get_node()->create_publisher<std_msgs::msg::Float64>("/debug/angle", rclcpp::SystemDefaultsQoS());
	this->angleRealtimePublisher = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(this->anglePublisher);
	this->pidPublisher = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/debug/pid", rclcpp::SystemDefaultsQoS());
	this->pidRealtimePublisher = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(this->pidPublisher);

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_activate(const rclcpp_lifecycle::State & /* previous_state */) {
	isActive = true;
	return controller_interface::CallbackReturn::SUCCESS;
}

// Chainable controller replaces update() with the following two functions
controller_interface::return_type UprightController::update_reference_from_subscribers(const rclcpp::Time & /* time */,
	const rclcpp::Duration & /* period */) {
	const sensor_msgs::msg::Imu::ConstSharedPtr imuMessage = *(imuRealtimeBuffer.readFromRT());
	const geometry_msgs::msg::TwistStamped::ConstSharedPtr speedMessage = *(speedRealtimeBuffer.readFromRT());

	if (imuMessage == nullptr) {
		reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
		return controller_interface::return_type::OK;
	}

	if(!(std::isfinite(imuMessage->orientation.x) && std::isfinite(imuMessage->orientation.y)
			&& std::isfinite(imuMessage->orientation.z) && std::isfinite(imuMessage->orientation.w))) {
		reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
		return controller_interface::return_type::OK;
	}

	tf2::Quaternion orientation;
	tf2::convert(imuMessage->orientation, orientation);

	// Get the angle with the z direction,
	reference_interfaces_[0] = orientation.y();

	if(speedMessage != nullptr && std::isfinite(speedMessage->twist.linear.x)
		&& std::isfinite(speedMessage->twist.angular.z)) {
		reference_interfaces_[1] = speedMessage->twist.linear.x;
		reference_interfaces_[2] = speedMessage->twist.angular.z;
	} else {
		// No speed provided
		reference_interfaces_[1] = 0.0;
		reference_interfaces_[2] = 0.0;
	}

	return controller_interface::return_type::OK;
}

controller_interface::return_type UprightController::update_and_write_commands(const rclcpp::Time & time,
	 const rclcpp::Duration & period) {
	const double angle = reference_interfaces_[0];
	const double twistLinear = reference_interfaces_[1];
	const double twistAngular = reference_interfaces_[2];

	if (!std::isfinite(angle)) {
		// Something failed upstream, and there is not state information
		return controller_interface::return_type::OK;
	}

	// TODO add rolling window average to the IMU data

	const std::string& baselinkFrame = "base_link";
	const std::string& imuFrame = "imu_link";

	geometry_msgs::msg::TransformStamped t;
	try {
		t = tfBuffer->lookupTransform(baselinkFrame, imuFrame, tf2::TimePointZero);
	} catch (const tf2::TransformException& ex) {
		RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), "Could not transform "
					<< imuFrame << " to " << baselinkFrame << ":" << ex.what());
		return controller_interface::return_type::ERROR;
	}

	const double deltaTime = period.seconds();
	const double targetAngle = 0.0;

	const auto error = angle - targetAngle;

	this->errorSum = std::clamp(error * deltaTime + errorSum, -1.0, 1.0);

	const auto p = 31.0 * error;
	const auto i = 25.0 * errorSum;
	const auto d = 0.075 * (angle - lastAngle) / deltaTime;
	const auto pid = p + i + d;

	lastAngle = angle;

	if (this->diffTwistRealtimePublisher->trylock()) {
		auto & message = this->diffTwistRealtimePublisher->msg_;
		// TODO actually use the states
		message.header.stamp = time;
		message.header.frame_id = baselinkFrame;
		message.twist.linear.x = pid + twistLinear;
		message.twist.linear.y = 0.0;
		message.twist.linear.z = 0.0;
		message.twist.angular.x = 0.0;
		message.twist.angular.y = 0.0;
		message.twist.angular.z = twistAngular;
		this->diffTwistRealtimePublisher->unlockAndPublish();
	}

	if (this->angleRealtimePublisher->trylock()) {
		auto & message = this->angleRealtimePublisher->msg_;
		message.data = angle;
		this->angleRealtimePublisher->unlockAndPublish();
	}

	if (this->pidRealtimePublisher->trylock()) {
		auto & message = this->pidRealtimePublisher->msg_;
		message.data = { p, i, d };
		this->pidRealtimePublisher->unlockAndPublish();
	}

	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn UprightController::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */) {
	isActive = false;
	std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), std::numeric_limits<double>::quiet_NaN());
	// TODO see if we need to send 0 velocity command

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */) {
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UprightController::on_error(const rclcpp_lifecycle::State & /* previous_state */) {
	return controller_interface::CallbackReturn::SUCCESS;
}

bool UprightController::on_set_chained_mode(bool chained_mode) {
	return chained_mode;
}

std::vector<hardware_interface::CommandInterface> UprightController::on_export_reference_interfaces() {
	std::vector<hardware_interface::CommandInterface> reference_interfaces;
	reference_interfaces.reserve(reference_interfaces_.size());

	reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name() + std::string("/angle"),
 		hardware_interface::HW_IF_POSITION, &reference_interfaces_[0]));

	reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name() + std::string("/linear"),
		hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[1]));

	reference_interfaces.push_back( hardware_interface::CommandInterface(get_node()->get_name() + std::string("/angular"),
		hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[2]));

	return reference_interfaces;
}

}


#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(upright_controller::UprightController, controller_interface::ChainableControllerInterface)