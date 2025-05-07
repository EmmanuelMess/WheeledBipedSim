cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# WARNING ffast-math will break Eigen's Eigen::BDCSVD, and the Solver
string(CONCAT CXX_FLAGS_WARN
    "-Wall -Wextra -Wfloat-equal -Wundef -Wshadow -Wpointer-arith -Wcast-align -Wstrict-overflow -Wwrite-strings "
    "-Wcast-qual -Wswitch-default -Wswitch-enum -Wconversion -Wunreachable-code -Wformat=2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS_WARN}")

set(CMAKE_CXX_FLAGS_DEBUG "-Og -ggdb3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(wheeled_biped_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_toolbox  REQUIRED)
find_package(controller_interface  REQUIRED)
find_package(hardware_interface  REQUIRED)
find_package(pluginlib  REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# get include dirs from control_toolbox for the custom validators
get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(upright_controller_parameters
    src/upright_controller_parameter.yaml
    ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp
)


add_library(upright_controller SHARED
    src/upright_controller.cpp
)
target_compile_features(upright_controller PUBLIC cxx_std_20)
target_include_directories(upright_controller PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/upright_controller>
)

target_link_libraries(upright_controller
    PUBLIC
    upright_controller_parameters
    control_toolbox::rate_limiter_parameters)
ament_target_dependencies(upright_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface upright_controller.xml)

ament_target_dependencies(upright_controller PUBLIC ament_cmake
    controller_interface
    control_toolbox
    hardware_interface
    pluginlib
    realtime_tools
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros)
pluginlib_export_plugin_description_file(controller_interface upright_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
install(
  DIRECTORY launch config
  DESTINATION share/wheeled_biped_control
)
install(TARGETS upright_controller upright_controller_parameters
    EXPORT export_upright_controller
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_export_targets(export_upright_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake
    controller_interface
    control_toolbox
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros)
ament_package()
