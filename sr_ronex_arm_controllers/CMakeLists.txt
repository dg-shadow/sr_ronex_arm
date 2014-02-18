cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_arm_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface sr_ronex_utilities pluginlib pr2_controller_interface realtime_tools)

include_directories( include ${catkin_INCLUDE_DIRS})



add_library(sr_ronex_controllers src/general_io_passthrough_controller.cpp src/fake_calibration_controller.cpp)
target_link_libraries(sr_ronex_controllers ${catkin_LIBRARIES})

add_dependencies(sr_ronex_controllers sr_ronex_msgs_gencpp)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface pr2_mechanism_controllers pr2_controller_interface pr2_mechanism_model pluginlib
    INCLUDE_DIRS include
)


