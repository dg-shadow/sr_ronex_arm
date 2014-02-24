cmake_minimum_required(VERSION 2.8.3)
project(sr_ronex_arm_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface sr_ronex_utilities pluginlib pr2_controller_interface realtime_tools)

include_directories( include ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(sr_ronex_arm_controllers src/position_servo_controller.cpp)
target_link_libraries(sr_ronex_arm_controllers ${catkin_LIBRARIES})


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs std_srvs sr_ronex_msgs sr_ronex_hardware_interface pr2_mechanism_controllers pr2_controller_interface pr2_mechanism_model pluginlib sr_ronex_controllers
    INCLUDE_DIRS include
    LIBRARIES sr_ronex_arm_controllers
)

install(TARGETS sr_ronex_arm_controllers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
