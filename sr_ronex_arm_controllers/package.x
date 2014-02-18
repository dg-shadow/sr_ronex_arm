<?xml version="1.0"?>
<package>
  <name>sr_ronex_arm_controllers</name>
  <version>0.0.0</version>
  <description>The sr_ronex_arm_controllers package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="dg@todo.todo">dg</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but mutiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/sr_ronex_arm_controllers</url> -->


  <!-- Author tags are optional, mutiple are allowed, one per tag -->
  <!-- Authors do not have to be maintianers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- Dependencies needed to compile this package. -->
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>std_srvs</build_depend>
  <build_depend>sr_ronex_msgs</build_depend>
  <build_depend>sr_ronex_hardware_interface</build_depend>
  <build_depend>sr_ronex_utilities</build_depend>
  <build_depend>pr2_mechanism_controllers</build_depend>
  <build_depend>pr2_controller_interface</build_depend>
  <build_depend>pr2_mechanism_model</build_depend>
  <build_depend>pluginlib</build_depend>

  <!-- Dependencies needed after this package is compiled. -->
  <run_depend>roscpp</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>std_srvs</run_depend>
  <run_depend>sr_ronex_msgs</run_depend>
  <run_depend>sr_ronex_hardware_interface</run_depend>
  <run_depend>sr_ronex_utilities</run_depend>
  <run_depend>pr2_mechanism_controllers</run_depend>
  <run_depend>pr2_controller_interface</run_depend>
  <run_depend>pr2_mechanism_model</run_depend>
  <run_depend>pluginlib</run_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <pr2_controller_interface plugin="${prefix}/controller_plugins.xml"/>
  </export>
</package>