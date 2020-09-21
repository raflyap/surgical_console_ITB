################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(sensor_handler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation)


################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  Float64ArrayLength7.msg
  sensorInstruction.msg
  sensorData.msg
  gripperLimit.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy message_runtime
  LIBRARIES sensor_handler
)

################################################################################
# Build
################################################################################
include_directories(
  include/sensor_handler
  ${catkin_INCLUDE_DIRS}
)

add_library(sensor_handler
  src/sensor_handler/forwardKinematic.cpp
)

add_dependencies(sensor_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_handler ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS sensor_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

  #activate code below for Sub-system testing
 catkin_install_python(PROGRAMS
#    src/profile_handler_node.py
#    src/test_node.py
#    src/test_load.py
#    src/gripper_saver.py
#    src/preload_node.py
   src/grip_load_test.py
   src/grip_save_test.py
   src/gripper_calibrator_r.py
   src/gripper_calibrator_l.py
   src/test_gripper_handler.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
################################################################################
# Test
################################################################################

# add_executable(calibrate_encoder src/calibrationTest.cpp)
# target_link_libraries(calibrate_encoder ${catkin_LIBRARIES})
# add_dependencies(calibrate_encoder sensor_handler_generate_messages_cpp)

add_executable(enc_calibrator src/enc_calibrator_node.cpp)
target_link_libraries(enc_calibrator ${catkin_LIBRARIES})
add_dependencies(enc_calibrator sensor_handler_generate_messages_cpp)

add_executable(sensor_node_r src/sensor_node_r.cpp)
target_link_libraries(sensor_node_r ${catkin_LIBRARIES})
add_dependencies(sensor_node_r sensor_handler_generate_messages_cpp)

add_executable(sensor_node_l src/sensor_node_l.cpp)
target_link_libraries(sensor_node_l ${catkin_LIBRARIES})
add_dependencies(sensor_node_l sensor_handler_generate_messages_cpp)

add_executable(fk_node_r src/fk_node_r.cpp)
target_link_libraries(fk_node_r sensor_handler ${catkin_LIBRARIES})
add_dependencies(fk_node_r ${catkin_EXPORTED_TARGETS})
add_dependencies(fk_node_r sensor_handler_generate_messages_cpp)

add_executable(fk_node_l src/fk_node_l.cpp)
target_link_libraries(fk_node_l sensor_handler ${catkin_LIBRARIES})
add_dependencies(fk_node_l ${catkin_EXPORTED_TARGETS})
add_dependencies(fk_node_l sensor_handler_generate_messages_cpp)