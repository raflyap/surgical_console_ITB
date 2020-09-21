################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(data_handler)


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



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
  ui_msgs.msg
  Profile.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy message_runtime
  LIBRARIES data_handler
)

################################################################################
# Build
################################################################################
include_directories(
  include/data_handler
  ${catkin_INCLUDE_DIRS}
)
if(APPLE)
  add_library(data_handler
    src/data_handler/signal_condition.cpp
  )
else()
  add_library(data_handler
    src/data_handler/signal_condition.cpp
  )
endif()

 add_dependencies(data_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(data_handler ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS data_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
catkin_install_python(PROGRAMS
   src/profile_handler_node.py
   src/test_node.py
      src/test_load.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
  #activate code below for Sub-system testing
#  catkin_install_python(PROGRAMS
#    src/profile_handler_node.py
#    src/test_node.py
#    src/test_load.py
#    src/gripper_saver.py
#    src/preload_node.py
#    src/grip_load_test.py
#    src/grip_save_test.py
#    src/gripper_data_handler_r.py
#    src/test_gripper_handler.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

################################################################################
# Test
################################################################################
add_executable(testSC src/sig_Cond_test.cpp)
target_link_libraries(testSC data_handler ${catkin_LIBRARIES})

add_executable(sig_cond_r src/signal_condition_R.cpp)
#set_target_properties(sigrc PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(sig_cond_r ${catkin_LIBRARIES})

add_dependencies(sig_cond_r ${catkin_EXPORTED_TARGETS})
add_dependencies(sig_cond_r data_handler_generate_messages_cpp)

add_executable(sig_cond_l src/signal_condition_L.cpp) 
target_link_libraries(sig_cond_l ${catkin_LIBRARIES}) 

add_dependencies(sig_cond_l ${catkin_EXPORTED_TARGETS})
add_dependencies(sig_cond_l data_handler_generate_messages_cpp)

add_executable(data_logger src/data_logger.cpp)
target_link_libraries(data_logger ${catkin_LIBRARIES})

add_dependencies(data_logger ${catkin_EXPORTED_TARGETS})
add_dependencies(data_logger data_handler_generate_messages_cpp)


add_executable(data_taker_r src/data_taker_r.cpp)
target_link_libraries(data_taker_r ${catkin_LIBRARIES})

add_dependencies(data_taker_r ${catkin_EXPORTED_TARGETS})
add_dependencies(data_taker_r data_handler_generate_messages_cpp)

add_executable(data_taker_l src/data_taker_l.cpp)
target_link_libraries(data_taker_l ${catkin_LIBRARIES})

add_dependencies(data_taker_l ${catkin_EXPORTED_TARGETS})
add_dependencies(data_taker_l data_handler_generate_messages_cpp)