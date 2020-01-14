cmake_minimum_required(VERSION 2.8.3)
project(tf_adder)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  rosbag
  cmake_modules
  rospy
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(tf_adder tf_adder.cpp)
target_link_libraries(tf_adder ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  tf_adder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



