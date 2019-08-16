cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_definitions(-std=c++11)

# set (PROJ4_LIBRARIES proj4::proj)

#find_package(PROJ4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters
  geometry_msgs
  nav_msgs
  tf
  #four_wheel_steering_msgs
  #novatel_gps_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
#  ${PROJ4_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} libproj.so) # ${PROJ4_LIBRARIES}
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
