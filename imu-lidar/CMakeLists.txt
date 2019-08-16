cmake_minimum_required(VERSION 2.8.3)
project(imu-lidar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf_conversions
  trajectory_msgs
  message_filters
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  wgs_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# find_package(Gflags REQUIRED)
# if (GFLAGS_FOUND)
#   message("--Found Gflags header in: ${GFLAGS_INCLUDE_DIRS}, "
#   "in namespace ${GFLAGS_NAMESPACE}")

#   include_directories(${GFLAGS_INCLUDE_DIRS})
# else (GFLAGS_FOUND)
#   message(FATAL_ERROR "--Can not find Gflags, please install Gflags")
# endif (GFLAGS_FOUND)

  
# find_package(Glog REQUIRED)
# if (GLOG_FOUND)
#   message("--Found Glog header in: ${GLOG_INCLUDE_DIRS}")
# else (GFLAGS_FOUND)
#   message(FATAL_ERROR "--Can not find Glog, please install Glog")
# endif (GFLAGS_FOUND)

find_package (Eigen3 REQUIRED)
message("--Found Eigen3 header: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Ceres REQUIRED)
message("--Found Ceres header: ${CERES_INCLUDE_DIRS}")
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED)
message("--Found Opencv header: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Threads QUIET)

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
#SET( G2O_ROOT /usr/local/include/g2o )
# find_package(G2O REQUIRED)
# message("--Found Opencv header: ${G2O_INCLUDE_DIRS}")
# include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

# LIST(APPEND G2O_LIBS
#   g2o_cli g2o_ext_freeglut_minimal g2o_simulator
#   g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
#   g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
#   g2o_types_sba g2o_types_slam3d g2o_csparse_extension
#   g2o_opengl_helper g2o_solver_dense g2o_stuff
#   g2o_types_sclam2d g2o_parser g2o_solver_pcg
#   g2o_types_data g2o_types_sim3
# )

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_calibration
  CATKIN_DEPENDS roscpp tf trajectory_msgs message_filters geometry_msgs sensor_msgs pcl_ros pcl_conversions nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
add_library(HandeyeLib
   src/HandEyeCalibration.cpp
 )
target_link_libraries(HandeyeLib
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}
 )

# add_library(MultiLidarOptimizationLib
#    src/MultiLidarOptimization.cpp
#  )
# target_link_libraries(MultiLidarOptimizationLib
#    ${catkin_LIBRARIES} ${GLOG_LIBRARIES}  ${GLOG_LIB}  ${GLOG_LIBS} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}
#  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#Build examples
add_executable(handeye_example_node examples/HandeyeExample.cpp)
target_link_libraries(handeye_example_node
   HandeyeLib
 )

add_executable(handeye_exporter_node examples/HandeyeDataExporter.cpp)
target_link_libraries(handeye_exporter_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

add_executable(pcl_viewer_node examples/PclViewer.cpp)
target_link_libraries(pcl_viewer_node
   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
 )

#add_executable(pose_evaluation_node examples/PosesEvaluation.cpp)
#target_link_libraries(pose_evaluation_node
#   ${catkin_LIBRARIES}
##   ${OpenCV_LIBRARIES}
# )


# add_executable(cloud_registration_node examples/CloudRegistration.cpp)
# target_link_libraries(cloud_registration_node
#    ${catkin_LIBRARIES} MultiLidarOptimizationLib ${G2O_LIBS}
# #   ${OpenCV_LIBRARIES}
#  )
#add_executable(multi_lidar_calibration examples/MultiLidarCalibration.cpp)
#target_link_libraries(multi_lidar_calibration
#   ${catkin_LIBRARIES} MultiLidarOptimizationLib
##   ${OpenCV_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sensor_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)