cmake_minimum_required(VERSION 3.0.2)
project(online_photometric_calib)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)

SET(OpenCV_DIR /home/lin/develop/3rd/opencv/install/opencv-4.2.0/lib/cmake/opencv4/)
find_package(OpenCV 4.2.0 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

### Find ROS package
SET(cv_bridge_DIR /home/lin/develop/ros/ros_ws/devel/share/cv_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs rosbag sensor_msgs nav_msgs
  geometry_msgs cv_bridge message_generation
  tf_conversions eigen_conversions
  dynamic_reconfigure message_filters image_transport 
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ?
 CATKIN_DEPENDS roscpp rospy std_msgs rosbag sensor_msgs cv_bridge dynamic_reconfigure
                geometry_msgs message_runtime tf_conversions eigen_conversions
                message_filters image_transport visualization_msgs tf nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${Boost_LIBRARY_DIR})

### build src
set(lib_opc 
  include/optimizer/JacobianGenerator.cpp
  include/optimizer/NonlinearOptimizer.cpp
  include/optimizer/OptimizationBlock.cpp
  include/tracker/GainRobustTracker.cpp
  include/tracker/Tracker.cpp
  include/Database.cpp
  include/RapidExposureTimeEstimator.cpp
  include/VignetteModel.cpp
)

add_executable(opc_node src/opc_node.cpp ${lib_opc})
target_link_libraries(opc_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${Boost_LIBRARIES}
)


##########################################
############### build test ###############
##########################################
