cmake_minimum_required(VERSION 2.8.3)
project(sloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(GTSAM REQUIRED)
INCLUDE_DIRECTORIES("/usr/include/eigen3")
include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIR}
	${GTSAM_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_executable(frameFeature src/frameFeature.cpp)
target_link_libraries(frameFeature ${catkin_LIBRARIES} ${PCL_LIBRARIES}  -lgtsam -ltbb)

add_executable(lidarOdometry src/lidarOdometry.cpp)
target_link_libraries(lidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}  -lgtsam -ltbb)

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}  -lgtsam -ltbb)

