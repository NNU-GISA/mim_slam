cmake_minimum_required(VERSION 2.8.3)
project(mim_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(PCL REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS 
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
      ${GTSAM_INCLUDE_DIR}
)

link_directories(
      include
	${PCL_LIBRARY_DIRS}
      ${GTSAM_LIBRARY_DIRS}
)

add_executable(stablePointExtract src/stablePointExtract.cpp)
target_link_libraries(stablePointExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(laserOdometry src/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(laserMapping src/laserMapping.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)


