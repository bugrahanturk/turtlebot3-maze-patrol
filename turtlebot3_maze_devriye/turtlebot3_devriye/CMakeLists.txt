cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_devriye)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tuw_multi_robot_msgs
)

catkin_package(
	CATKIN_DEPENDS tuw_multi_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(devriye src/devriye.cpp)
add_executable(voronoi_segments src/voronoi_segments.cpp)
target_link_libraries(devriye ${catkin_LIBRARIES})
target_link_libraries(voronoi_segments ${catkin_LIBRARIES})
