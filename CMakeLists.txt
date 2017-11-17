cmake_minimum_required(VERSION 2.8.3)
project(vicon_outerloop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  serial
)

catkin_package(
	CATKIN_DEPENDS
	std_msgs
	nav_msgs
)


include_directories(

  ${catkin_INCLUDE_DIRS}
)


 add_executable(vicon_outerloop_node src/vicon_outerloop_node.cpp)
 target_link_libraries(
   vicon_outerloop_node
   ${catkin_LIBRARIES}
 )
