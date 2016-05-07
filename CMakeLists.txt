cmake_minimum_required(VERSION 2.8.3)
project(cjn_turtlebot)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros rospy std_msgs genmsg roslaunch laser_geometry sensor_msgs nav_msgs)
roslaunch_add_file_check(launch)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(navigation src/navigation.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(navigation major_project_generate_messages_cpp)

add_executable(navigation_sim src/navigation_sim.cpp)
target_link_libraries(navigation_sim ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(navigation_sim major_project_generate_messages_cpp)


