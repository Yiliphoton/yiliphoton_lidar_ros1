cmake_minimum_required(VERSION 3.5)
project(pcl_ros_bag_to_pcd)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_conversions pcl_ros)
find_package(PCL REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bag_to_pcd src/bag_to_pcd.cpp)
target_link_libraries(bag_to_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS bag_to_pcd RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
