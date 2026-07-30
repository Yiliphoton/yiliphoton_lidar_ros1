cmake_minimum_required(VERSION 3.5)
project(depth_make)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs pcl_conversions pcl_ros)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(depth_make src/depth_make.cpp)
target_link_libraries(depth_make ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS depth_make RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
