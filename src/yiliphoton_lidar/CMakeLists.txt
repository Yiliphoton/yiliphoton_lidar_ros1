cmake_minimum_required(VERSION 3.5)
project(yiliphoton_lidar)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wno-unused-parameter)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

# ============================================================
#  Options
# ============================================================
set(POINT_TYPE XYZIRTC CACHE STRING "Point type: XYZI, XYZIRT, XYZIRTC")

option(ENABLE_TRANSFORM          "Enable point transform"              OFF)
option(ENABLE_DOUBLE_RCVBUF      "Double UDP receive buffer"           ON)
option(ENABLE_POINT_EXTEND_FIELD "Enable azimuth/elevation in output"  OFF)
option(ENABLE_IMU                "Enable IMU motion compensation"      OFF)

add_definitions(-DROS1_FOUND -DRUN_IN_ROS1_OR_ROS2 -DROS1)
add_definitions(-DPROJECT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

if(POINT_TYPE STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
elseif(POINT_TYPE STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
elseif(POINT_TYPE STREQUAL "XYZIRTC")
  add_definitions(-DPOINT_TYPE_XYZIRTC)
endif()
message("-- POINT_TYPE: ${POINT_TYPE}")

if(ENABLE_TRANSFORM)
  add_definitions(-DENABLE_TRANSFORM)
endif()
if(ENABLE_DOUBLE_RCVBUF)
  add_definitions(-DENABLE_DOUBLE_RCVBUF)
endif()
if(ENABLE_POINT_EXTEND_FIELD)
  add_definitions(-DENABLE_POINT_EXTEND_FIELD)
endif()
if(ENABLE_IMU)
  add_definitions(-DENABLE_IMU -DENABLE_MOTION_COMPENSATION)
endif()

# ============================================================
#  ROS1 dependencies
# ============================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
#  Message generation (packet msg merged into this package)
# ============================================================
add_message_files(FILES
  YlplidarPacket.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime roslib
)

# ============================================================
#  SDK
# ============================================================
set(YLP_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/driver_sdk)
set(YLP_SDK_INCLUDE ${YLP_SDK_DIR}/src)

# Select precompiled library directory based on target architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
  set(YLP_LIB_ARCH_DIR arm64)
else()
  set(YLP_LIB_ARCH_DIR x86-64)
endif()
message("-- Target arch lib dir: ${YLP_LIB_ARCH_DIR}")

find_library(YLP_AG1_LIB aglidar_ag1 PATHS ${YLP_SDK_DIR}/lib/${YLP_LIB_ARCH_DIR})

add_library(ylp_driver_core STATIC
  ${YLP_SDK_DIR}/src/ylp_driver/driver/decoder/reflectivity_compensation.cpp
)
target_include_directories(ylp_driver_core PUBLIC ${YLP_SDK_INCLUDE})
target_link_libraries(ylp_driver_core Eigen3::Eigen pcap pthread)

# ============================================================
#  Main node
# ============================================================
add_executable(yiliphoton_lidar_node
  node/yiliphoton_lidar_node.cpp
  src/manager/node_manager.cpp
)

target_include_directories(yiliphoton_lidar_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${YLP_SDK_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(yiliphoton_lidar_node
  ylp_driver_core
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pthread pcap
  $<$<BOOL:${YLP_AG1_LIB}>:${YLP_AG1_LIB}>
)

add_dependencies(yiliphoton_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(yiliphoton_lidar_node PROPERTIES
  INSTALL_RPATH "${YLP_SDK_DIR}/lib/${YLP_LIB_ARCH_DIR}"
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

# ============================================================
#  Install
# ============================================================
install(TARGETS yiliphoton_lidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  ${YLP_SDK_DIR}/src/ylp_driver/config/V192-Correction.json
  ${YLP_SDK_DIR}/src/ylp_driver/config/P60-Correction.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY ${YLP_SDK_DIR}/lib/${YLP_LIB_ARCH_DIR}/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  FILES_MATCHING PATTERN "*.so*"
)
