cmake_minimum_required(VERSION 3.5)
project(lidar_driver)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wno-unused-parameter)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

# ============================================================
#  Options
# ============================================================
set(POINT_TYPE XYZIRT CACHE STRING "Point type: XYZI, XYZIRT, XYZIRTC")
set_property(CACHE POINT_TYPE PROPERTY STRINGS XYZI XYZIRT XYZIRTC)
if(NOT POINT_TYPE STREQUAL "XYZI" AND NOT POINT_TYPE STREQUAL "XYZIRT" AND NOT POINT_TYPE STREQUAL "XYZIRTC")
  message(FATAL_ERROR "Unsupported POINT_TYPE: ${POINT_TYPE}. Use XYZI, XYZIRT, or XYZIRTC.")
endif()

option(ENABLE_TRANSFORM          "Enable point transform"              OFF)
option(ENABLE_DOUBLE_RCVBUF      "Double UDP receive buffer"           ON)
option(ENABLE_POINT_EXTEND_FIELD "Enable azimuth/elevation in output"  OFF)

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
  LidarPacket.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime roslib
)

# ============================================================
#  SDK
# ============================================================
set(SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/driver_sdk)
set(SDK_INCLUDE ${SDK_DIR}/src)

# Select precompiled library directory based on target architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
  set(LIB_ARCH_DIR arm64)
else()
  set(LIB_ARCH_DIR x86-64)
endif()
message("-- Target arch lib dir: ${LIB_ARCH_DIR}")

find_library(LIDAR_ALGO_LIB lidar_algo PATHS ${SDK_DIR}/lib/${LIB_ARCH_DIR})

add_library(lidar_driver_core STATIC
  ${SDK_DIR}/src/lidar_sdk/driver/decoder/reflectivity_compensation.cpp
)
target_include_directories(lidar_driver_core PUBLIC ${SDK_INCLUDE})
target_link_libraries(lidar_driver_core Eigen3::Eigen pcap pthread)

# ============================================================
#  Main node
# ============================================================
add_executable(lidar_node
  node/lidar_node.cpp
  src/manager/node_manager.cpp
)

target_include_directories(lidar_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${SDK_INCLUDE}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(lidar_node
  lidar_driver_core
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pthread pcap
  $<$<BOOL:${LIDAR_ALGO_LIB}>:${LIDAR_ALGO_LIB}>
)

add_dependencies(lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set_target_properties(lidar_node PROPERTIES
  INSTALL_RPATH "${SDK_DIR}/lib/${LIB_ARCH_DIR}"
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

if(BUILD_PCAP_TEST)
  if(NOT LIDAR_ALGO_LIB)
    message(FATAL_ERROR "BUILD_PCAP_TEST requires liblidar_algo.so in ${SDK_DIR}/lib/${LIB_ARCH_DIR}")
  endif()

  add_executable(lidar_driver_pcap_smoke_test
    tests/pcap_smoke_test.cpp
  )
  target_include_directories(lidar_driver_pcap_smoke_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SDK_INCLUDE}
  )
  target_link_libraries(lidar_driver_pcap_smoke_test
    lidar_driver_core
    ${LIDAR_ALGO_LIB}
    pcap
    pthread
  )
  set_target_properties(lidar_driver_pcap_smoke_test PROPERTIES
    INSTALL_RPATH "${SDK_DIR}/lib/${LIB_ARCH_DIR}"
    LINK_FLAGS "-Wl,--disable-new-dtags"
  )
endif()

# ============================================================
#  Install
# ============================================================
install(TARGETS lidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  ${SDK_DIR}/src/lidar_sdk/config/V192-Correction.json
  ${SDK_DIR}/src/lidar_sdk/config/P60-Correction.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY ${SDK_DIR}/lib/${LIB_ARCH_DIR}/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  FILES_MATCHING PATTERN "*.so*"
)
