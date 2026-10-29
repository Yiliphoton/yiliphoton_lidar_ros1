cmake_minimum_required(VERSION 3.5)
project(lidar_sdk VERSION 1.1.1)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()
message("-- Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================
#  Options
# ============================================================
option(DISABLE_PCAP_PARSE        "Disable PCAP file parse"             OFF)
option(ENABLE_TRANSFORM          "Enable point transform (Eigen)"       OFF)
option(ENABLE_DOUBLE_RCVBUF      "Double UDP receive buffer size"       ON)
option(ENABLE_THREAD_PRIORITY    "Raise recv/process thread priority"   OFF)
option(ENABLE_PCL_POINTCLOUD     "Use PCL PointCloud type"              OFF)
option(ENABLE_CRC32_CHECK        "Enable CRC32 check on MSOP packets"   OFF)
option(ENABLE_POINT_EXTEND_FIELD "Enable azimuth/elevation in output"   OFF)
option(COMPILE_DEMOS             "Build demo programs"                  ON)
option(COMPILE_TOOLS             "Build tool programs"                  OFF)

# ============================================================
#  Platform
# ============================================================
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-parameter")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -g")
endif()

# ============================================================
#  Include paths
# ============================================================
set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
include_directories(${DRIVER_INCLUDE_DIRS})

# ============================================================
#  External dependencies
# ============================================================
list(APPEND EXTERNAL_LIBS pthread)

if(NOT DISABLE_PCAP_PARSE)
  list(APPEND EXTERNAL_LIBS pcap)
else()
  add_definitions(-DDISABLE_PCAP_PARSE)
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Software filter precompiled library
find_library(LIDAR_ALGO_LIB lidar_algo PATHS ${CMAKE_CURRENT_LIST_DIR}/lib)
if(LIDAR_ALGO_LIB)
  list(APPEND EXTERNAL_LIBS ${LIDAR_ALGO_LIB})
  message("-- Found software filter library: ${LIDAR_ALGO_LIB}")
else()
  message(WARNING "Software filter library (liblidar_algo) not found")
endif()

# ============================================================
#  Feature flags
# ============================================================
if(ENABLE_TRANSFORM)
  add_definitions(-DENABLE_TRANSFORM)
endif()


if(ENABLE_DOUBLE_RCVBUF)
  add_definitions(-DENABLE_DOUBLE_RCVBUF)
endif()

if(ENABLE_THREAD_PRIORITY)
  add_definitions(-DENABLE_THREAD_PRIORITY)
  add_definitions(-DHIGH_THREAD_PRIORITY=11)
endif()

if(ENABLE_PCL_POINTCLOUD)
  add_definitions(-DENABLE_PCL_POINTCLOUD)
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  list(APPEND EXTERNAL_LIBS ${PCL_LIBRARIES})
endif()

if(ENABLE_CRC32_CHECK)
  add_definitions(-DENABLE_CRC32_CHECK)
endif()

if(ENABLE_POINT_EXTEND_FIELD)
  add_definitions(-DENABLE_POINT_EXTEND_FIELD)
endif()

# ============================================================
#  Core library (compiled .cpp files from the SDK)
# ============================================================
set(LIDAR_SDK_SOURCES
  src/lidar_sdk/driver/decoder/reflectivity_compensation.cpp
)

add_library(lidar_driver_core STATIC ${LIDAR_SDK_SOURCES})
target_include_directories(lidar_driver_core PUBLIC ${DRIVER_INCLUDE_DIRS})
list(APPEND EXTERNAL_LIBS lidar_driver_core)

# ============================================================
#  Sub-targets
# ============================================================
if(COMPILE_DEMOS)
  add_subdirectory(demo)
endif()

if(COMPILE_TOOLS)
  add_subdirectory(tool)
endif()
