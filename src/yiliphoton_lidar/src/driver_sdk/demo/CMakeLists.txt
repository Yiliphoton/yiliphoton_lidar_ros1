cmake_minimum_required(VERSION 3.5)
project(ylp_driver_demos)

message("-- Building demos")
include_directories(${DRIVER_INCLUDE_DIRS})

add_executable(demo_online demo_online.cpp)
target_link_libraries(demo_online ${EXTERNAL_LIBS})

add_executable(demo_online_multi_lidars demo_online_multi_lidars.cpp)
target_link_libraries(demo_online_multi_lidars ${EXTERNAL_LIBS})

if(NOT DISABLE_PCAP_PARSE)
  add_executable(demo_pcap demo_pcap.cpp)
  target_link_libraries(demo_pcap ${EXTERNAL_LIBS})
endif()
