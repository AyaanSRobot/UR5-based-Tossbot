cmake_minimum_required(VERSION 2.8)
project( pylon_opencv )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# OpenCV
find_package( OpenCV REQUIRED )

# Pylon 
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )
find_package(Pylon)
find_package(ur_rtde REQUIRED)
include_directories(${Pylon_INCLUDE_DIRS})


add_executable( main main.cpp
    client.h
    client.cpp
    server.h
    server.cpp
    wsg.h
    wsg.cpp)
target_link_libraries( main PRIVATE ur_rtde::rtde ${OpenCV_LIBS} ${Pylon_LIBRARIES})
