cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED OUTPUT_PATH)
  set(OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

add_definitions(-DOUTPUT_PATH="${OUTPUT_PATH}")

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})
