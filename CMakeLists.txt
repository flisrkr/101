cmake_minimum_required(VERSION 3.10)
project(GAMES101)

# 根据传入的HW变量选择要构建的作业
if(NOT DEFINED HW)
  message(FATAL_ERROR "请指定要构建的作业，例如: -DHW=00")
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
# 添加相应的子目录
add_subdirectory(homework/${HW})