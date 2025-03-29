cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include ./include)


if(NOT DEFINED MODELS_PATH)
  set(MODELS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models")
endif()
add_definitions(-DMODELS_PATH="${MODELS_PATH}")

if(NOT DEFINED OUTPUT_PATH)
  set(OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
endif()
add_definitions(-DOUTPUT_PATH="${OUTPUT_PATH}")

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})

# target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)