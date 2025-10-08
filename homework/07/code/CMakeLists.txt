cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED MODELS_PATH)
  set(MODELS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models")
endif()

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

target_compile_definitions(RayTracing PRIVATE MODELS_PATH="${MODELS_PATH}")
