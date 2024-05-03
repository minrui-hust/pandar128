cmake_minimum_required(VERSION 3.15.0)
project(panda128)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -D_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${PROJECT_SOURCE_DIR}/src)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(PCL REQUIRED COMPONENTS common)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(app)
