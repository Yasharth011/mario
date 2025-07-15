cmake_minimum_required(VERSION 3.31.6)

project(Mario VERSION 0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(yasmin REQUIRED)

find_package(realsense2 REQUIRED)

add_executable(mario ./src/mario.cpp)

target_include_directories(mario PRIVATE ${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(mario PRIVATE yasmin realsense2)
