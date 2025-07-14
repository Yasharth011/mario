cmake_minimum_required(VERSION 3.31.6)

project(Mario VERSION 0.1 LANGUAGES CXX)

find_package(yasmin REQUIRED)

add_executable(mario ./src/mario.cpp)

target_include_directories(mario PRIVATE ${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(mario PRIVATE yasmin)
