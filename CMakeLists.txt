cmake_minimum_required(VERSION 3.31.6)

project(Mario VERSION 0.1 LANGUAGES CXX)

add_library(yasmin STATIC IMPORTED)
set_target_properties(yasmin PROPERTIES 
	IMPORTED_LOCATION "${Yasmin_INCLUDE_DIR}/lib/libyasmin.so" 
	INTERFACE_INCLUDE_DIRECTORIES "${Yasmin_INCLUDE_DIR}/include/")

add_executable(mario ./src/mario.cpp)

target_include_directories(mario PRIVATE ${CMAKE_SOURCE_DIR}/lib)

target_link_directories(mario PRIVATE yasmin)
