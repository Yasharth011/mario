cmake_minimum_required(VERSION 3.31.6)

project(Mario VERSION 0.1 LANGUAGES CXX)

set(YASMIN_INCLUDE_DIR "/nix/store/qmzs2jxj8jg57jzx5wqvfxa84ii8pa5h-yasmin")
add_library(yasmin STATIC IMPORTED)
set_target_properties(yasmin PROPERTIES 
	IMPORTED_LOCATION "${YASMIN_INCLUDE_DIR}/lib/libyasmin.so" 
	INTERFACE_INCLUDE_DIRECTORIES "${YASMIN_INCLUDE_DIR}/include/")

add_library(Mario STATIC ./lib/mario.hpp)
target_include_directories(Mario PTIVATE ./lib)
add_executable(mario ./src/mario.cpp)
target_link_directories(mario PRIVATE yasmin Mario)
