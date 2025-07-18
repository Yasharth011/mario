cmake_minimum_required(VERSION 3.31.6)

project(Mario VERSION 0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES 
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

find_package(yasmin REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(onnxruntime REQUIRED)

add_library(serial ${CMAKE_SOURCE_DIR}/src/serialib.cpp)
target_include_directories(serial PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(mario ${CMAKE_SOURCE_DIR}/src/mario.cpp)

target_include_directories(mario PUBLIC ${OpenCV_INCLUDE_DIRS} PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(mario PUBLIC ${OpenCV_LIBS} onnxruntime PRIVATE yasmin realsense2 serial)
