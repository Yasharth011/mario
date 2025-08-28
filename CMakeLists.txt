cmake_minimum_required(VERSION 3.31.6)

project(Mario VERSION 0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES 
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

find_package(yasmin REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(Boost 1.87.0 COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
pkg_check_modules(Asio asio)
pkg_check_modules(Cobs cobs)

add_library(tarzan src/tarzan.cpp )
target_include_directories(tarzan PUBLIC include ${Cobs_INCLUDE_DIRS} ${Asio_INCLUDE_DIRS})
target_link_libraries(tarzan ${Cobs_LDFLAGS})

option(BUILD_TESTS "Build Test Scipts" ON)
if(BUILD_TEST)
	# tarzan test 
	add_executable(tarzan_test src/tarzan.cpp)
	target_compile_definitions(tarzan_test PRIVATE TARZAN_TEST_CPP=1)
	target_link_libraries(tarzan_test PRIVATE tarzan)
endif()

add_executable(mario ${CMAKE_SOURCE_DIR}/src/mario.cpp)

target_include_directories(mario PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

target_link_libraries(mario PRIVATE ${OpenCV_LIBS} onnxruntime Eigen3::Eigen ${PCL_LIBRARIES} yasmin realsense2 Boost::program_options tarzan)
