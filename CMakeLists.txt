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
find_package(motionplanner REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(stella_vslam REQUIRED)
pkg_check_modules(Asio asio)
pkg_check_modules(Cobs cobs)

# tarzan
add_library(tarzan src/tarzan.cpp )
target_include_directories(tarzan PUBLIC include ${Cobs_INCLUDE_DIRS} ${Asio_INCLUDE_DIRS})
target_link_libraries(tarzan ${Cobs_LDFLAGS})

# alloc
add_library(alloc bin/alloc.cpp)
target_include_directories(alloc PUBLIC include)

# localize (slam)
add_library(localize bin/localize.cpp)
target_include_directories(
  localize
  PUBLIC include ${rerun_sdk_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
         ${GLEW_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS}
         ${Pangolin_INCLUDE_DIRS})
target_link_libraries(
  localize
  rerun_sdk
  Eigen3::Eigen
  stella_vslam::stella_vslam
  ${realsense2_LIBRARY}
  ${GLEW_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen)

option(BUILD_TESTS "Build Test Scipts" ON)
if(BUILD_TEST)
	# tarzan test 
	add_executable(tarzan_test src/tarzan.cpp)
	target_compile_definitions(tarzan_test PRIVATE TARZAN_TEST_CPP=1)
	target_link_libraries(tarzan_test PRIVATE tarzan)
endif()

add_executable(mario ${CMAKE_SOURCE_DIR}/src/mario.cpp)

target_include_directories(mario PRIVATE ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${rerun_sdk_INCLUDE_DIRS})

target_link_libraries(mario PRIVATE ${OpenCV_LIBS} onnxruntime Eigen3::Eigen ${PCL_LIBRARIES} yasmin realsense2 Boost::program_options tarzan motionplanner rerun_sdk stella_vslam)
