project(f1_tracks)
cmake_minimum_required(VERSION 3.16)

message(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if(${CMAKE_BUILD_TYPE} MATCHES "Release")
  message(STATUS "Using additional flags for Release mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
elseif(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  message(STATUS "Using additional flags for Debug mode")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
endif()

message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gflags REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_subdirectory(src/shared)
include_directories(src/shared)
include_directories(${glog_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set(libs Eigen3::Eigen amrl_shared_lib gflags ${glog_LIBRARIES} ${Boost_LIBRARIES})

add_executable(trackpoints_to_scans src/trackpoints_to_scans.cpp
                                    src/vector_map/vector_map.cpp)
target_link_libraries(trackpoints_to_scans ${libs})
