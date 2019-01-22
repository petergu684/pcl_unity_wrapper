cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_wrapper)
find_package(PCL 1.5 REQUIRED)
#find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (wrapper wrapper.cpp)
target_link_libraries (wrapper ${PCL_LIBRARIES} )