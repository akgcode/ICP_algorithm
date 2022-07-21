cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#add_definitions(-std=c++14)
project(runfile)
find_package(PCL 1.10 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_executable(runfile run_icp.cpp)
add_executable(runfile show_icp.cpp) #use any one add_executable at a time
target_link_libraries (runfile ${PCL_LIBRARIES})