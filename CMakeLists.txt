cmake_minimum_required(VERSION 3.6)
project(project)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp video.cpp video.h Raycast.cpp Raycast.h utils.cpp utils.h globals.h globals.cpp.cpp)
add_executable(project ${SOURCE_FILES})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(project ${OpenCV_LIBS}) # replace binary_name
