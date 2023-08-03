cmake_minimum_required(VERSION 3.16)
project(YOLOv8_TensorRT_CPP)

# Use ccache to speed up rebuilds
include(cmake/ccache.cmake)

# Set C++ version and optimization level
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")

# CMake Options
option(ENABLE_BENCHMARKS "Benchmark the preprocessing, inference, and post processing" OFF)
if (ENABLE_BENCHMARKS)
    add_compile_definitions(ENABLE_BENCHMARKS)
endif()
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        cv_bridge_my
        image_transport)
find_package(yaml-cpp REQUIRED)

catkin_package()
set(OpenCV_DIR "/home/hshong/Application/opencv-4.7.0/build")
find_package(OpenCV 4.5 REQUIRED)

# TODO: Specify the path to TensorRT root dir
set(TensorRT_DIR "/home/hshong/Application/TensorRT-8.6.1.6/")
include_directories(${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})
# Build the TensorRT inference engine library
# TensorRT is found and linked by the tensorrt-cpp-api
add_subdirectory(libs/tensorrt-cpp-api)

# Build the YoloV8 library
add_library(YoloV8_TRT SHARED src/yolov8.cpp)
target_link_libraries(YoloV8_TRT PUBLIC tensorrt_cpp_api ${OpenCV_LIBS})
target_include_directories(YoloV8_TRT PUBLIC libs/tensorrt-cpp-api/src)

# Build and link the executables
#add_executable(detect_object_image src/object_detection_image.cpp)
#target_link_libraries(detect_object_image YoloV8_TRT)
#
#add_executable(benchmark src/benchmark.cpp)
#target_link_libraries(benchmark YoloV8_TRT)
#
#add_executable(detect_object_video src/object_detection_video_stream.cpp)
#target_link_libraries(detect_object_video YoloV8_TRT)

add_executable(rosNode src/rosNode.cpp)
target_link_libraries(rosNode YoloV8_TRT ${catkin_LIBRARIES} yaml-cpp)
