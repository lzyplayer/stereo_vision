cmake_minimum_required(VERSION 3.15)
project(stereo_vision)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3 REQUIRED)

find_package(PCL REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

message("${OpenCV_INCLUDE_DIRS}")

add_executable(stereo_vision main.cpp src/param_loader.cpp src/stereo_constructor.cpp )
target_link_libraries(stereo_vision
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(stereo_vision_example sample_main.cpp src/param_loader.cpp)
target_link_libraries(stereo_vision_example
        ${OpenCV_LIBRARIES}
        )

#add_executable(stereo_vision_example_gpu sample_main_gpu.cpp)
#target_link_libraries(stereo_vision_example_gpu
#        ${OpenCV_LIBRARIES}
#        )
