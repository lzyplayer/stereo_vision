cmake_minimum_required(VERSION 3.15)
project(stereo_vision)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 3 REQUIRED)

find_package(PCL REQUIRED)

set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_PATH}
        include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

message("${OpenCV_INCLUDE_DIRS}")

add_executable(stereo_vision
        main.cpp
        src/stereo_match/param_loader.cpp
        src/stereo_match/stereo_constructor.cpp)
target_link_libraries(stereo_vision
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(stereo_vision_online
        main_online.cpp
        src/stereo_match/param_loader.cpp
        src/stereo_match/stereo_constructor.cpp)
target_link_libraries(stereo_vision
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(stereo_vision_example src/stereo_match/sample_main.cpp src/stereo_match/param_loader.cpp)
target_link_libraries(stereo_vision_example
        ${OpenCV_LIBRARIES}
        )

#add_executable(stereo_vision_example_gpu sample_main_gpu.cpp)
#target_link_libraries(stereo_vision_example_gpu
#        ${OpenCV_LIBRARIES}
#        )
