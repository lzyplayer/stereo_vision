cmake_minimum_required(VERSION 3.5)
project(stereo_vision)

add_compile_options(-std=c++14  )
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)


set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_PATH}
        include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(stereo_vision
        main.cpp
        src/stereo_match/param_loader.cpp
        src/stereo_match/stereo_constructor.cpp
        src/registration/register.cpp
        )
target_link_libraries(stereo_vision
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(stereo_vision_online
        main_online.cpp
        src/stereo_match/param_loader.cpp
        src/stereo_match/stereo_constructor.cpp
        src/registration/register.cpp
        )
target_link_libraries(stereo_vision_online
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
