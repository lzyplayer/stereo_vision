cmake_minimum_required(VERSION 3.15)
project(stereo_vision)

add_compile_options(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_PATH}
        include
        ndt_omp/include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(ndt_omp
        ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
        ndt_omp/src/pclomp/ndt_omp.cpp
        ndt_omp/src/pclomp/gicp_omp.cpp
        )


add_executable(stereo_vision
        main.cpp
        src/stereo_match/param_loader.cpp
        src/stereo_match/stereo_constructor.cpp
        src/registration/register.cpp
        ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
        ndt_omp/src/pclomp/ndt_omp.cpp
        ndt_omp/src/pclomp/gicp_omp.cpp)
target_link_libraries(stereo_vision
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(stereo_vision_online
        main_online.cpp
        src/stereo_match/param_loader.cpp
        src/stereo_match/stereo_constructor.cpp
        src/registration/register.cpp
        ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
        ndt_omp/src/pclomp/ndt_omp.cpp
        ndt_omp/src/pclomp/gicp_omp.cpp)
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
#add_executable(align
#        ndt_omp/apps/align.cpp
#        )
#add_dependencies(align
#        ndt_omp
#        )
#target_link_libraries(align
#        ${PCL_LIBRARIES}
#        ndt_omp
#        )