cmake_minimum_required(VERSION 3.4)
project(orbslam3)

LIST(APPEND CMAKE_MODULE_PATH 
    ${PROJECT_SOURCE_DIR}/cmake
)

add_subdirectory(extern/pybind11)
add_subdirectory(extern/ORB_SLAM3)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

pybind11_add_module(orbslam3 
    src/ORBSLAM3Wrapper.cpp
    src/NDArrayConverter.cpp
    # misc/TuplePoint3fConverter.cpp
)

target_link_libraries(orbslam3
PRIVATE
    ORB_SLAM3
)

target_include_directories(orbslam3
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/ORB_SLAM3>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/ORB_SLAM3/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/ORB_SLAM3/include/CameraModels>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/ORB_SLAM3/Thirdparty/g2o/g2o>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/ORB_SLAM3/Thirdparty/DBoW2/DBoW2>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/ORB_SLAM3/Thirdparty/Sophus/sophus>
)