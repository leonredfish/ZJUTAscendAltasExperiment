# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(classification_video)

# Compile options
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

set(INC_PATH $ENV{DDK_PATH})

if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend/ascend-toolkit/latest/acllib/include/")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/ascend-toolkit/latest/acllib/lib64/stub")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

# Header path
include_directories(
    ../inc/
    ../../software/ascend_ddk/arm/include/
    ${INC_PATH}
    /usr/include/opencv4
)
# add host lib path
link_directories(
    ../../software/ascend_ddk/arm/lib/
    ${LIB_PATH}
    ${INC_PATH}/atc/lib64
    /usr/lib/aarch64-linux-gnu/
)



add_executable(yolov3_demo
        utils.cpp
        model_process.cpp
        object_detect.cpp
        main.cpp)

target_link_libraries(yolov3_demo 
    ascendcl 
    acl_dvpp 
    stdc++ 
    opencv_highgui  
    opencv_core 
    opencv_imgproc 
    opencv_imgcodecs 
    opencv_calib3d 
    opencv_features2d 
    opencv_videoio  
    avcodec
    avformat
    avutil
    swscale
    protobuf 
    presenteragent)

install(TARGETS yolov3_demo DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})