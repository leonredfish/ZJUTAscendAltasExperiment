cmake_minimum_required(VERSION 3.5.1)

project(hardware_demo)

add_subdirectory(./font)

set(LIBRARY_OUTPUT_PATH  "../lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../out")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fexec-charset=GBK")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")
set(SRC_LIST 
    key.c       
    pca9557.c       
    task.c
    adxl345.c       
    ds1339u.c  
    i2c.c       
    led.c  
    multiButton.c  
    sht20.c    
    ssd1306.c  
    uart.c
    v4l2.c
    lcd_driver.c
    lcd_gui.c
    serial_listener.c
)
add_definitions(-DENABLE_DVPP_INTERFACE)
set(INC_PATH $ENV{DDK_PATH})

if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})

if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/acllib/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

include_directories(
    ../inc/

)

link_directories(
    /usr/local/lib
    ${LIB_PATH}
)

add_executable(hardware_demo 
    main.cpp
    ${SRC_LIST}
)
target_link_libraries(hardware_demo 
    pthread 
    font
)

install(TARGETS hardware_demo DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

