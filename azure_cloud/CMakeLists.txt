cmake_minimum_required(VERSION 3.18)
project(azure_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-mfma")

#set(CMAKE_PREFIX_PATH /usr/local/lib)
set(K4A_SDK_INCLUDE_DIR /home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/include)
set(K4A_LIBRARIES /usr/local/lib/libk4a.so)  
set(K4ARECORD_LIBRARIES /usr/local/lib/libk4arecord.so) 
find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED)
#include_directories(/home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/include)
include_directories(${K4A_SDK_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(azure_test main.cpp)



target_link_libraries(azure_test PRIVATE ${K4A_LIBRARIES} ${K4ARECORD_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
