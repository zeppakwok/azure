# # cmake_minimum_required(VERSION 3.10)

# # project(azure_test)

# # set(CMAKE_CXX_FLAGS "-std=c++14 -mfma")

# # include_directories(/home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/include)

# # add_executable(azure_test main.cpp)

# # target_link_libraries(azure_test /usr/local/lib/libk4a.so)

# # add_executable(azure_test main.cpp)

# # target_link_libraries(azure_test PRIVATE ${K4A_LIBRARIES})

# # target_include_directories(azure_test PRIVATE ${K4A_INCLUDE_DIRS})

# cmake_minimum_required(VERSION 3.10)

# project(azure_test)

# set(CMAKE_CXX_STANDARD 14)
# # 设置 Azure Kinect SDK 的路径
# # set(AZURE_KINECT_SDK_PATH "/home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/build")
# # set(K4A_INCLUDE_DIRS "/usr/local/include/k4a")
# # set(K4A_LIB "/usr/local/lib")

# find_package(k4a REQUIRED)
# find_package(OpenCV REQUIRED)
# include_directories( ${CMAKE_CURRENT_LIST_DIR} ) # 包含当前目录下我的头文件
# include_directories( ${OpenCV_INCLUDE_DIRS} )
# # 添加库目录
# link_directories(${K4A_LIB})

# # 添加可执行文件
# add_executable(azure_test main.cpp)


# # 添加包含目录
# # target_include_directories(azure_test  )


# # 链接 Azure Kinect SDK 库
# target_link_libraries(azure_test  k4a::k4a   ${OpenCV_LIBS})#${K4A_INCLUDE_DIRS}


# # cmake needs this line
# cmake_minimum_required(VERSION 3.18)
 
# # # Define project name
# project(WDKinectDKDemo)
 
# # # Kinect DK相机
# # set(AZURE_KINECT_SDK_PATH "/home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/build")
# find_package(k4a 1.4.1 REQUIRED PATHS "/home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/build" NO_DEFAULT_PATH)# 关键。因为后面的target_link_libraries中用到了k4a::k4a，所以在这儿加了一行，否则会显示未定义声明
# # Find OpenCV
# find_package(OpenCV REQUIRED)
 
# # Add includes
# include_directories( ${CMAKE_CURRENT_LIST_DIR} ) # 包含当前目录下我的头文件
# include_directories( ${OpenCV_INCLUDE_DIRS} )
 
# # Declare the executable target built from your sources
# add_executable(
# 	WDKinectDKDemo 
# 	main.cpp 
# )
 
# # Link your application with other libraries
# target_link_libraries(WDKinectDKDemo k4a::k4a ${OpenCV_LIBS})
 
 
# # Enable C++11
# set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
 
# cmake_minimum_required(VERSION 3.18)
# project(azure_test)
# unset(k4a_DIR CACHE)
# include_directories(${K4A_INCLUDE_DIRS})
# link_directories(${K4A_LIB})
# add_executable(azure_test main.cpp)
# set(K4A_LIBS k4a::k4a;k4a::k4arecord)
# target_link_libraries(azure_test PRIVATE k4a)
cmake_minimum_required(VERSION 3.18)
project(azure_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-mfma")

#set(CMAKE_PREFIX_PATH /usr/local/lib)
set(K4A_SDK_INCLUDE_DIR /home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/include)
set(K4A_LIBRARIES /usr/local/lib/libk4a.so)  
set(K4ARECORD_LIBRARIES /usr/local/lib/libk4arecord.so) 
find_package(OpenCV REQUIRED)
#include_directories(/home/kwok/software/Azure-Kinect-Sensor-SDK-1.4.1/include)
include_directories(${K4A_SDK_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(azure_test main.cpp)



target_link_libraries(azure_test PRIVATE ${K4A_LIBRARIES} ${K4ARECORD_LIBRARIES} ${OpenCV_LIBS})
