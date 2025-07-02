cmake_minimum_required(VERSION 3.0)

project(hikrobot_camera)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/camera.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport camera_info_manager
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

find_package(OpenCV REQUIRED)
message(STATUS "Find OpenCV version: "${OpenCV_VERSION})

# 增加x64 aarch64 arm64判断，设置不同的路径
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
    LINK_DIRECTORIES(/opt/MVS/lib/64/)
    message(STATUS "Running on x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    LINK_DIRECTORIES(/opt/MVS/lib/aarch64/)
    message(STATUS "Running on arrch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  /opt/MVS/include/
)


add_executable(${PROJECT_NAME}
  src/hikrobot_camera.cpp src/main.cpp)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)


 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
    MvCameraControl
    MVGigEVisionSDK
    MVRender
    MvUsb3vTL
    MediaProcess
    X11
 )
