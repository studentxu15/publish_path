cmake_minimum_required(VERSION 3.0.2)
project(publish_path)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

######################
### Packages
######################

## 查找catkin并包含所有依赖项
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  tf
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

add_message_files(
  FILES
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/Reconfig.cfg
)

## 配置catkin
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  DEPENDS
  system_lib
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

## 指定头文件的位置
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 编译可执行文件及其依赖的源文件
add_executable(${PROJECT_NAME} src/publish_path.cpp)
## 添加编译目标的依赖
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
## 指定链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)