cmake_minimum_required(VERSION 2.8.3)
project(controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/controller.cpp
)

add_executable(control_node src/control_node.cpp include/${PROJECT_NAME}/controller.cpp)

add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(control_node ${catkin_LIBRARIES})


