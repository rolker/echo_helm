cmake_minimum_required(VERSION 2.8.3)
project(echo_helm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  project11_msgs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs mavros roscpp project11_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/echo_helm_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

