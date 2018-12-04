cmake_minimum_required(VERSION 2.8.3)
project(echo_helm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  marine_msgs
  project11
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES echo_helm
#  CATKIN_DEPENDS geometry_msgs mavros roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/echo_helm_node.cpp)
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

