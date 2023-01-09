cmake_minimum_required(VERSION 2.8.3)
project(f110_central_ctrler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES f110_central_ctrler
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
 include
 test
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/periodic_ctrl.cpp
)

add_executable(periodic_ctrl_node src/periodic_ctrl.cpp)
target_link_libraries(periodic_ctrl_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
