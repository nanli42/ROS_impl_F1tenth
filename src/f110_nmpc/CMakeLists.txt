cmake_minimum_required(VERSION 2.8.3)
project(f110_nmpc)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-stack-protector")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-stack-protector")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Find catkin and any catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(PythonLibs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
  roscpp
  genmsg
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

### FOR PROFILING (need to set after catkin_package)
#add_compile_options(-pg)
#set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  DEPENDS Eigen
  INCLUDE_DIRS include)

## Build talker and listener
include_directories(
  /usr/local/include
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  src/acado_generated_code_for_f110_nmpc
  src/acado_LO_generated
  src/acado_RO_generated
  external/qpoases/
  external/qpoases/INCLUDE/
  external/qpoases/SRC/
  include/test_WCET
  ../f110_perception/include/f110_opp_detect
)

## Declare a C++ library
# find ACADO generated code
file(GLOB ACADO_GENERATED_SRC
  "src/acado_generated_code_for_f110_nmpc/*.cpp"
  "src/acado_generated_code_for_f110_nmpc/*.c"
  "src/acado_LO_generated/*.cpp"
  "src/acado_LO_generated/*.c"
  "src/acado_RO_generated/*.cpp"
  "src/acado_RO_generated/*.c"
  "external/qpoases/SRC/*.cpp"
)
file(GLOB NMPC_CTRL_SRC
  "src/*.cpp"
  "src/LOROFO.cpp"
)

# make a static lib
add_library(acado_generated_rti STATIC ${ACADO_GENERATED_SRC})

link_directories(/usr/local/lib)

add_executable(nmpc_ctrl_node ${NMPC_CTRL_SRC})
target_link_libraries(nmpc_ctrl_node ${catkin_LIBRARIES} acado_generated_rti)
add_dependencies(nmpc_ctrl_node ${PROJECT_NAME}_generate_messages_cpp)

file(GLOB SRC_FILES_test_WCET "src/test_WCET/*.cpp"
  ../f110_perception/src/f110_opp_detect/handle_lidar.cpp
  ../f110_perception/src/f110_opp_detect/general_utils.cpp
  ../f110_perception/src/f110_opp_detect/load_map.cpp
  src/set_priority.cpp
  src/nmpc_ctrl_callback.cpp
  src/nmpc_ctrl_solver.cpp
  src/nmpc_ctrl_utility.cpp
)
#add_executable(test_WCET_nmpc ${SRC_FILES_test_WCET})
#target_link_libraries(test_WCET_nmpc
#   ${catkin_LIBRARIES}
#   acado_generated_rti
#)
