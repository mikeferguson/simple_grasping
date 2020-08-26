cmake_minimum_required(VERSION 2.8.3)
project(simple_grasping)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    cmake_modules
    geometry_msgs
    grasping_msgs
    message_generation
    moveit_msgs
    pcl_ros
    roscpp
    sensor_msgs
    shape_msgs
    tf
)
find_package(Eigen REQUIRED)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    grasping_msgs
    moveit_msgs
    pcl_ros
    sensor_msgs
    shape_msgs
  INCLUDE_DIRS include
  LIBRARIES
    simple_grasping
)

include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

### Build simple_grasping library
add_library(simple_grasping
  src/cloud_tools.cpp
  src/object_support_segmentation.cpp
  src/shape_extraction.cpp
  src/shape_grasp_planner.cpp
)
target_link_libraries(simple_grasping
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(simple_grasping grasping_msgs_generate_messages_cpp)

### Build basic_grasping_perception
add_executable(basic_grasping_perception src/basic_grasping_perception.cpp)
target_link_libraries(basic_grasping_perception
  simple_grasping
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(basic_grasping_perception grasping_msgs_generate_messages_cpp)

### Build grasp_planner_node
add_executable(grasp_planner_node src/grasp_planner_node.cpp)
target_link_libraries(grasp_planner_node
  simple_grasping
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(grasp_planner_node grasping_msgs_generate_messages_cpp)

### Test
if (CATKIN_ENABLE_TESTING)
add_subdirectory(test)
endif()

### Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS simple_grasping basic_grasping_perception grasp_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
