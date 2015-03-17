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
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    grasping_msgs
    moveit_msgs
    pcl_ros
    sensor_msgs
    shape_msgs
)

include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                   )

### Build basic_grasping_perception
add_executable(basic_grasping_perception src/basic_grasping_perception.cpp
                                         src/cloud_tools.cpp
                                         src/object_support_segmentation.cpp
                                         src/shape_extraction.cpp
                                         src/shape_grasp_planner.cpp)
target_link_libraries(basic_grasping_perception ${Boost_LIBRARIES}
                                                ${catkin_LIBRARIES}
                                                ${PCL_LIBRARIES})
add_dependencies(basic_grasping_perception grasping_msgs_generate_messages_cpp)

### Test
if (CATKIN_ENABLE_TESTING)
add_subdirectory(test)
endif()

### Install
install(
  TARGETS basic_grasping_perception
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
