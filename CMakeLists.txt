cmake_minimum_required(VERSION 3.5)
project(simple_grasping)

#find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(common_depends
  geometry_msgs
  grasping_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  shape_msgs
  tf2_ros
)

### Build simple_grasping library
add_library(simple_grasping SHARED
  src/cloud_tools.cpp
  src/object_support_segmentation.cpp
  src/shape_extraction.cpp
  src/shape_grasp_planner.cpp
)
ament_target_dependencies(simple_grasping ${common_depends})
target_link_libraries(simple_grasping ${PCL_LIBRARIES})

### Build basic_grasping_perception
add_library(basic_grasping_perception SHARED
  src/basic_grasping_perception.cpp
)
ament_target_dependencies(basic_grasping_perception ${common_depends})
target_link_libraries(basic_grasping_perception simple_grasping ${PCL_LIBRARIES})
rclcpp_components_register_node(basic_grasping_perception
  PLUGIN "simple_grasping::BasicGraspingPerception"
  EXECUTABLE basic_grasping_perception_node
)

### Build grasp_planner_node
add_library(grasp_planner SHARED
  src/grasp_planner_node.cpp
)
ament_target_dependencies(grasp_planner ${common_depends})
target_link_libraries(grasp_planner simple_grasping ${PCL_LIBRARIES})
rclcpp_components_register_node(grasp_planner
  PLUGIN "simple_grasping::GraspPlannerNode"
  EXECUTABLE grasp_planner_node
)

### Test
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(simple_grasping_test_shape_extraction
    test/test_shape_extraction.cpp
  )
  ament_target_dependencies(simple_grasping_test_shape_extraction ${common_depends})
  target_link_libraries(simple_grasping_test_shape_extraction simple_grasping ${PCL_LIBRARIES})

  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()

### Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/do_perception.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    simple_grasping
    basic_grasping_perception
    grasp_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PACKAGE_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(simple_grasping)
ament_export_dependencies(
  geometry_msgs
  grasping_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  shape_msgs
  tf2_ros
)
ament_package()
