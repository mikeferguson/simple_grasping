catkin_add_gtest(simple_grasping_test_shape_extraction
  test_shape_extraction.cpp
  ../src/shape_extraction.cpp                     
)
target_link_libraries(simple_grasping_test_shape_extraction
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
