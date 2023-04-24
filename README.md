# simple_grasping

ROS2 components for grasping

## Running With Visualization

Following runs a ROS2 Action server. 

```
ros2 run simple_grasping basic_grasping_perception_node --ros-args -p debug_topics:=true frame_id:=base_link
```
Arguments:
- `debug_topics`: If `true`, graspable objects and support planes info will be published as `/object_cloud`, `/support_cloud`, respectively.
- `frame_id`: World frame to be trasnformed from. Default: `base_link`.

Get the info of detected objects and planes by running a ROS Action client:
- [FindGraspableObjects](http://docs.ros.org/en/noetic/api/grasping_msgs/html/action/FindGraspableObjects.html) (link to ROS noetic).
   - As `grasping_msgs/Object` doc states, `shape_msgs/SolidPrimitive` works well with MoveIt.
      Example result callback:
      ```
       void gradpable_result_callback(const GoalHandleFindGraspableObjects::WrappedResult &result) {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(LOGGER, "Goal was aborted");
            return;
         :
          }
             for (auto object : result.result->objects) {
            pose_x = object.object.primitive_poses[0].position.x;
            pose_y = object.object.primitive_poses[0].position.y;
            pose_z = object.object.primitive_poses[0].position.z;
          }
      ```
- [grasping_msgs/GraspPlanning](http://docs.ros.org/en/noetic/api/grasping_msgs/html/action/GraspPlanning.html): To be filled.
