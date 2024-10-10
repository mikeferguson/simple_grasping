# simple_grasping

ROS2 components for simple perception and grasping.

## Basic Grasping Perception Node

The most common use of this package is to run the
``basic_grasping_perception_node``. This node connects to an RGBD
camera and offers a ``find_objects`` action server.

The node can be run with visualization using:

```
ros2 run simple_grasping basic_grasping_perception_node --ros-args -p debug_topics:=true
```

### Parameters

In addition to the following parameters, this node supports all of
the same parameters as the 

 * **debug_topics** - If true, debug topics will be published.
   Default: false.
 * **continuous_detection** - If true, will continuously segment
   objects and support surfaces. This is only useful if using
   the debug topics downstream. Default: false.
 * **frame_id** - frame ID that RGBD point cloud should be
   transformed into - the "world frame". Default: "base_link".

## Subscribed Topics

 * **/head_camera/depth_registered/points** - RGBD camera input.

## Published Topics

 * **object_cloud** - Colorized point cloud of objects detected.
   Only published if **debug_topics** is set to true.
 * **surface_cloud** - Colorized point cloud of surfaces detected.
   Only published if **debug_topics** is set to true.

## Actions

 * **find_objects** - primary action interface, of type
   ``grasping_msgs::action::FindGraspableObjectsAction``. Returns
   an array of ``grasping_msgs::msg::GraspableObjects`` and an
   array of support surfaces as ``grasping_msgs::msg::Object``

For an example using this node, see ``ubr1_demo`` in the
[ubr_reloaded package](https://github.com/mikeferguson/ubr_reloaded).

## Standalone Grasp Planner

The grasp planner can also be run standalone using ``grasp_planner_node``.

### Parameters

The grasp planner works for a parallel jaw gripper.

 * **gripper/left_joint** - name of the left gripper joint. Default: "l_gripper_finger_joint".
 * **gripper/right_joint** - name of the right gripper joint. Default: "r_gripper_finger_joint".
 * **gripper/max_opening** - maximum opening of the gripper, in meters. Default: 0.110.
 * **gripper/max_effort** - maximum effort of the gripper, in Newtons. Default: 50.0.
 * **gripper/finger_depth** - length of the gripper fingers, in meters. The grasp will be
   centered in this region. Default: 0.02.
 * **gripper/grasp_duration** - maximum amount of time to wait for gripper to close, in
   seconds. Default: 2.0.
 * **gripper/gripper_tolerance** - the gripper will open at least this much larger
   than the object. Size in meters. Default: 0.02.
 * **gripper/approach/frame** - name of the robot link the approach portion of the
   grasp will be referenced to. Default: "wrist_roll_link".
 * **gripper/approach/min** - minimum distance that gripper needs to approach,
   in meters. Default: 0.1.
 * **gripper/approach/max** - maximum distance that gripper should use for approach,
   in meters. Default: 0.15.
 * **gripper/retreat/frame** - name of the robot link the retreat portion of the
   grasp will be referenced to. Default: "wrist_roll_link".
 * **gripper/retreat/min** - minimum distance that gripper needs to retreat,
   in meters. Default: 0.1.
 * **gripper/retreat/max** - maximum distance that gripper should use for retreat,
   in meters. Default: 0.15.
 * **gripper/tool_to_planning_frame** - distance between planning frame and tool
   frame, in meters. The planning frame is usually the last wrist frame and so
   this distance offsets the "tool" location, which is usually the center of the
   gripper fingers, from that wrist frame. Default: 0.165.

## Actions

 * **plan** - primary action interface, of type
   ``grasping_msgs::action::GraspPlanningAction``. Returns
   an array of ``moveit_msgs::msg::Grasp`` for a given object.
