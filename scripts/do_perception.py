#!/usr/bin/env python3

# Copyright 2020, Michael Ferguson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Michael Ferguson nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from grasping_msgs.action import FindGraspableObjects

class PerceptionClient(Node):

    def __init__(self):
        super().__init__("basic_grasping_perception_client")
        self.action_client = ActionClient(self, FindGraspableObjects, "find_objects")

    def run(self):
        goal = FindGraspableObjects.Goal()
        goal.plan_grasps = True
        self.action_client.wait_for_server()
        self._goal_future = self.action_client.send_goal_async(goal)
        self._goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        for surface in result.support_surfaces:
            self.get_logger().info('{}'.format(surface.header))
            self.get_logger().info('Surface: {}'.format(surface.surface))
        for object in result.objects:
            self.get_logger().info('{}'.format(object.object.header))
            self.get_logger().info('Object on {}: {}'.format(object.object.support_surface, object.object.primitive_poses))
        rclpy.shutdown()

if __name__=="__main__":
    rclpy.init()
    client = PerceptionClient()
    client.run()
    rclpy.spin(client)
