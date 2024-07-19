#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# TU Delft Robotics Institute.
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
#  * Neither the name of SRI International nor the names of its
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
#
# Authors: the HRWROS mooc instructors
import rospy
import rospkg
import os

# Import the spawn urdf model service from Gazebo.
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, GetModelProperties
from geometry_msgs.msg import Pose


def spawn_unknown_obstacle(obstacle_name, obstacle_model_xml, obstacle_pose):
    # Service proxy to spawn urdf object in Gazebo.
    spawn_obstacle_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model',
                                                SpawnModel)
    # Create service request.
    spawn_obstacle_request = SpawnModelRequest()
    spawn_obstacle_request.model_name = obstacle_name
    spawn_obstacle_request.model_xml = obstacle_model_xml
    spawn_obstacle_request.robot_namespace = ''
    spawn_obstacle_request.initial_pose = obstacle_pose
    spawn_obstacle_request.reference_frame = 'map'
    # Call spawn model service.
    spawn_obstacle_response = spawn_obstacle_service(spawn_obstacle_request)
    if(not spawn_obstacle_response.success):
        rospy.logerr('Could not spawn unknown obstacle')
        rospy.logerr(spawn_obstacle_response.status_message)
    else:
        rospy.loginfo(spawn_obstacle_response.status_message)


def check_obstacle_existence(obstacle_name):
    # Service proxy to spawn urdf object in Gazebo.
    check_obstacle_service = rospy.ServiceProxy('/gazebo/get_model_properties',
                                                GetModelProperties)

    check_obstacle_response = check_obstacle_service(obstacle_name)
    return check_obstacle_response.success


if __name__ == '__main__':
    try:
        # Initialize a ROS Node
        rospy.init_node('spawn_unknown_obstacle')
        # Get file name of the object to be spawned
        rospack = rospkg.RosPack()
        with open(os.path.join(rospack.get_path('hrwros_support'),
                               "urdf/unknown_obstacle/unknown_obstacle.urdf"),
                  "r") as box_file:
            box_xml = box_file.read()
        # Wait for a couple of seconds to prevent the unknown obstacle from
        # being considered as a part of the map.
        rospy.sleep(4)

        if not check_obstacle_existence('unknown_obstacle1'):
            # First obstacle.
            obstacle1_pose = Pose()
            obstacle1_pose.position.x = -2.2
            obstacle1_pose.position.y = 1.0
            obstacle1_pose.position.z = 0.0
            obstacle1_pose.orientation.x = 0.0
            obstacle1_pose.orientation.y = 0.0
            obstacle1_pose.orientation.z = 0.0
            obstacle1_pose.orientation.w = 1.0
            spawn_unknown_obstacle('unknown_obstacle1',
                                   box_xml,
                                   obstacle1_pose)

        if not check_obstacle_existence('unknown_obstacle2'):
            # Second obstacle.
            obstacle2_pose = Pose()
            obstacle2_pose.position.x = -5.0
            obstacle2_pose.position.y = 0.5
            obstacle2_pose.position.z = 0.0
            obstacle2_pose.orientation.x = 0.0
            obstacle2_pose.orientation.y = 0.0
            obstacle2_pose.orientation.z = 0.0
            obstacle2_pose.orientation.w = 1.0
            spawn_unknown_obstacle('unknown_obstacle2',
                                   box_xml,
                                   obstacle2_pose)

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received to stop ROS node.")
