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

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the messages used by the MoveBase action, including the
# goal message and the result message.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == '__main__':
    # Initialize a ROS Node
    rospy.init_node('move_turtlebot')

    # Create a SimpleActionClient for the 'move_base' action server.
    turtlebot_navigation_client = actionlib.SimpleActionClient("move_base",
                                                               MoveBaseAction)

    # Wait until the move_base action server becomes available.
    rospy.loginfo("Waiting for move_base action server to come up...")
    turtlebot_navigation_client.wait_for_server()
    rospy.loginfo("move_base action server is available...")

    # Creates a goal to send to the action server.
    turtlebot_robot1_goal = MoveBaseGoal()

    # Construct the target pose for the turtlebot in the "map" frame.
    turtlebot_robot1_goal.target_pose.header.stamp = rospy.Time.now()
    turtlebot_robot1_goal.target_pose.header.frame_id = "map"
    turtlebot_robot1_goal.target_pose.header.seq = 1
    turtlebot_robot1_goal.target_pose.pose.position.x = 0.13
    turtlebot_robot1_goal.target_pose.pose.position.y = 1.44
    turtlebot_robot1_goal.target_pose.pose.position.z = 0.0
    turtlebot_robot1_goal.target_pose.pose.orientation.x = 0.0
    turtlebot_robot1_goal.target_pose.pose.orientation.y = 0.0
    turtlebot_robot1_goal.target_pose.pose.orientation.z = 0.0
    turtlebot_robot1_goal.target_pose.pose.orientation.w = 1.0

    # Send the goal to the action server.
    try:
        turtlebot_navigation_client.send_goal(turtlebot_robot1_goal)
        rospy.loginfo("Goal sent to move_base action server.")
        rospy.loginfo("Goal position x: %s  y: %s",
                      turtlebot_robot1_goal.target_pose.pose.position.x,
                      turtlebot_robot1_goal.target_pose.pose.position.y)

        # Wait for the server to finish performing the action.
        turtlebot_navigation_client.wait_for_result()

        # Display a log message depending on the navigation result.
        navigation_result_status = turtlebot_navigation_client.get_state()
        if GoalStatus.SUCCEEDED != navigation_result_status:
            rospy.logerr('Navigation to the desired goal failed')
            rospy.logerr(':( -- Sorry, try again!)')
        else:
            rospy.loginfo('Hooray! Successfully reached the desired goal!')

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received to stop ROS node.")
