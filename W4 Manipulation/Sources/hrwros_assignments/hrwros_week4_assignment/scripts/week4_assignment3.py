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
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs


def week4_assignment3():
    # First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('week4_assignment3',
                    anonymous=True)

    # Instantiate a MoveGroupCommander object.  This object is an interface
    # to one group of joints.  In this case the group refers to the joints of
    # robot2. This interface can be used to plan and execute motions on robot2.
    robot2_group = moveit_commander.MoveGroupCommander("robot2")

    # Action clients to the ExecuteTrajectory action server.
    robot2_client = actionlib.SimpleActionClient('execute_trajectory',
                                                 moveit_msgs.msg.ExecuteTrajectoryAction)
    robot2_client.wait_for_server()
    rospy.loginfo('Execute Trajectory server is available for robot2')

    # Move robot2 to R2Home robot pose with the set_named_target API.
    robot2_group.set_named_target("R2Home")

    # Plan to the desired joint-space goal using
    # the default planner (RRTConnect).
    _, plan, _, _ = robot2_group.plan()
    # Create a goal message object for the action server.
    robot2_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    # Update the trajectory in the goal message.
    robot2_goal.trajectory = plan

    # Print message
    rospy.loginfo('Go to Home')

    # Send the goal to the action server.
    robot2_client.send_goal(robot2_goal)
    robot2_client.wait_for_result()

    # Move robot2 to R2PreGrasp pose
    robot2_group.set_named_target("R2PreGrasp")

    _, plan, _, _ = robot2_group.plan()
    robot2_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot2_goal.trajectory = plan

    # Print message
    rospy.loginfo('Go to Pre Grasp')

    robot2_client.send_goal(robot2_goal)
    robot2_client.wait_for_result()

    # Pick motions with the compute_cartesian_path API.
    waypoints = []
    # start with the current pose
    current_robot2_pose = robot2_group.get_current_pose()
    rospy.sleep(0.5)
    current_robot2_pose = robot2_group.get_current_pose()

    # create linear offsets to the current pose
    new_robot2_eef_pose = geometry_msgs.msg.Pose()

    # Manual offsets because we don't have a camera to detect objects yet.
    # Create an x-offset of -5cm to the current x position.
    new_robot2_eef_pose.position.x = current_robot2_pose.pose.position.x - 0.05
    # Create a y-offset of +10cm to the current y position.
    new_robot2_eef_pose.position.y = current_robot2_pose.pose.position.y + 0.10
    # Create a z-offset of -10cm to the current z position.
    new_robot2_eef_pose.position.z = current_robot2_pose.pose.position.z - 0.10

    # Retain orientation of the current pose.
    new_robot2_eef_pose.orientation = copy.deepcopy(current_robot2_pose.pose.orientation)

    # Update the list of waypoints.
    # First add the new desired pose of the end effector for robot2.
    waypoints.append(new_robot2_eef_pose)

    rospy.loginfo('Cartesian path - Waypoint 1:')
    print(new_robot2_eef_pose.position)

    # Then go back to the pose where we started the linear motion from.
    waypoints.append(current_robot2_pose.pose)

    rospy.loginfo('Cartesian path - Waypoint 2:')
    print(current_robot2_pose.pose.position)

    # We want the cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in cartesian
    # translation.  We will specify the jump threshold as 0.0, effectively
    # disabling it.
    fraction = 0.0
    for count_cartesian_path in range(0, 3):
        if fraction < 1.0:
            (plan_cartesian, fraction) = robot2_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
        else:
            break

    robot2_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot2_goal.trajectory = plan_cartesian
    # Print message
    rospy.loginfo('Cartesian movement')

    robot2_client.send_goal(robot2_goal)
    robot2_client.wait_for_result()

    # After executing the linear motions,
    # go to the R2Place robot pose using the set_named_target API.
    robot2_group.set_named_target("R2Place")

    _, plan, _, _ = robot2_group.plan()
    robot2_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot2_goal.trajectory = plan
    rospy.loginfo('Go to Place')
    robot2_client.send_goal(robot2_goal)
    robot2_client.wait_for_result()

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        week4_assignment3()
    except rospy.ROSInterruptException:
        pass
