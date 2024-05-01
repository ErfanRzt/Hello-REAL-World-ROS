#! /usr/bin/env python3
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
from __future__ import print_function
import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the CounterWithDelay action, including the
# goal message and the result message.
from hrwros_msgs.msg import CounterWithDelayAction, CounterWithDelayGoal


def counter_with_delay_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (CounterWithDelayAction) to the constructor.
    client = actionlib.SimpleActionClient('counter_with_delay', CounterWithDelayAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    num_counts = 3

    # Creates a goal to send to the action server.
    goal = CounterWithDelayGoal(num_counts)

    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.loginfo("Goal has been sent to the action server.")

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Does something else while the action is being done:
    for count_idx in range(0, num_counts):
        rospy.loginfo('I am doing other things while the goal is being serviced by the server')
        rospy.sleep(1.2)

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('counter_with_delay_ac')
        result = counter_with_delay_client()
        rospy.loginfo(result.result_message)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
