#! /usr/bin/env python
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

# Assignment 1 for Week1: In this assignment you will subscribe to the topic
# that publishes sensor information. Then, you will transform the sensor
# reading from the reference frame of the sensor to compute the height of
# a box based on the illustration shown in the assignment document.
# Then, you will publish the box height on a new message type ONLY if the
# height of the box is more than 10cm.

# All necessary python imports go here.
import rospy

# Import the correct type of message
from hrwros_msgs.msg import SensorInformation


def sensor_info_callback(data):

    # Compute the height of the box from the sensor reading.
    # Tip: You need to substract the reading from the max_range of the sensor.
    height_box = 2.0 - data.sensor_data.range

    # Compute the height of the box.
    # Boxes that are detected to be shorter than 0.1m are due to sensor noise.
    # Do not publish information about them.
    if height_box < 0.1:
        pass
    else:
        # If imlemented correctly only the height of boxes bigger than 0.1m
        # will be printed
        rospy.loginfo('Height of box %0.3f' % height_box)


if __name__ == '__main__':
    # Initialize the ROS node here.
    rospy.init_node('compute_box_height', anonymous=False)

    # Wait for the topic that publishes sensor information to become available - Part1
    rospy.loginfo('Waiting for topic %s to be published...', '/sensor_info')
    rospy.wait_for_message('/sensor_info', SensorInformation)
    rospy.loginfo('%s topic is now available!', '/sensor_info')

    # Create the subscriber for Part1 here
    rospy.Subscriber('/sensor_info', SensorInformation, sensor_info_callback)

    # Prevent this code from exiting until Ctrl+C is pressed.
    rospy.spin()
