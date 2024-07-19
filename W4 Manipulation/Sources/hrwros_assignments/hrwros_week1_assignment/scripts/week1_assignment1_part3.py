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
# reading from the reference frame of the sensor to compute the height of a
# box based on the illustration shown in the assignment document. Then, you
# will publish the box height on a new message type ONLY if the height of the
# box is more than 10cm.

# All necessary python imports go here.
import rospy
# Copy the code from Part1 here
from hrwros_msgs.msg import SensorInformation
# Import the correct message type for part 3
from hrwros_week1_assignment.msg import BoxHeightInformation


def sensor_info_callback(data, bhi_publisher):

    # Copy the code from Part1 here
    height_box = 2.0 - data.sensor_data.range

    # Compute the height of the box.
    # Boxes that are detected to be shorter than 10cm are due to sensor noise.
    # Do not publish information about them.
    # Copy the code from Part1 here
    if height_box < 0.1:
        pass
    else:
        # Declare a message object for publishing the box height information.
        box_height_info = BoxHeightInformation()
        # Update height of box.
        box_height_info.box_height = height_box
        # Publish box height using the publisher argument passed to the
        # callback function.
        bhi_publisher.publish(box_height_info)


if __name__ == '__main__':

    # Initialize the ROS node here.
    rospy.init_node('compute_box_height', anonymous=False)

    # Copy the code from Part1 here
    rospy.loginfo('Waiting for topic %s to be published...', '/sensor_info')
    rospy.wait_for_message('/sensor_info', SensorInformation)
    rospy.loginfo('%s topic is now available!', '/sensor_info')

    # Create the publisher for Part3 here
    bhi_publisher = rospy.Publisher('/box_height_info', BoxHeightInformation, queue_size=10)

    # Note here that an ADDITIONAL ARGUMENT (bhi_publisher) is passed to the
    # subscriber. This is a way to pass ONE additional argument to the
    # subscriber callback. If you want to pass multiple arguments, you can use
    # a python dictionary. And if you don't want to use multiple arguments to
    # the subscriber callback then you can also consider using a Class
    # Implementation like we saw in the action server code illustration.

    # Copy the subscriber from Part1 here
    rospy.Subscriber('/sensor_info', SensorInformation, sensor_info_callback, bhi_publisher)

    # Prevent this code from exiting until Ctrl+C is pressed.
    rospy.spin()
