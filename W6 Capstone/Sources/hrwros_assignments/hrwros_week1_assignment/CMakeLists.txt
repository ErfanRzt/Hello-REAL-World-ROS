cmake_minimum_required(VERSION 3.0.2)
project(hrwros_week1_assignment)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  hrwros_msgs
)


add_message_files(DIRECTORY msg
  FILES
  ## Add the file name for A1 part 2 here ##
  BoxHeightInformation.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hrwros_week1
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
