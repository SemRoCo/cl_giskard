cmake_minimum_required(VERSION 2.8.3)
project(cl_giskard)

find_package(PkgConfig)
find_package(catkin REQUIRED COMPONENTS
  giskard_msgs
  std_msgs
  message_generation
  cl_tf
  cl_tf2
  cl_transforms_stamped
  roslisp
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS roslisp cl_tf cl_tf2 cl_transforms_stamped std_msgs giskard_msgs message_runtime
)

