from ppl_interfaces.msg import PPLPose, GroupPose
from coordinated_commander.group_navigator import GroupNavigator

import rclpy
from rclpy.duration import Duration

import time
import argparse
import math

def main(args=None):

  # Print warnings and assumptions
  print("This script is expecting a ppl group .rdmp.path file for planar mobile robots.")
  print("The names given as input arguments should correspond to the sequence of DOFs in the path file.")
  print("Each robot is assumed to have 3 DOFs per configuration (x,y,theta).")

  # Parse arguments
  parser = argparse.ArgumentParser(description='Set robots to ppl group path')
  parser.add_argument('-ns','--robot_namespaces',type=str, default='',
                      help='Namespaces of robots seprated by spaces (inside quotations)')
  parser.add_argument('-p','--path_file',type=str,default='',
                      help='File path to the pathfile to follow')

  args,unknown = parser.parse_known_args()

  # Convert names into list of namespaces
  namespaces = args.robot_namespaces.split(' ')

  # Convert path file into sequence of group poses
  path_file = open(args.path_file,'r')
  lines = path_file.readlines()

  group_poses = []

  for line in lines:
    if line.startswith('VIZMO_PATH_FILE'):
      continue

    raw = line.split(' ')
    dofs = []
    for elem in raw:
      if elem != '' and elem != '\n':
        dofs.append(elem)

    if len(dofs) == 1:
      continue

    poses = []
    data = []
    for index in range(len(dofs)):
      dof = dofs[index]
      if index == 0:
        continue

      data.append(dof)

      if (index)%3 == 0:
        # Reached end of robot DOFs
        # Convert to pose
        pose = PPLPose()
        pose.x = float(data[0])
        pose.y = float(data[1])
        pose.z = math.sin(float(data[2])*3.14/2)-0.01
        pose.w = math.cos(float(data[2])*3.14/2)-0.01

        # Add to set of poses
        poses.append(pose)
        data = []

    # Reach end of group configuration
    # Convert to GroupPose
    group_pose = GroupPose()
    group_pose.namespaces = namespaces
    group_pose.poses = poses

    # Add to sequence of group poses
    group_poses.append(group_pose)
      

  # Initialize ROS2
  rclpy.init()

  # Initialize group navigator
  group_nav = GroupNavigator(namespaces)

  # Set initial pose to first group pose in the sequence
  group_nav.setInitialPose(group_poses[0])

  group_nav.waitUntilNav2Active()

  # Send robots through the sequence of group poses
  index = 1
  next_index = index

  while index < len(group_poses):
    if index == next_index:
      goal = group_poses[index]
      group_nav.goToPose(goal)
      next_index = next_index + 1
      time.sleep(1)

    elif group_nav.isTaskComplete():
      index = index+1
  
