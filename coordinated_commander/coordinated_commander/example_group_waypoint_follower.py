from ppl_interfaces.msg import PPLPose, GroupPose
from coordinated_commander.group_navigator import GroupNavigator
#from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

import time
"""
Basic navigation demo to go to poses.
"""

initial_poses = {
  'robot1':{
            'x':0.0,
            'y':0.5,
            'z':1.0,
            'w':0.0
           },
  'robot2':{
            'x':-2.0,
            'y':0.5,
            'z':1.0,
            'w':0.0
           }
}

goal_poses1 = {
  'robot1':{
            'x':1.5,
            'y':0.5,
            'z':0.707,
            'w':0.707
           },
  'robot2':{
            'x':0.0,
            'y':0.5,
            'z':0.707,
            'w':0.707
           }
}

goal_poses2 = {
  'robot1':{
            'x':1.5,
            'y':-3.75,
            'z':0.707,
            'w':0.707
           },
  'robot2':{
            'x':0.0,
            'y':-5.75,
            'z':0.707,
            'w':0.707
           }
}

def dictToGroupPose(dictionary):
  group_pose = GroupPose()

  namespaces = []
  individual_poses = []

  for robot,pose in dictionary.items():
    namespaces.append(robot)
    pplPose = PPLPose()
    pplPose.x = pose['x']
    pplPose.y = pose['y']
    pplPose.z = pose['z']
    pplPose.w = pose['w']
    individual_poses.append(pplPose)

  group_pose.namespaces = namespaces
  group_pose.poses = individual_poses

  return group_pose

def main():
  rclpy.init()

  namespaces = []
  for robot in initial_poses:
    namespaces.append(robot)

  gn = GroupNavigator(namespaces)

  # Create and set initial pose
  initial = dictToGroupPose(initial_poses)
  gn.setInitialPose(initial)

  gn.waitUntilNav2Active()

  # Send robots through goal poses

  goals = [
    dictToGroupPose(goal_poses1),
    dictToGroupPose(goal_poses2)
  ]

  next_index = 0
  index = 0
  while index < len(goals):
    if index == next_index:
      goal = goals[index]
      gn.goToPose(goal)
      next_index = next_index + 1
      time.sleep(1)

    elif gn.isTaskComplete():
      index = index + 1

  
