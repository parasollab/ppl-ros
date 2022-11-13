from launch import LaunchDescription
from launch_ros.actions import Node

num_workstations = 4

def generate_launch_description():
  ld = LaunchDescription()
  for i in range(0, num_workstations):
    ld.add_action(
      Node(
        package='factory_sim',
        namespace='ws'+str(i),
        executable='workstation_node',
        name='workstation'
      )
    )
  return ld
