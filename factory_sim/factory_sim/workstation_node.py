#!/usr/bin/env python3
from ppl_interfaces.srv import ReceiveParts

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32

# Workstation
#   srv/
#       receive_parts:
#           inputs: time per assembly, num new parts, namespace, completed/failed assembly threshold
#           outputs: update parts_remaining, estimated_time_to_empty
#
#   topic/
#       parts_remaining
#       estimated_time_to_empty
#       completed_assemblies
#       failed_assemblies
#       estimated_time_to_full_completed
#       estimated_time_to_full_failed

parts_remaining = 10
estimated_time_to_empty = 10.0
completed_assemblies = 0
failed_assemblies = 0
estimated_time_to_full_completed = 10.0
estimated_time_to_full_failed = 1000.0

class WorkstationStateNode(Node):
    def __init__(self):
        super().__init__('workstation_state_node')
        self.parts_remaining_pub                  = self.create_publisher(Int32, 'parts_remaining', 10)
        self.estimated_time_to_empty_pub          = self.create_publisher(Float32, 'estimated_time_to_empty', 10)
        self.completed_assemblies_pub             = self.create_publisher(Int32, 'completed_assemblies', 10)
        self.failed_assemblies_pub                = self.create_publisher(Int32, 'failed_assemblies', 10)
        self.estimated_time_to_full_completed_pub = self.create_publisher(Float32, 'estimated_time_to_full_completed', 10)
        self.estimated_time_to_full_failed_pub    = self.create_publisher(Float32, 'estimated_time_to_full_failed', 10)
        self.receive_parts_service                = self.create_service(ReceiveParts, 'receive_parts', self.receive_parts_callback)

    # def workstation_state_callback(self):
    #   # parts_remaining_msg = 
    #   pass

    def receive_parts_callback(self, request, response):
        return response
      

def main(args=None):
  print('Hi from factory_workstation.')
  
  rclpy.init(args=args)

  workstation_state_node = WorkstationStateNode()

  rclpy.spin(workstation_state_node)

  rclpy.shutdown()

    


if __name__ == '__main__':
    main()
