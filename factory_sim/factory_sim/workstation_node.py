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

class WorkstationStateNode(Node):
    def __init__(self):
        super().__init__('workstation_state_node')
        self.parts_remaining                  = 0
        self.estimated_time_to_empty          = 0.0
        self.completed_assemblies             = 0
        self.failed_assemblies                = 0
        self.estimated_time_to_full_completed = 0.0
        self.estimated_time_to_full_failed    = 0.0
        self.time_per_assembly                = 0.0
        self.completed_threshold              = 0.0
        
        # temporary timer to simulate workstation execution
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.parts_remaining_pub                  = self.create_publisher(Int32, 'parts_remaining', 10)
        self.estimated_time_to_empty_pub          = self.create_publisher(Float32, 'estimated_time_to_empty', 10)
        self.completed_assemblies_pub             = self.create_publisher(Int32, 'completed_assemblies', 10)
        self.failed_assemblies_pub                = self.create_publisher(Int32, 'failed_assemblies', 10)
        self.estimated_time_to_full_completed_pub = self.create_publisher(Float32, 'estimated_time_to_full_completed', 10)
        self.estimated_time_to_full_failed_pub    = self.create_publisher(Float32, 'estimated_time_to_full_failed', 10)
        self.receive_parts_service                = self.create_service(ReceiveParts, 'receive_parts', self.receive_parts_callback)

    def receive_parts_callback(self, request, response):
        # update parts_remaining and estimated time to empty
        self.parts_remaining += request.num_new_parts
        
        self.time_per_assembly = request.time_per_assembly
        self.completed_threshold = request.completed_threshold

        self.estimated_time_to_empty = self.time_per_assembly * self.parts_remaining

        return response

    def timer_callback(self):
      # parts_remaining msg
      parts_msg = Int32()
      parts_msg.data = self.parts_remaining
      self.parts_remaining_pub.publish(parts_msg)
      
      # estimated_time_to_empty msg
      estimated_time_to_empty_msg = Float32()
      estimated_time_to_empty_msg.data = self.estimated_time_to_empty
      self.estimated_time_to_empty_pub.publish(estimated_time_to_empty_msg)

      # completed_assemblies msg
      completed_assemblies_msg = Int32()
      completed_assemblies_msg.data = self.completed_assemblies
      self.completed_assemblies_pub.publish(completed_assemblies_msg)

      # failed_assemblies msg
      failed_assemblies_msg = Int32()
      failed_assemblies_msg.data = self.failed_assemblies
      self.failed_assemblies_pub.publish(failed_assemblies_msg)

      # estimated_time_to_full_completed msg
      estimated_time_to_full_completed_msg = Float32()
      estimated_time_to_full_completed_msg.data = self.estimated_time_to_full_completed
      self.estimated_time_to_full_completed_pub.publish(estimated_time_to_full_completed_msg)

      # estimated_time_to_full_failed msg
      estimated_time_to_full_failed_msg = Float32()
      estimated_time_to_full_failed_msg.data = self.estimated_time_to_full_failed
      self.estimated_time_to_full_failed_pub.publish(estimated_time_to_full_failed_msg)

          

def main(args=None):
  print('Hi from factory_workstation.')
  
  rclpy.init(args=args)

  workstation_state_node = WorkstationStateNode()

  rclpy.spin(workstation_state_node)

  rclpy.shutdown()

    


if __name__ == '__main__':
    main()
