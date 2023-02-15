#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32
from ppl_interfaces.msg import ReceiveParts

# Workstation
#   msg/
#       receive_parts:
#           inputs: part_name, num_parts
#           outputs: parts_remaining, estimated_time_to_empty
#
#   topic/
#       "part_name"/parts_remaining
#       estimated_time_to_empty
#       completed_assemblies
#       failed_assemblies
#       estimated_time_to_full_completed
#       estimated_time_to_full_failed

part_names = ["top_shell", "main_shell", "insert_mold"]

class WorkstationStateNode(Node):
    def __init__(self):
        super().__init__('workstation_state_node')
        # dictionary with key as part type and value as number of remaining parts left
        self.parts_remaining                  = {}
        for part in part_names:
            self.parts_remaining[part]        = 0

        # number of complete parts that can be assembled. Need at least one of each type
        self.assembly_success_prob            = 95 # 95%
        self.part_grasp_success_prob          = 95 # 95%
        self.full_assemblies_remaining        = 0
        self.estimated_time_to_empty          = 0.0
        self.completed_assemblies             = 0
        self.failed_assemblies                = 0
        self.estimated_time_to_full_completed = 0.0
        self.estimated_time_to_full_failed    = 0.0

        # random assembly time with mean of 5s
        self.time_per_assembly                = np.random.normal(5.0, 4.9)
        # print(self.time_per_assembly)
        self.assembly_bin_max                 = 0.0
        
        # temporary timer to simulate workstation execution
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.assembly_timer = self.create_timer(self.time_per_assembly, self.assembly_callback)

        self.estimated_time_to_empty_pub          = self.create_publisher(Float32, 'estimated_time_to_empty', 10)
        self.completed_assemblies_pub             = self.create_publisher(Int32, 'completed_assemblies', 10)
        self.failed_assemblies_pub                = self.create_publisher(Int32, 'failed_assemblies', 10)
        self.estimated_time_to_full_completed_pub = self.create_publisher(Float32, 'estimated_time_to_full_completed', 10)
        self.estimated_time_to_full_failed_pub    = self.create_publisher(Float32, 'estimated_time_to_full_failed', 10)
        self.full_assemblies_remaining_pub        = self.create_publisher(Int32, 'full_assemblies_remaining', 10)

        self.parts_remaining_pubs                 = {}
        for part in part_names:
          self.parts_remaining_pubs[part]         = self.create_publisher(Int32, part + '/parts_remaining', 10)

        self.receive_parts_sub                    = self.create_subscription(ReceiveParts, 'receive_parts', self.receive_parts_callback, 1)

    def receive_parts_callback(self, msg):

      # check the part type exists in the set of parts
      if(msg.part_type in part_names):
        # update parts_remaining for the part type
        self.parts_remaining[msg.part_type] += msg.num_parts

      # update the number of full parts remaining ()
      self.full_assemblies_remaining = min(self.parts_remaining.values())

      self.estimated_time_to_empty = self.time_per_assembly * self.full_assemblies_remaining

      print("RECEVIED PARTS!\n\tEstimated Time to Empty:"+str(self.estimated_time_to_empty))

    def assembly_callback(self):
      # check if there are any full parts remaining to be assembled
      if not self.full_assemblies_remaining == 0:
        # compute grasp success prob for each part
        grasp_probs = np.random.randint(0, 100, len(self.parts_remaining.keys()))

        grasp_fails = []
        [grasp_fails.append(i) for i in range(0, len(part_names)) if grasp_probs[i] > self.part_grasp_success_prob]

        # check if there are any grasp failures
        success = len(grasp_fails) == 0

        # if no grasp failures
        if success:
          # decrement the number of parts remaining for each part
          for part in part_names:
            self.parts_remaining[part] -= 1

          # determine if assembly succeeds according to a success probability
          if np.random.randint(0, 100,) < self.assembly_success_prob:
            self.completed_assemblies += 1
          else:
            self.failed_assemblies += 1
        else:
          # else decrement the part that failed to be grasped
          # Assume it was dropped out of reach. Could be modified in future to have any
          # behavior if desired
          for idx in grasp_fails:
            self.parts_remaining[part_names[idx]] -= 1

        # compute full assemblies remaining from the individual parts remaining
        self.full_assemblies_remaining = min(self.parts_remaining.values())

        # compute the estimated time until empty for full assemblie
        self.estimated_time_to_empty = self.time_per_assembly * self.full_assemblies_remaining

        # compute the estimated time until completed assemblies bin needs to be replaced
        # time_per_assembly * num_parts_until_full * prob_of_success
        self.estimated_time_to_full_completed = self.time_per_assembly * (self.assembly_bin_max - self.completed_assemblies) / self.assembly_success_prob

        self.estimated_time_to_full_failed = self.time_per_assembly * (self.assembly_bin_max - self.failed_assemblies) / (1.0 - self.assembly_success_prob)

    def timer_callback(self):
      # parts_remaining msg for each part
      parts_msg = Int32()
      for part in part_names:
        parts_msg.data = self.parts_remaining[part]
        self.parts_remaining_pubs[part].publish(parts_msg)
      
      # full_assemblies msg for number of full assemblies remaining
      full_assemblies_remaining_msg = Int32()
      full_assemblies_remaining_msg.data = self.full_assemblies_remaining
      self.full_assemblies_remaining_pub.publish(full_assemblies_remaining_msg)


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
