import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped

def main():
  rclpy.init()

  robot1 = 'robot1'

  initial_pose = PoseStamped()
  initial_pose.header.frame_id = 'origin'
  initial_pose.pose.position.x = 0.0
  initial_pose.pose.position.y = 0.0
  initial_pose.pose.orientation.z = 1.0
  initial_pose.pose.orientation.w = 0.0

  
  pose1 = PoseStamped()
  pose1.header.frame_id = 'take_from_depot'
  pose1.pose.position.x = 8.0
  pose1.pose.position.y = 0.5
  pose1.pose.orientation.z = 1.0
  pose1.pose.orientation.w = 0.0


  pose2 = PoseStamped()
  pose2.header.frame_id = 'ws0/receive_parts'
  pose2.pose.position.x = 4.0
  pose2.pose.position.y = 3.0
  pose2.pose.orientation.z = 1.0
  pose2.pose.orientation.w = 0.0


  pose3 = PoseStamped()
  pose3.header.frame_id = 'ws3/receive_parts'
  pose3.pose.position.x = -4.0
  pose3.pose.position.y = -3.0
  pose3.pose.orientation.z = 1.0
  pose3.pose.orientation.w = 0.0

  node = Node('dummy_allocator')
  pub = node.create_publisher(PoseStamped,robot1 + '/task_queue',

                              QoSProfile(
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST,
                                depth=1
                              )
  )
  pub.publish(initial_pose)
  pub.publish(pose1)
  pub.publish(pose2)
  pub.publish(pose3)
