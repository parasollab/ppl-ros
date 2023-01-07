from ppl_interfaces.msg import GroupPose, PPLPose
from coordinated_commander.robot_navigator import NamespaceNavigator, TaskResult

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GroupNavigator(Node):

  def __init__(self,namespaces):
    super().__init__(node_name='group_navigator')

    self.navigators = {}
    for namespace in namespaces:
      self.navigators[namespace] = NamespaceNavigator(namespace)

    group_pose_qos = QoSProfile(
      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
      reliability=QoSReliabilityPolicy.RELIABLE,
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=10
    )
    self.group_pose_sub = self.create_subscription(GroupPose,
                                                   'group_pose',
                                                   self._receivePoseCallback,
                                                   group_pose_qos
                                                   )
  def waitUntilNav2Active(self):
    for namespace,nav in self.navigators.items():
      nav.waitUntilNav2Active()
    return

  def setInitialPose(self, initial_pose):
    self.initial_pose_received = False
    self.initial_pose = initial_pose
    self._setInitialPose()

  def goToPose(self,pose,sync=True):
    
    if not sync:
      self.error("Non synchronized go to pose not implemented")
      return False

    # convert to pose stamped and send to navigators
    for index in range(len(pose.namespaces)):
      namespace = pose.namespaces[index]
      ppl_pose = pose.poses[index]
      nav = self.navigators[namespace]

      # TODO::Set speed

      msg = self.pplPoseToPoseStamped(nav,ppl_pose)
      nav.goToPose(msg)
    

  def followWaypoints(self, poses, sync=True):
    
    if not sync:
      self.error("Non synchronized waypoint following not implemented")
      return False

  def isTaskComplete(self):
    for namespace,nav in self.navigators.items():
      if not nav.isTaskComplete():
        return False

    return True

  def pplPoseToPoseStamped(self,nav,pplPose):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = pplPose.x
    pose.pose.position.y = pplPose.y
    pose.pose.orientation.z = pplPose.z
    pose.pose.orientation.w = pplPose.w
    return pose

  def _receivePoseCallback(self, msg):
    print(msg)

  def _setInitialPose(self):
    for index in range(len(self.initial_pose.namespaces)):
      namespace = self.initial_pose.namespaces[index]
      ppl_pose = self.initial_pose.poses[index]
      nav = self.navigators[namespace]
      pose = self.pplPoseToPoseStamped(nav,ppl_pose)
      nav.setInitialPose(pose)

