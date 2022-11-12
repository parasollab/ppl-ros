import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped

from coordinated_commander.robot_navigator import NamespaceNavigator
from nav2_simple_commander.robot_navigator import BasicNavigator

class AGVAgent(Node):

  def __init__(self,namespace=''):
    super().__init__('agv_agent')

    self.namespace = namespace
    self.task_queue = []
    self.reached_goal = True
    self.goal_index = 0

    if self.namespace == '':
      self.prefix = ''
    else:
      self.prefix = namespace + '/'

    # Task queue subscriber
    self.task_queue_sub = self.create_subscription(
                              PoseStamped,
                              self.prefix + 'task_queue',
                              self._addTaskCallback,
                              QoSProfile(
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST,
                                depth=1
                              )
                          )


    if self.namespace == '':
      self.navigator = BasicNavigator()
    else:
      self.navigator = NamespaceNavigator(self.namespace)


  def _addTaskCallback(self,task_pose):
    self.task_queue.append(task_pose)

    if self.reached_goal:
      self.executeTasks()

  def executeTasks(self):
      while self.goal_index < len(self.task_queue):
        self.navigator.goToPose(self.task_queue[self.goal_index])
        while not self.navigator.isTaskComplete():
          print('Moving to task',self.task_queue[self.goal_index])

        self.goal_index = self.goal_index + 1
    

def main(args=None):
  print('Hi from agv_agent')
  
  rclpy.init(args=args)

  agv_agent = AGVAgent()

  rclpy.spin(agv_agent)

  rclpy.shutdown()

    


if __name__ == '__main__':
    main()
