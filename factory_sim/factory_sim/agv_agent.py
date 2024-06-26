import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from ppl_interfaces.msg import ReceiveParts

from coordinated_commander.robot_navigator import NamespaceNavigator
#from nav2_simple_commander.robot_navigator import BasicNavigator

from std_msgs.msg import String,Int32, Bool

import time

class AGVAgent(Node):

  def __init__(self,namespace='',x=0,y=0):
    super().__init__('agv_agent')

    self.namespace = namespace
    self.task_queue = []
    self.reached_goal = True
    self.goal_index = 0
    self.goal_counter = 0

    if self.namespace == '':
      self.prefix = ''
    else:
      self.prefix = namespace + '/'

    # Task queue subscriber
    self.task_queue_sub = self.create_subscription(
                              PoseStamped,
                              self.prefix + 'task_queue',
                              self._addTaskCallback,
                              1000
                          )
    
    self.cur_task_pub = self.create_publisher(
                                PoseStamped,
                                self.prefix + 'current_task',
                                1000
                            )
    
    self.dump_pub = self.create_publisher(
                          String,
                          self.prefix + 'dump',
                          1
                        )
    
    self.task_complete_pub = self.create_publisher(Bool,'/robot1/task_complete',1)

    
    # self.task_complete_pub = self.create_publisher(
    #                       String,
    #                       self.prefix + 'task_complete',
    #                       1
    # )

    #if self.namespace == '':
    self.navigator = NamespaceNavigator(self.namespace)
    #else:
      #self.navigator = NamespaceNavigator(self.namespace)

    #msg = PoseStamped()
    #msg.header.frame_id = 'map'
    #msg.header.stamp = self.navigator.get_clock().now().to_msg()
    #msg.pose.position.x = x
    #msg.pose.position.y = y
    #msg.pose.orientation.z = 1.0
    #msg.pose.orientation.w = 0.0

    #self.navigator.setInitialPose(msg)

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    #initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x
    initial_pose.pose.position.y = y
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0

    print('Setting initial pose',initial_pose)

    self.navigator.setInitialPose(initial_pose)
    self.navigator.waitUntilNav2Active()

    cur_task_msg = PoseStamped()
    cur_task_msg.header.frame_id = 'none'
    self.cur_task_pub.publish(cur_task_msg)

    print('Done initializing')

  def _addTaskCallback(self,task_pose):
    self.task_queue.append(task_pose)

    print('Received Task:\n',task_pose)

    if self.reached_goal:
      self.executeTasks()

  def executeTasks(self):

      #rate = self.create_rate(1)

      self.reached_goal = False

      while self.goal_index < len(self.task_queue):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = self.task_queue[self.goal_index].pose
        self.navigator.goToPose(msg)

        cur_task_msg = PoseStamped()
        cur_task_msg.header.frame_id = self.task_queue[self.goal_index].header.frame_id
        cur_task_msg.pose = self.task_queue[self.goal_index].pose

        counter = 0
        while not self.navigator.isTaskComplete():
          counter = counter + 1
          self.cur_task_pub.publish(cur_task_msg)
          if counter % 100 == 0:
            print(self.navigator.isTaskComplete())

        self.complete_task(self.task_queue[self.goal_index])
        self.goal_index = self.goal_index + 1
      
      cur_task_msg = PoseStamped()
      cur_task_msg.header.frame_id = 'none'
      self.cur_task_pub.publish(cur_task_msg)

      # self.reached_goal = True

  def timer_callback(self):
    print('elapsed time: ', time.time() - self.tic)
    # topic = '/robot1/task_complete'

    msg = Bool()
    msg.data = True
    self.task_complete_pub.publish(msg)
    self.destroy_timer(self.timer)
    self.timer = None
    self.reached_goal = True

    # if self.goal_index < len(self.task_queue):
    #   self.executeTasks()

  def complete_task(self,task):

    print('COMPLETING TASK')
    self.goal_counter += 1

    if self.goal_counter != 0 and self.goal_counter % 2 == 0:
      msg = String()
      msg.data = 'dump'
      self.dump_pub.publish(msg)

      self.timer = self.create_timer(10, self.timer_callback)
      self.tic = time.time()
    else:
        self.reached_goal = True

      # self.create_rate(0.2).sleep()
    # print('done waiting')
    # topic = self.prefix + 'task_complete'
    # # # grap topic from task
    # task_complete_pub = self.create_publisher(Bool,topic,1)

    # msg = Bool()
    # msg.data = True
    # task_complete_pub.publish(msg)
    # task_complete_pub = self.create_publisher(ReceiveParts, topic, 5)

    # msg = ReceiveParts()
    # msg.part_type = "top_shell"
    # msg.num_parts = 50
    # task_complete_pub.publish(msg)

    # msg.part_type = "main_shell"
    # task_complete_pub.publish(msg)

    # msg.part_type = "insert_mold"
    # task_complete_pub.publish(msg)
    
    # self.get_logger().info('Publishing: "%s", "%i" pcs' % (msg.part_type, msg.num_parts))
    return
  
  def run(self):
    while(rclpy.ok()):
      if self.reached_goal and self.goal_index < len(self.task_queue):
        print('STARTING NEXT TASK')
        self.executeTasks()
      rclpy.spin_once(self)
    

def main(args=None):
  print('Hi from agv_agent')
  
  rclpy.init(args=args)

  parser = argparse.ArgumentParser(description='Read in filenames')
  parser.add_argument('-x',help='Starting pose x value.')
  parser.add_argument('-y',help='Starting pose y value.')
  parser.add_argument('--name',help='Agent name.')
  args = parser.parse_args()

  x = args.x
  if x != None:
    x = float(x)

  y = args.y
  if y != None:
    y = float(y)

  name = args.name
  if args.name == None:
    name = ''

  print(name,x,y)

  agv_agent = AGVAgent(namespace=name,x=x,y=y)

  agv_agent.run()

  # rclpy.spin(agv_agent)

  rclpy.shutdown()

    


if __name__ == '__main__':
    main()
