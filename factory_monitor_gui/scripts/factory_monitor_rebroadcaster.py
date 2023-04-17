#!/usr/bin/env python3

import sys
import os
cwd = os.getcwd()

# Python imports
from collections import defaultdict

# ROS imports
import rospy
import rospkg
import rostopic

# Message imports
from std_msgs.msg import *
from geometry_msgs.msg import *
from jsk_rviz_plugins.msg import *


type_str_mappings = {
  'std_msgs/Float32': Float32,
  'geometry_msgs/PoseStamped': PoseStamped
}

str_type_mappings = {
  'c_gauge': Float32,
  'l_gauge': Float32,
  'plotter': Float32,
  'text': OverlayText
}
##################
# On construction, read in list of topics to rebroadcast
# Store mapping of orignial topics to new topic publisher (1 callback function, multiple subscribers)

class Rebroadcaster():
  def __init__(self, mapping_file):
    self.rate = rospy.Rate(10)
    # Read in topics to be mapped
    rospack = rospkg.RosPack()

    # Get the mapping file
    self.mapping_file = rospack.get_path('factory_monitor_gui') + '/config/' + str(mapping_file)

    # Create emtpy map to hold topic rebroadcast info
    # {(from_type, to_type): [topic, ...], ...}
    self.rebroadcast_map = defaultdict(list)
    with open(self.mapping_file) as file:
      msg_type = None
      for line in file:
        # If line is empty skip
        if line.strip() == '':
          continue       
        if line.strip() in str_type_mappings.keys():
          msg_type = line.strip()
        else:
          self.rebroadcast_map[msg_type].append(line.strip())

    self.remap_subscribers = {}
    self.remap_publishers  = {}
    for key in str_type_mappings:
      topics = self.rebroadcast_map[key]

      for topic in topics:
        from_type = rostopic.get_topic_class(topic)[0]

        self.remap_subscribers[topic] = rospy.Subscriber(topic, from_type, callback=self.callback, callback_args=(topic, key), queue_size=10)
        self.remap_publishers[topic]  = rospy.Publisher('/gui/'+ key + topic, str_type_mappings[key], queue_size=10)

  def run(self):
    while not rospy.is_shutdown():
      self.rate.sleep()
  
  # currently only std_msg types are supported
  def callback(self, msg, mapping_info):
    msg_info = None
    msg_type = type_str_mappings[msg._type]
    topic = mapping_info[0]
    type = mapping_info[1]
    if(msg_type == Float32 or msg_type == Int32):
      msg_info = msg.data
    elif(msg_type == PoseStamped):

      msg_info = msg.header.frame_id
    else:
      return
    
    pub = self.remap_publishers[topic]

    new_msg = str_type_mappings[type]
    if new_msg is Float32:
      # print("\nFloat32")
      new_msg = str_type_mappings[type]()
      new_msg.data = msg_info
    elif new_msg is OverlayText:
      # print("\nOverlayText")
      new_msg = str_type_mappings[type]()
      new_msg.text = topic + ': ' + str(msg_info)
    else:
      # print("Unknown Message Type")
      return

    pub.publish(new_msg)
    return

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print("usage: factory_monitor_rebroadcaster.py filename")
  else:
    try:
      rospy.init_node("rebroadcast_node")
      node = Rebroadcaster(sys.argv[1])
      node.run()
    except rospy.ROSInterruptException:
      pass