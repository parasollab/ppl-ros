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
    self.ws_service_map = defaultdict(str)
    with open(self.mapping_file) as file:
      msg_type = None
      for line in file:
        # If line is empty skip
        if line.strip() == '':
          continue
        # Get type mapping as string
        if line.strip() in str_type_mappings.keys():
          msg_type = line.strip()
        # add topic to reboradcast map using type mapping
        else:
          l = line.strip()
          self.rebroadcast_map[msg_type].append(l)

          # Check if topic is for a workspace
          ws_part = str(l).split('/')[1]
          if str(ws_part).startswith('ws'):
            # if topic is for a workspace add to set of 
            # workspaces to track which robot is servicing it
            if str(ws_part) not in self.ws_service_map:
              self.ws_service_map[str(ws_part)] = 'none'


    self.remap_subscribers = {}
    self.remap_publishers  = {}
    # setup rebroadcast subscribers and publishers
    for key in str_type_mappings:
      topics = self.rebroadcast_map[key]

      for topic in topics:
        from_type = rostopic.get_topic_class(topic)[0]

        self.remap_subscribers[topic] = rospy.Subscriber(topic, from_type, callback=self.callback, callback_args=(topic, key), queue_size=10)
        
        if not str(topic).startswith('/robot'):
          self.remap_publishers[topic]  = rospy.Publisher('/gui/'+ key + topic, str_type_mappings[key], queue_size=10)

    # create publisher for workstation name and servicing robot
    for key in self.ws_service_map:
      self.remap_publishers[key] = rospy.Publisher('/gui/text/' + key + '/servicing_robot', OverlayText, queue_size=10)


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
    
    if msg_type == PoseStamped:
      robot = str(topic).split("/")[1]
      new_msg = str_type_mappings[type]
      if str(msg_info) in self.remap_publishers.keys():
        pub = self.remap_publishers[str(msg_info)]
        # TODO: reset pub to be /ws*/servicing_robot
        new_msg.text = """%s
                        Servicing Robot: %s
                        """ % (str(msg_info), str(robot))
        pub.publish(new_msg)
      else:
        for key in self.ws_service_map.keys():
          print(str(key))
          pub = self.remap_publishers[str(key)]
          new_msg.text = """%s
                        Servicing Robot: none
                        """ % (str(key))
          pub.publish(new_msg)
    else:
      pub = self.remap_publishers[topic]

      new_msg = str_type_mappings[type]
      if new_msg is Float32:
        # print("\nFloat32")
        new_msg = str_type_mappings[type]()
        new_msg.data = msg_info
      elif new_msg is OverlayText:
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