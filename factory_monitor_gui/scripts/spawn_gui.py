#!/usr/bin/env python3

import sys
import os
cwd = os.getcwd()

# Python imports
from collections import defaultdict
import shutil
import fileinput
import copy

# ROS imports
import rospy
import rospkg
import rostopic

# Message imports
from std_msgs.msg import *
from geometry_msgs.msg import *
from jsk_rviz_plugins.msg import *

c_gauge_text = '    - Class: jsk_rviz_plugin/PieChart\n\
      Enabled: true\n\
      Name: {name}\n\
      Topic: {topic}\n\
      Value: true\n\
      auto color change: false\n\
      background color: 0; 0; 0\n\
      backround alpha: 0\n\
      clockwise rotate direction: false\n\
      foreground alpha: 0.699999988079071\n\
      foreground alpha 2: 0.4000000059604645\n\
      foreground color: 25; 255; 240\n\
      left: {left}\n\
      max color: 255; 0; 0\n\
      max color change threthold: 0\n\
      max value: 1\n\
      med color: 255; 0; 0\n\
      med color change threthold: 0\n\
      min value: 0\n\
      show caption: true\n\
      size: {height}\n\
      text size: {text_size}\n\
      top: {top}\n'

overlay_text = '    - Align Bottom: false\n\
      Background Alpha: 0.800000011920929\n\
      Background Color: 0; 0; 0\n\
      Class: jsk_rviz_plugin/OverlayText\n\
      Enabled: true\n\
      Foreground Alpha: 0.800000011920929\n\
      Foreground Color: 25; 255; 240\n\
      Invert Shadow: false\n\
      Name: {name}\n\
      Overtake BG Color Properties: false\n\
      Overtake FG Color Properties: true\n\
      Overtake Position Properties: true\n\
      Topic: {topic}\n\
      Value: true\n\
      font: DejaVu Sans Mono\n\
      height: {height}\n\
      left: {left}\n\
      line width: 2\n\
      text size: {text_size}\n\
      top: {top}\n\
      width: {width}\n'

plotter_text = '    - Buffer length: 100\n\
      Class: jsk_rviz_plugin/Plotter2D\n\
      Enabled: true\n\
      Name: {name}\n\
      Show Value: true\n\
      Topic: {topic}\n\
      Value: true\n\
      auto color change: false\n\
      auto scale: true\n\
      background color: 0; 0; 0\n\
      backround alpha: 0\n\
      border: true\n\
      foreground alpha: 0.699999988079071\n\
      foreground color: 25; 255; 240\n\
      height: {height}\n\
      left: {left}\n\
      linewidth: 1\n\
      max color: 255; 0; 0\n\
      max value: 1\n\
      min value: -1\n\
      show caption: true\n\
      text size: {text_size}\n\
      top: {top}\n\
      update interval: 0.03999999910593033\n\
      width: {width}\n'

l_gauge_text = '    - Class: jsk_rviz_plugin/LinearGauge\n\
      Enabled: true\n\
      Name: {name}\n\
      Show Value: true\n\
      Topic: {topic}\n\
      Value: true\n\
      Vertical Gauge: false\n\
      auto color change: false\n\
      background color: 0; 0; 0\n\
      backround alpha: 0\n\
      border: true\n\
      foreground alpha: 0.699999988079071\n\
      foreground color: 25; 255; 240\n\
      height: {height}\n\
      left: {left}\n\
      linewidth: 1\n\
      max color: 255; 0; 0\n\
      max value: 100\n\
      min value: 0\n\
      show caption: true\n\
      text size: {text_size}\n\
      top: {top}\n\
      update interval: 0.03999999910593033\n\
      width: {width}\n'

type_str_mappings = {
  'std_msgs/Float32': Float32,
  'geometry_msgs/PoseStamped': PoseStamped,
  'jsk_rviz_plugins/OverlayText': OverlayText
}

type_element_map = {
  'text': overlay_text,
  'c_gauge': c_gauge_text,
  'plotter': plotter_text,
  'l_gauge': l_gauge_text
}

LEFT      = 50
TOP       = 50
HEIGHT    = 150
WIDTH     = 150
TEXT_SIZE = 6

MARGIN_SIZE = 50

# TODO: Iterate through published topics and find all with prefix '/gui'
# TODO: Get type of pulished topic to determine appropriate gui element to display
# TODO: For each gui element, replace the tokens in element text with proper values
# TODO: Figure out when a new config is needed/how to scale the elements appropriately
class gui_spawner():
  def __init__(self):
    self.rviz_components = self.generate_components_string()

    # Read lines of template file
    data = None
    with open('src/ppl_ros/factory_monitor_gui/config/template.rviz', 'r') as file:
      data = file.readlines()

    for i in range(0, len(data)):
      # Find line to insert rviz components
      if str(data[i]).strip() == '##':
        # Replace line with rviz component text
        data[i+1] = self.rviz_components
        break

    # Write new data to rviz config
    with open('src/ppl_ros/factory_monitor_gui/config/config.rviz', 'w') as file:
      file.writelines(data)
      
    return
  
  def run(self):
    return
  
  def generate_components_string(self):
    gui_elements = ''
    all_topics = rospy.get_published_topics()

    gui_topics = defaultdict(list)
    for topic in all_topics:
      str_parts = str(topic[0]).split('/')[1:]
      if str_parts[0] == 'gui':
        gui_topics[str_parts[2]].append(topic)
    
    left = LEFT
    top = TOP
    height = HEIGHT
    width = WIDTH
    text_size = TEXT_SIZE
    # scaling = 1.0

    
    elem_num = 1
    for key in gui_topics:
      if str(key).startswith('ws'):
        elem_text, new_top, new_left = self.generate_workstation_string(key, gui_topics[key], top, left, text_size)
        gui_elements += elem_text
        top = new_top
        if elem_num % 4 == 0:
          left = new_left
          top = TOP
        elem_num += 1
        

    return gui_elements
  
  def generate_workstation_string(self, ws_name, topics, top, left, text_size):
    """Generate the string which represents the populated RViz widget for
    information about a workstation.

    Args:
        ws_name (str): Name of the workstation
        ws_topics ([(topic, type, ?), ...]): list of topics broadcasted by the workstaiton
        top (float): offset from the top
        left (float): offset from the left
        text_size (int): text size to use

    Returns:
        str: populated string of gui elements for the workstation.
    """
    gui_elements = ''

    key_top = top
    key_left = left

    ws_elem_height = 0
    ws_elem_width = 0

    for topic_type in type_element_map.keys():
      for topic in topics:
        topic_parts = str(topic[0]).split('/')
        element_type = topic_parts[2]
        if topic_type == element_type:
          topic_str = str(topic[0]).strip('/gui/'+ element_type)
          elem_str = ''

          # WS label + servicing robot
          if element_type == 'text':
            ws_name_label = type_element_map['text']
            element_text = ws_name_label.format(name=topic_str, topic=topic[0], left=str(key_left), top=str(key_top), height=str(int(HEIGHT/4)), width=str(int(WIDTH*3)), text_size=str(text_size*2))

            key_top += int(HEIGHT/4 + MARGIN_SIZE/4)

          # Est time to empty c_gauge
          elif element_type == 'c_gauge':
            elem_str = type_element_map[element_type]
            element_text += elem_str.format(name=topic_str, topic=topic[0], left=str(key_left), top=str(key_top), height=str(HEIGHT), width=str(WIDTH), text_size=str(text_size))
            
            # assume c_gauges are at bottom, reset top

            ws_elem_height = int(key_top + HEIGHT)
            key_top = top
            key_left += int(WIDTH + MARGIN_SIZE/4)
          elif element_type == 'plotter':
            elem_str = type_element_map[element_type]
            element_text += elem_str.format(name=topic_str, topic=topic[0], left=str(key_left), top=str(int(key_top + MARGIN_SIZE/6)), height=str(int(HEIGHT/3)), width=str(WIDTH), text_size=str(text_size))

            key_top += int(HEIGHT/3 + MARGIN_SIZE)
            ws_elem_width = int(key_left + WIDTH)
          elif element_type == 'l_gauge':
            elem_str = type_element_map[element_type]
            element_text += elem_str.format(name=topic_str, topic=topic[0], left=str(key_left), top=str(key_top), height=str(int(HEIGHT/6)), width=str(WIDTH), text_size=str(text_size))

            key_top += int(HEIGHT/3 + MARGIN_SIZE/4)
        
      # add text to gui_elements string
    gui_elements += element_text

    # adjust top and left offsets for next workstation
    new_top = int(ws_elem_height + MARGIN_SIZE)
    # left = int(ws_elem_width + MARGIN_SIZE)
    new_left = int(ws_elem_width + MARGIN_SIZE)
    
    return gui_elements, new_top, new_left
  
  # def populate_component_str(self, data, topic, top, left, height, width, text_size):
  #   element_text = type_element_map[str(topic[1])]
  #   topic_str = str(topic[0]).strip('/gui')
  #   if type_str_mappings[topic[1]] == OverlayText:
  #     text_size = TEXT_SIZE * 2
  #     width = int(WIDTH * 3)
    
  #   # Format ignores unneeded tokens so we can ignore
  #   element_text = element_text.format(name=topic_str, topic=topic[0], left=str(left), top=str(top), height=str(height), width=str(width), text_size=str(text_size))

  #   new_left = left + int(width + LEFT)
  #   new_top = top + height

  #   return element_text, new_top, new_left

  def generate_robot_string(self):
    gui_elements = ''
    return gui_elements

if __name__ == '__main__':
  try:
    rospy.init_node("gui_spawner")
    node = gui_spawner()
    node.run()
  except rospy.ROSInterruptException:
    pass