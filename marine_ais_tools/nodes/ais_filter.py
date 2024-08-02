#!/usr/bin/env python3

import rospy
from marine_ais_msgs.msg import AIS
from geographic_msgs.msg import BoundingBox

filtered_ais_publisher = None

include_areas = []
include_ids = []

world = BoundingBox()
world.max_pt.latitude = 90.0
world.min_pt.latitude = -90.0
world.max_pt.longitude = 180.0
world.min_pt.longitude = -180.0


def updateParametersCallback(timer_event: rospy.timer.TimerEvent):
  global include_areas
  if rospy.has_param('~include_areas'):
    areas = rospy.get_param('~include_areas')
    if type(areas) == dict:
      areas = [areas,]
    bounding_boxes = []
    for area in areas:
      bb = BoundingBox()
      try:
        bb.min_pt.latitude = area['min_pt']['latitude']
        bb.min_pt.longitude = area['min_pt']['longitude']
        bb.max_pt.latitude = area['max_pt']['latitude']
        bb.max_pt.longitude = area['max_pt']['longitude']
        bounding_boxes.append(bb)
      except Exception as e:
        rospy.logerr("Can't decode geographic_msgs/BoundingBox: " + str(area))
        rospy.logerr(str(type(e).__name__)+": "+str(e))
    include_areas = bounding_boxes

def isInside(ais_msg: AIS, box: BoundingBox):
  if ais_msg.navigation.pose.position.latitude >= box.min_pt.latitude and \
     ais_msg.navigation.pose.position.latitude <= box.max_pt.latitude and \
     ais_msg.navigation.pose.position.longitude >= box.min_pt.longitude and \
     ais_msg.navigation.pose.position.longitude <= box.max_pt.longitude:
    return True
  return False

def hasValidPosition(ais_msg: AIS):
  return isInside(ais_msg, world)

def aisCallback(msg: AIS):
  skip = False
  if len(include_areas) > 0:
    skip = True
    if(hasValidPosition(msg)):
      for area in include_areas:
        if isInside(msg, area):
          skip = False
          break
      if skip and msg.id in include_ids:
        rospy.logdebug('remove: '+str(msg.id))
        include_ids.remove(msg.id)
      if not skip and not msg.id in include_ids:
        rospy.logdebug('add: '+str(msg.id))
        include_ids.append(msg.id)
    else:
      if (msg.id in include_ids):
        skip = False
      
  if not skip:
    filtered_ais_publisher.publish(msg)


rospy.init_node('ais_filter')

filtered_ais_publisher = rospy.Publisher('filtered_messages',AIS,queue_size=10)

ais_subscriber = rospy.Subscriber('messages', AIS, aisCallback)
update_timer = rospy.Timer(rospy.Duration(1.0), updateParametersCallback)

rospy.spin()

