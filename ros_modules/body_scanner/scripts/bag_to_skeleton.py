#!/usr/bin/env python

import roslib; roslib.load_manifest('body_scanner')
import tf
import rosbag

import sys

bag = rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages(topics=['/tf']):
  print msg
bag.close()
