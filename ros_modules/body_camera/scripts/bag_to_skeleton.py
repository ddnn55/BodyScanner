#!/usr/bin/env python

import roslib; roslib.load_manifest('body_camera')
import tf
import rosbag

import sys
from collections import OrderedDict


# FIXME: move this list body_scanner common python file
openni_skeleton_tf_frames = ['/head_1','/neck_1','/torso_1','/left_shoulder_1','/left_elbow_1','/left_hand_1','/right_shoulder_1','/right_elbow_1','/right_hand_1','/left_hip_1','/left_knee_1','/left_foot_1','/right_hip_1','/right_knee_1','/right_foot_1']


bag = rosbag.Bag(sys.argv[1])

moments = {}
skeleton_frames = []
current_frame = {}
last_stamp = None
for topic, msg, t in bag.read_messages(topics=['/tf']):
  joint_name = msg.transforms[0].child_frame_id
  if joint_name in openni_skeleton_tf_frames:
    joint_transform = msg.transforms[0]
    stamp = joint_transform.header.stamp
    if last_stamp == None:
      last_stamp = stamp
    if stamp not in moments:
      moments[stamp] = {}
    moments[stamp][joint_name] = joint_transform.transform
#    print joint_transform
    last_stamp = stamp
bag.close()


moments = OrderedDict(sorted(moments.items(), key=lambda t: t[0]))
for stamp in moments:
  print str(stamp.secs) + "." + str(stamp.nsecs) + ": "
  print moments[stamp]

print str(len(moments)) + " moments"
