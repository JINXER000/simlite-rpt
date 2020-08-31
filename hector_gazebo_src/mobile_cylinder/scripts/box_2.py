#!/usr/bin/env python
# -*- coding: utf-8 -*-
#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelState

def pose_publisher():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'box_3'
    pose_msg.pose.position.x=-2
    pose_msg.pose.position.y=-6
    pose_msg.pose.position.z=1
    rate = rospy.Rate(30)
    p=1
    while not rospy.is_shutdown():

           pose_msg.pose.position.x += ((-1)**p)*(2./30)
           if  pose_msg.pose.position.x>=4:
                p=1
           if  pose_msg.pose.position.x<=-5:
                p=2
           pub.publish(pose_msg)

           rate.sleep()

if __name__ == '__main__':
      rospy.init_node('pose_publisher3')
      try:
          pose_publisher()
      except rospy.ROSInterruptException:
          pass
