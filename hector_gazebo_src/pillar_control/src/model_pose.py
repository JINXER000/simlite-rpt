#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import *
from hector_uav_msgs.msg import Model_pose


def talker():
    pub = rospy.Publisher('/model_pose', Model_pose, queue_size=10)
    rospy.init_node('model_srv', anonymous=True)
    #rate = rospy.Rate(30) # 10hz
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'box_1'
    while not rospy.is_shutdown():
        objstate = get_state_service(model)
        pub.publish(objstate)
        #rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
