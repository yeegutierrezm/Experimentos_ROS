#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
    
def model_state_callback(msg):
    msg.model_name = 'qball'
    msg.reference_frame = 'qball'
    print (msg)
    ssm.publish(msg)

rospy.init_node('Subscriber')

sub = rospy.Subscriber('/gazebo/model_state', ModelState, model_state_callback)
ssm = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

rospy.spin()
