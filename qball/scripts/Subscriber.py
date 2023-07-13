#!/usr/bin/env python3
# Este archivo publica las 2 variables string 
import rospy
from gazebo_msgs.msg import ModelState
    
def model_state_callback(msg):
    msg.model_name = 'qball'
    msg.reference_frame = 'ground_plane'
    print (msg)
    ssm.publish(msg)

rospy.init_node('Subscriber')

sub = rospy.Subscriber('/gazebo/model_state', ModelState, model_state_callback)
ssm = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

rospy.spin()
