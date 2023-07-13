#!/usr/bin/env python3

# Este archivo publica 12 variables de estado arbtrarias en el tópico 'gazebo/model_state'.
# Los usamos para simular el envío de datos desde Matlab hacia ROS

import rospy
from gazebo_msgs.msg import ModelState


def talker():   

    set_self_state = ModelState()
    set_self_state.model_name = 'qball' 
    set_self_state.pose.position.x = 0.0
    set_self_state.pose.position.y = 0.1
    set_self_state.pose.position.z = 0
    set_self_state.pose.orientation.x = 0.0
    set_self_state.pose.orientation.y = 0.0
    set_self_state.pose.orientation.z = 0.0
    set_self_state.pose.orientation.w = 0
    set_self_state.twist.linear.x = 0
    set_self_state.twist.linear.y = 0.
    set_self_state.twist.linear.z = 0
    set_self_state.twist.angular.x = 0.
    set_self_state.twist.angular.y = 0.
    set_self_state.twist.angular.z = 0
    set_self_state.reference_frame = 'qball'
   
    pub = rospy.Publisher('gazebo/set_model_state', ModelState)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data = set_self_state
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
    
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
