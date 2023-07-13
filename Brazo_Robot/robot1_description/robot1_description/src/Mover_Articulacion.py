#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math

def Mover_Articulacion():
    pub = rospy.Publisher('/robot1/joint3_position_controller/command', Float64, queue_size=10)
    rospy.init_node('Mover_Articulacion', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
	    position = 3.141592/2 # Radianes, no grados
	    rospy.loginfo(position)    # Mostrar en consola
	    pub.publish(position)
	    rate.sleep()

if __name__ == '__main__':
    try:
        Mover_Articulacion()
    except rospy.ROSInterruptException:
        pass
