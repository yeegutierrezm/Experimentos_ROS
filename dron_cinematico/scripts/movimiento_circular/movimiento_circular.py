#!/usr/bin/env python

#Use 'python' for ROS Kinetic and Melodic.
#Use 'python3' for ROS Noetic
"""This python script is the equivalent to the 'src/hector_quadrotor_simple_movement.cpp', so it executes the same tasks.
For more information, please refers to that source file.

-- Important: To land the drone and finish this node, please press 'q' key."""

__author__ = "C. Mauricio Arteaga-Escamilla"

import rospy
from hector_uav_msgs.srv import EnableMotors #rosservice info /enable_motors 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#kbhit function implemented on Linux
import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Define global variables (position and orientation, using Euler anlges)
x=0; y=0; z=0; roll=0; pitch=0; yaw=0
takeoff_alt = 1.2 #Desired initial altitude

vel = Twist() #Create Twist message instance


def getKey(): #Function to use keyboard events on Linux
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def enable_motors(): #Function to call "/enable_motors" service, needed to move the drone
    SERVICE_ENABLE_MOTORS = "enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("Enable motors successful\n")
        else:
            print("Enable motors failed\n")
    except rospy.ServiceException:
    	print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")

def poseCallback(msg): #Callback function to get the drone posture
        global x, y, z, roll, pitch, yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        #Operations to convert from quaternion to Euler angles
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, yaw) = euler_from_quaternion(quater_list) #Euler angles are given in radians

def movement(x,y,z,turn): #Function to assign control signals (Vx, Vy, Vz, Wz)
        vel.linear.x = x; vel.linear.y = y
        vel.linear.z = z; vel.angular.z = turn

def takeoff(): #Takeoff function
	movement(0,0,0.2,0)
	velocity_publisher.publish(vel)
	
	rospy.logwarn(" Desired takeoff altitude = %.2f\n Taking off ...\n", takeoff_alt)
	while(z < takeoff_alt-0.1):
		velocity_publisher.publish(vel)
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == 'q'):
			break
	
	movement(0,0,0,0)
	velocity_publisher.publish(vel)

def land(): #Land function
	movement(0,0,-0.2,0)
	velocity_publisher.publish(vel)
	print("\n Landing ...\n")
	
	while(z > 0.3):
		velocity_publisher.publish(vel)
		rate.sleep(); rospy.sleep(0.1)
	
	movement(0,0,0,0)
	velocity_publisher.publish(vel)
	

def main_function():
	rospy.init_node("hector_quadrotor_simple_movement", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	global rate
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
	
	global velocity_publisher
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #To publish in the topic
	rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) #To subscribe to the topic
	
	
	enable_motors() #Execute the functions
	takeoff()
	
	#Circular movement on the horizontal plane
	movement(0.4,0,0,0.5)
	
	print("Press 'q' to land the drone and finish the node\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n") #Warning message
	
	#t0 = rospy.Time.now().to_sec()
	
	while(1):
		velocity_publisher.publish(vel) #Publish the velocities
		if counter == 50: #Frequency divisor
			rospy.loginfo("x: %.3f y: %.3f z: %.3f roll: %.3f pitch: %.3f yaw: %.3f\n", x,y,z, roll,pitch,yaw)
			#Print in terminal some variables
			counter = 0 #Reset the counter
		else:
			counter += 1
		
		rate.sleep() #spinOnce() function does not exist in python
		key = getKey()
		if(key == 'q'):
			break

	land() #Execute land function
	print("\n Node finished\n")


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    try:
    	main_function()  #Execute the function
	
    except rospy.ROSInterruptException:
        pass

    #if os.name != 'nt':
    #   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
