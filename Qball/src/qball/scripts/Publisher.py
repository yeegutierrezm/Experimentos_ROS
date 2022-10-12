#!/usr/bin/env python3

'''
Este archivo python corre un nodo de ROS de nombre Publisher, el cual controla inyecta las 12 variables de estado al dron

Este nodo publica y subscribe en los siguientes tópicos:
        PUBLICATIONS                        SUBSCRIPTIONS
        /gazebo/set_model_state             /
'''

# Importando las librerías requeridas
import rospy
import numpy as np
import time
import tf
import math # Para la raíz cuadrada

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float32

# Variables globales
pi = 3.141592653589

# Intensidad de campo gravitatorio en m/s²
g = 9.81		

# Masa del dron completo en Kg		
m = 1.79 
T = 0.04			

# Constantes de proporcionalidad
kf = 0.007
km = 0.8

# Longitud desde el propel hasta el centro del dron en metros
l = 0.2

# Inercias
Ixx = 0.03
Iyy = 0.03
Izz = 0.04


class Qball():

    def __init__(self):
    
        # Inicializando el nodo de ROS de nombre Publisher
        rospy.init_node('Publisher')
         
        self.tau1 = [0.0, 0.0, 17.4] # [fx fy fz]
        self.tau2 = [0.000, 0.000, 0.000] # [tau_phi tau_the tau_psi]

        self.eta1 = [0.0, 0.0, 0.0] # [x y z]
        self.eta2 = [0.0, 0.0, 0.0] # [phi the psi]
        
        self.nu1 = [0.0, 0.0, 0.0] # [u v w]
        self.nu2 = [0.0, 0.0, 0.0] # [p q r]

        # Este es el Período de muestreo en el cual se necesita correr el PID. Escoger el adecuado. Recordemos que el tiempo
        # paso de simulación  50 ms
        self.sample_time = 0.04  # En segundos
        
        global state
        state = ModelState()
        state.model_name = 'qball'
        state.reference_frame = 'qball' 

        # Cinemática
        #state.pose.position.x = 0
        #state.pose.position.y = 0
        #state.pose.position.z = 0
        #quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        #state.pose.orientation.x = quaternion[0]
        #state.pose.orientation.y = quaternion[1]
        #state.pose.orientation.z = quaternion[2]
        #state.pose.orientation.w = quaternion[3]
        
        # Dinámica
        state.twist.linear.x = 0
        state.twist.linear.y = 0
        state.twist.linear.z = 0
        state.twist.angular.x = 0
        state.twist.angular.y = 0
        state.twist.angular.z = 0



    def Publicar_Estado(self):        

        # Realimentando las 12 variables de Estado         
        
        # Cinemática
        #state.pose.position.x = self.eta1[0]
        #state.pose.position.y = self.eta1[1]
        #state.pose.position.z = self.eta1[2]
        #quaternion = tf.transformations.quaternion_from_euler(self.eta2[0], self.eta2[1], self.eta2[2])
        #state.pose.orientation.x = quaternion[0]
        #state.pose.orientation.y = quaternion[1]
        #state.pose.orientation.z = quaternion[2]
        #state.pose.orientation.w = quaternion[3]

        # Dinámica
        state.twist.linear.x = self.nu1[0]
        state.twist.linear.y = self.nu1[1]
        state.twist.linear.z = self.nu1[2]
        state.twist.angular.x = self.nu2[0]
        state.twist.angular.y = self.nu2[1]
        state.twist.angular.z = self.nu2[2]
        
        # Publicando las 12 variables de Estado
        #ssm(state)     
        ssm.publish(state)
        
        self.eta1 = [state.pose.position.x, state.pose.position.y, state.pose.position.z]   # [x y z]
        
        # Convirtiendo Quaterniones en ángulos de Euler                                     # [phi the psi]       
        (self.eta2[1], self.eta2[0], self.eta2[2]) = tf.transformations.euler_from_quaternion([state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w])
        
        self.nu1 = [state.twist.linear.x, state.twist.linear.y, state.twist.linear.z]       # [u v w]
        self.nu2 = [state.twist.angular.x, state.twist.angular.y, state.twist.angular.z]    # [p q r]
        
        
        

    def Modelo(self):
    
        fz = self.tau1[2]
        
        tau_phi = self.tau2[0]
        tau_the = self.tau2[1]
        tau_psi = self.tau2[2]

        # Variables previas (x_prev)
        x = self.eta1[0]
        y = self.eta1[1]
        z = self.eta1[2]
        
        phi = self.eta2[0]
        the = self.eta2[1]
        psi = self.eta2[2]
        
        u = self.nu1[0]
        v = self.nu1[1]
        w = self.nu1[2]
        
        p = self.nu2[0]
        q = self.nu2[1]
        r = self.nu2[2]
            
            
        # Ecuaciones del modelo    
        d_x = (  u*np.cos(psi)*np.cos(the) - v*(np.cos(phi)*np.sin(psi) - np.sin(phi)*np.cos(psi)*np.sin(the)) +
        w*(np.sin(phi)*np.sin(psi) + np.cos(psi)*np.cos(phi)*np.sin(the)) )

        d_y = (u*np.cos(the)*np.sin(psi) + v*(np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(psi)
        * np.sin(the)) - w*(np.cos(psi)*np.sin(phi) - np.cos(phi)*np.sin(psi)*np.sin(the)))
        
        d_z = -u*np.sin(the) + v*np.cos(the)*np.sin(phi) + w*np.cos(phi)*np.cos(the)
        
        d_phi = p + q*np.sin(phi)*np.tan(the) + r*np.cos(phi)*np.tan(the)
        d_the = q*np.cos(phi) - r*np.sin(phi)
        d_psi = r*np.cos(phi)/np.cos(the) + q*np.sin(phi)/np.cos(the)

        d_u = r*v - q*w - g*np.sin(the)
        d_v = p*w - r*u + g*np.sin(phi)*np.cos(the)
        d_w = q*u - p*v - fz/m + g*np.cos(phi)*np.cos(the)
        
        d_p = (Iyy-Izz)/Ixx * q*r + tau_phi/Ixx
        d_q = (Izz-Ixx)/Iyy * p*r - tau_the/Iyy
        d_r = (Ixx-Iyy)/Izz * p*q - tau_psi/Izz


        # Integración
        
        # Variables actuales
        # x_act = x_prev + T * d_x
        
        self.eta1[0] = x + T * d_x
        self.eta1[1] = y + T * d_y
        self.eta1[2] = z + T * d_z

        self.eta2[0] = phi + T * d_phi
        self.eta2[1] = the + T * d_the
        self.eta2[2] = psi + T * d_psi

        self.nu1[0] = u + T * d_u;
        self.nu1[1] = v + T * d_v;
        self.nu1[2] = w + T * d_w;

        self.nu2[0] = p + T * d_p;
        self.nu2[1] = q + T * d_q;
        self.nu2[2] = r + T * d_r;        
        
        
        
        
        
        
    def Torques_a_Velocidades(self):
        # Matriz H
        H = np.array([[kf, kf, kf, kf],[0, l*kf, 0, -l*kf], [-l*kf, 0, l*kf, 0], [km, -km, km, -km]])
        H_inversa = np.linalg.inv(H)
        Torques = np.array([ [self.tau1[2]], [self.tau2[0]], [self.tau2[1]], [self.tau2[2]] ])
        wi2 = np.dot(H_inversa, Torques)  # Vector fila
        
        w1 = math.sqrt(wi2[0])
        w2 = math.sqrt(wi2[1])
        w3 = math.sqrt(wi2[2])
        w4 = math.sqrt(wi2[3])
         
        prop1_pub.publish(w1)
        prop2_pub.publish(w2)
        prop3_pub.publish(w3)
        prop4_pub.publish(w4)
        


if __name__ == '__main__':
    
    # Pausa de 6 segundos para abrir y cargar Gazebo
    t = time.time()
    while time.time() -t < 6: 
        pass

    q_ball = Qball()
    r = rospy.Rate(q_ball.sample_time) 
    #ssm = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState) 
    #ssm = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    prop1_pub = rospy.Publisher('/edrone/prop1_speed', Float32, queue_size=1)
    prop2_pub = rospy.Publisher('/edrone/prop2_speed', Float32, queue_size=1)
    prop3_pub = rospy.Publisher('/edrone/prop3_speed', Float32, queue_size=1)
    prop4_pub = rospy.Publisher('/edrone/prop4_speed', Float32, queue_size=1)

    while not rospy.is_shutdown():
        #q_ball.Publicar_Estado()    
        #q_ball.Modelo()
        #q_ball.Torques_a_Velocidades()
        time.sleep(0.04)

