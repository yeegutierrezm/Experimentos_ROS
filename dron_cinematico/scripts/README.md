# Lazo Cerrado - Trayectoria, Líder-Seguidor Gráfica

![](https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/dron_cinematico/src/lider_seguidor/lider_seguidor.gif)

Este archivo python es equivalente a  ```src/lider_seguidor_grafica.cpp```, así que este ejecuta las mismas tareas.  
<br><br>

Con este nodo, la trayectoria deseada del dron líder y la trayectoria del dron seguidor son publicadas en el tópico correspondiente para su visualización en Rviz.

Este nodo ejecuta el controlador para resolver el problema de control de formación, usando el esquema líder-seguidor, con 2 drones simulados en Gazebo. El dron líder desplega su trayectoria deseada (usando el controlador de velocidad), y luego el dron seguidor sigue la trayectoria del líder con un offset.
<br><br>

Primero se hace una llamda al servicio ```/enable_motors``` para cada uno d elos drones. Se definden las funciones de despegue y aterrizaje para ambos drones.  
<br>

Para aterrizar ambos drones y terminar este nodo, se necesita presioanr la tecla ```q```.  
<br><br> 

## Instrucciones:

En diferentes terminales:

Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

> roslaunch dron_cinematico lanzador_2_quadrotores.launch

Corre el nodo
> rosrun dron_cinematico lider_seguidor_grafica 

Corre Rviz con mi archivo de configuración para visualizar los 2 drones y sus trayectorias
> roslaunch dron_cinematico visualizacion_lider_seguidor_graficas.launch
<br><br>













## Explicación de las librerías

```python
#!/usr/bin/env python

# Use 'python' for ROS Kinetic and Melodic
# Use 'python3' for ROS Noetic
```

Para hacer la llamada al servicio ```roservice info/enable_motors```
```python
import rospy, math
from hector_uav_msgs.srv import EnableMotors
```

Para suscribirle a la geometria del dron (mensaje)
```python
from nav_msgs.msg import Odometry, Path
```

Para publicar velocidades (mensaje)
```python
from geometry_msgs.msg import Twist
```

Para hacer la conversion de cuaternión a ángulo-Euler (fichero)
```python
from tf.transformations import euler_from_quaternion, quaternion_from_euler
```

Para poder visualizar trayectorias en Rviz
```python
from geometry_msgs.msg import PoseStamped
```

Las últimas librerías se necesitan para utilizar eventos del teclado
```python
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
```

Importamos las funciones ```Seno()``` y ```Coseno()```.
```python
from math import sin, cos, pi
```
<br><br>







## Explicación del código

Se define la postura de los robots usando listas [x,y,z,roll,pitch,yaw]
```python
r_pose1 = [0,0,0,0,0,0]
r_pose2 = [0,0,0,0,0,0]
```

Altura inicial deseada
```python
takeoff_alt = 0.5 
```

Crear dos instancias de Twist
```python
vel1 = Twist() 
vel2 = Twist()
```

Crear tres instancias de Path
```python
drone_path_msg1 = Path(); drone_path_msg2 = Path(); tr_path_msg = Path()
```

Crear el título del marco global. El marco global es requerido por Rviz para definir el MISMO marco para cualquier robot
```python
global_frame = "/world"
```

Temporizador, tiempo inicial, velocidades de traslación máximas [m / s] y velocidades de rotación máximas [rad / s] para obtener una simulacion mas precisa a la real. (los rotores no tienen velocidades infinitas)
```python
t = 0.0; t0 = 0.0; Vxy_max = 1.5;  Vz_max = 0.5; Wz_max = 4;
```

Período de trayectoria, el controlador de ganancia kx = ky = k para los errores de seguimiento 
```python
T = 100.0; k = 0.1;
```

Errores de seguimiento cartesianos (robot líder)
```python
ex = 0.0; ey = 0.0; ez = 0; e_yaw = 0
```

Señales de control (robot líder)
```python
Vxl = 0; Vyl = 0; Vzl = 0; Wzl = 0
```

Errores de seguimiento cartesianos y estado de formación deseado (robot seguidor)
```python
exf = 0; eyf = 0; ezf = 0; e_yawf = 0; s_d = 1.5; alp_d = pi; s_zd = 0
```

Señales de control (robot seguidor)
```python
Vxf = 0; Vyf = 0; Vzf = 0; Wzf = 0
```
<br><br>


Función para utilizar eventos de teclados en Linux
```python
def getKey():
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
```
<br><br>


Función para llamar al servicio ```/enable_motors```, necesaria para mover el dron
```python
def enable_motors():
    # Dron líder
    SERVICE_ENABLE_MOTORS = "uav1/enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("uav1: Enable motors successful\n")
        else:
            print("uav1: Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")
    
    # Dron seguidor
    SERVICE_ENABLE_MOTORS = "uav2/enable_motors"
    print("Waiting for service", SERVICE_ENABLE_MOTORS)
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print("uav2: Enable motors successful\n")
        else:
            print("uav2: Enable motors failed\n")
    except rospy.ServiceException:
        print("Enable service", SERVICE_ENABLE_MOTORS, "call failed")
```
<br><br>


Función para obtener la postura del dron 1 (líder)
```python
def poseCallback1(msg):
    global r_pose1
    r_pose1[0] = msg.pose.pose.position.x
    r_pose1[1] = msg.pose.pose.position.y
    r_pose1[2] = msg.pose.pose.position.z
```
Operaciones para convertir de cuaternión a ángulos de Euler
```python
    quater = msg.pose.pose.orientation
    quater_list = [quater.x, quater.y, quater.z, quater.w]
    (r_pose1[3], r_pose1[4], r_pose1[5]) = euler_from_quaternion(quater_list)
```
Crear una instancia de PoseStamped
```python
	drone_pose1 = PoseStamped()
	drone_pose1.pose.position = msg.pose.pose.position
	# Agregar la posición del dron a la lista de puntos a graficar
	drone_path_msg1.poses.append(drone_pose1) 
```	
Asignación de los TF (usando para Rviz únicamente)
```python
	br = tf.TransformBroadcaster()
	br.sendTransform( (r_pose1[0],r_pose1[1],r_pose1[2]), 
		tf.transformations.quaternion_from_euler(r_pose1[3], r_pose1[4], r_pose1[5]), rospy.Time.now(), 
		"/uav1/base_link", global_frame )
```	
<br><br>


Función para obtener la postura del dron 2 (seguidor)
```python
def poseCallback2(msg):
	global r_pose2
	r_pose2[0] = msg.pose.pose.position.x
	r_pose2[1] = msg.pose.pose.position.y
	r_pose2[2] = msg.pose.pose.position.z
```
Operaciones para convertir de cuaternión a ángulos de Euler
```python
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(r_pose2[3], r_pose2[4], r_pose2[5]) = euler_from_quaternion(quater_list)
```
Crear una instancia de PoseStamped
```python
	drone_pose2 = PoseStamped()
	drone_pose2.pose.position = msg.pose.pose.position
	# Agregar la posición del dron a la lista de puntos a graficar
	drone_path_msg2.poses.append(drone_pose1) 
```	
Asignación de los TF (usando para Rviz únicamente)
```python
	br = tf.TransformBroadcaster()
	br.sendTransform( (r_pose2[0],r_pose2[1],r_pose2[2]), 
		tf.transformations.quaternion_from_euler(r_pose2[3], r_pose2[4], r_pose2[5]), rospy.Time.now(), 
		"/uav2/base_link", global_frame )
```	
<br><br>


Función para asignar señales de control y publicarlas, drone 1
```python
def move1(x,y,z,turn): #Function to assign control signals (Vx, Vy, Vz, Wz)
       vel1.linear.x = x; vel1.linear.y = y
       vel1.linear.z = z; vel1.angular.z = turn
       vel_pub1.publish(vel1)
```

Función para asignar señales de control y publicarlas, drone 2
```python
def move2(x,y,z,turn):
	vel2.linear.x = x; vel2.linear.y = y
	vel2.linear.z = z; vel2.angular.z = turn
	vel_pub2.publish(vel2)
```
<br><br>


Función de despegue de los 2 drones
```python
def takeoff():
  rospy.logwarn(" Desired takeoff altitude = %.2f\n Taking off ...\n", takeoff_alt)

  # Quadrotor 1 - Asigno velocidades de ascenso
  move1(0,0,0.3,0); 
  
  # Compara si se alcanza la altitud deseada
	while(r_pose1[2] < takeoff_alt-0.1):
		vel_pub1.publish(vel1)
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == 'q'):
			break
  
  # Detener
  move1(0,0,0,0); 

  # Quadrotor 2 - Asigno velocidades de ascenso
  move2(0,0,0.3,0); 

  # Compara si se alcanza la altitud deseada
	while(r_pose2[2] < takeoff_alt-0.1): 
		vel_pub2.publish(vel2)
		vel_pub1.publish(vel1) # Importante: Las velocidades del dron 1 deben seguir sindo publicadas para evitar un aterrizaje no deseado.
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == 'q'):
			break

  # Detener
  move2(0,0,0,0); 
```
Importante: Las velocidades del dron 1 deben seguir sindo publicadas para evitar un aterrizaje no deseado.  
<br><br>


Funcion de aterrizaje de todos los drones 
```python
def land():
  print("\n Landing ...\n")

  # Asigno velocidades de descenso
  move1(0,0,-0.2,0); move2(0,0,-0.2,0)

	while(r_pose1[2] > 0.3 or r_pose2[2] > 0.3):
		vel_pub1.publish(vel1)
		vel_pub2.publish(vel2)
		rate.sleep(); rospy.sleep(0.1)
  
  # Apaga todos los drones
  move1(0,0,0,0);  move2(0,0,0,0); 
}
```
<br><br>


## Función leader_vel_control()
Función para generar la trayectoria deseada y calcular el control de señales del robot líder
```python
def leader_vel_control():
  #   Indicar cuáles son las variables globales para ser usadas en la función principal
  global ex, ey, ez, e_yaw, Vxl, Vyl, vzl, Wzl 
  # Definir la trayectoria deseada: Hélice
  X0 = 0; Y0 = 0; Z0 = 0.5; radius = 3; w = 2*pi/T; Vzd = 0.02
```

Posición deseada en el espacio 3D
```python
	Xd = X0+radius*sin(w*t)
	Yd = Y0+radius*cos(w*t)
  # Incremento lineal
  Zd = Z0+Vzd*t
  # comportamiento sinusoidal
  Yawd = sin(w*t)
```

Derivadas de tiempo correspondientes
```python
	Xdp = radius*w*cos(w*t)
	Ydp = -radius*w*sin(w*t)
	Zdp = Vzd
	Yawdp = w*cos(w*t)
```

Asignar la posición deseada al objeto "tr_pose"
```python
	# Crear una instancia PoseStamped
	tr_pose = PoseStamped() 
	tr_pose.pose.position.x = Xd; tr_pose.pose.position.y = Yd
	tr_pose.pose.position.z = Zd 
	# Agregar el nuevo punto a la lista
	tr_path_msg.poses.append(tr_pose) 
```	

Calcular errores de seguimiento
```python
  ex = r_pose1[0]-Xd; ey = r_pose1[1]-Yd; ez = r_pose1[2]-Zd; e_yaw = r_pose1[5]-Yawd
```

Controlador cinemático. Controles auxiliares, en coordenadas globales (variables locales)
```python 
	Ux = Xdp-k*ex; Uy = Ydp-k*ey
```

Velocidades de traslación con respecto al marco del robot
```python
	Vxl = Ux*cos(r_pose1[5])+Uy*sin(r_pose1[5])
	Vyl = -Ux*sin(r_pose1[5])+Uy*cos(r_pose1[5])
```

Controlador cinemático. Nota: Vz y Wz se calculan directamente ya que se dan con respecto al marco de referencia del robot.
```python  
	Vzl = Zdp-k*ez
	Wzl = Yawdp-k*e_yaw
```

 Velocidades de saturacion
```python  
	if(abs(Vxl)>Vxy_max):
		Vxl = Vxy_max*abs(Vxl)/Vxl; print("Sat Vxl\t")
	if(abs(Vyl)>Vxy_max):
		Vyl = Vxy_max*abs(Vyl)/Vyl; print("Sat Vyl\t")
	if(abs(Vzl)>Vz_max):
		Vzl = Vz_max*abs(Vzl)/Vzl; print("Sat Vzl\t")
	if(abs(Wzl)>Wz_max):
		Wzl = Wz_max*abs(Wzl)/Wzl; print("Sat Wzl\t")
```
Publica las 4 señales de control
```python
  move1(Vxl,Vyl,Vzl,Wzl)
```
<br><br>


## Función follower_vel_control()

Función para calcular el control de señales del robot seguidor.

Nota: la posición deseada del seguidor y la orientación de guiñada vienen dadas por las del líder.

```python
def follower_vel_control():
  # Indicar cuáles son las variables globales para ser usadas en la función principal
  global exf, eyf, ezf, e_yawf
```

Posición deseada del Seguidor con respecto al marco global
```python  
	Xfd = r_pose1[0]+s_d*cos(r_pose1[5]+alp_d)
	Yfd = r_pose1[1]+s_d*sin(r_pose1[5]+alp_d)
```

Velocidades de traslación del dron líder en el marco global
```python  
	xlp = Vxl*cos(r_pose1[5])-Vyl*sin(r_pose1[5])
	ylp = Vxl*sin(r_pose1[5])+Vyl*cos(r_pose1[5])
```

Derivadas de tiempo deseadas correspondientes
```python  
	Xfdp = xlp-Wzl*s_d*sin(r_pose1[5]+alp_d)
	Yfdp = ylp+Wzl*s_d*cos(r_pose1[5]+alp_d)
```

Calcular los errores de seguimiento (con respecto al marco global)
```python  
	exf = r_pose2[0]-Xfd; eyf = r_pose2[1]-Yfd
	ezf = r_pose2[2]-(r_pose1[2]+s_zd); e_yawf = r_pose2[5] - r_pose1[5] # Tanto la altitud como la orientación son iguales a las del drone líder.
```

Controlador cinematico. Controles auxiliares, en coordenadas globales
```python
	Ux = Xfdp-k*exf; Uy = Yfdp-k*eyf
 ```

Velocidades de traslación con respecto al marco del robot
```python
	Vxf = Ux*cos(r_pose2[5])+Uy*sin(r_pose2[5])
	Vyf = -Ux*sin(r_pose2[5])+Uy*cos(r_pose2[5])
```

Velocidades verticales y de rotación
```python
	Vzf = Vzl-k*ezf
	Wzf = Wzl-k*e_yawf
```

Velocidades de saturacion
```python  
	if(abs(Vxf)>Vxy_max):
		Vxf = Vxy_max*abs(Vxf)/Vxf; print("Sat Vxf\t")
	if(abs(Vyf)>Vxy_max):
		Vyf = Vxy_max*abs(Vyf)/Vyf; print("Sat Vyf\t")
	if(abs(Vzf)>Vz_max):
		Vzf = Vz_max*abs(Vzf)/Vzf; print("Sat Vzf\t")
	if(abs(Wzf)>Wz_max):
		Wzf = Wz_max*abs(Wzf)/Wzf; print("Sat Wzf\t")
```

Publica las 4 señales de control
```python  
	move2(Vxf,Vyf,Vzf,Wzf)
```
<br><br>


## Función Principal main()

```python  
def main_function():
  # Inicializo el nodo
  rospy.init_node("two_hector_quadrotors_lf_paths", anonymous=True)
  # Frecuencia del nodo (Hz)
	global rate
	rate = rospy.Rate(50)
	counter = 0
```

Publicar en el tópico
```python   
	global vel_pub1, vel_pub2
	vel_pub1 = rospy.Publisher('uav1/cmd_vel', Twist, queue_size=10)
	vel_pub2 = rospy.Publisher('uav2/cmd_vel', Twist, queue_size=10)
```

Suscribirse al tópico
```python 
	rospy.Subscriber('uav1/ground_truth/state', Odometry, poseCallback1)
	rospy.Subscriber('uav2/ground_truth/state', Odometry, poseCallback2)
 ```

Publicar las gráficas de las trayectorias
```python
	drone_path_pub1 = rospy.Publisher('uav1/path', Path, queue_size=10)
	drone_path_pub2 = rospy.Publisher('uav2/path', Path, queue_size=10)
	tr_path_pub = rospy.Publisher('/tr_path', Path, queue_size=10) 
 ```

Importante: Asignación del MISMO marco de referencia para TODOS los robots
```python
	global drone_path_msg1, drone_path_msg2, tr_path_msg
	drone_path_msg1.header.frame_id = drone_path_msg2.header.frame_id = tr_path_msg.header.frame_id = global_frame
 ```

Ejecuta las funciones "enable_motors" y "takeoff" (para todos los robots)
```python
  enable_motors()
  takeoff()
```

Mensaje de advertencia
```python
	print("Press 'q' to land the drones and finish the node\n")
	print(" To clean up all paths, press 'c' key\n")
	rospy.logwarn(" To start the movement, the simulation must be running\n\n")
```

Obtener el tiempo inicial
```python   
	global t, t0
	t0 = rospy.Time.now().to_sec()
```

```python 
	while(1):
    # Cálculo el tiempo del controlador
		t = rospy.Time.now().to_sec()-t0
    # Cálculo las señales de control del robot líder
		leader_vel_control()
    # Cálculo las señales de control del robot seguidor
		follower_vel_control()
```

Divisor de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
```python 
		if counter == 25:
      # Imprimir en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
			rospy.loginfo("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw)
      # Errores de formación del seguidor de impresión y error de orientación
			rospy.loginfo("exf: %.3f eyf: %.3f ezf: %.2f e_yawf: %.3f\n", exf,eyf,ezf,e_yawf)
			# Reseteo el contador
      counter = 0 
		else:
			counter += 1
```      

Publicar los mensajes "path"
```python 
		drone_path_pub1.publish(drone_path_msg1); drone_path_pub2.publish(drone_path_msg2); tr_path_pub.publish(tr_path_msg)
``` 

```python 
    # Comando para esperar el resto del tiempo para completar la frecuencia del bucle
		rate.sleep() 
		key = getKey()

		# Limpia las trayectorias
		if(key == 'c'): 
			del drone_path_msg1.poses[:]; del drone_path_msg2.poses[:]; del tr_path_msg.poses[:]
			rospy.logwarn("Clear paths\n")

			# Reseteo de la tecla clave
			key = 0;
		elif(key == 'q'):
			break
``` 

```python 
  # Función aterrizar el dron 
	land()
	print("\n Node finished\n")
```

Función que se ejecuta de manera automática al llamar al archivo ```.py```en terminal
```python 
if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
    	main_function()
	
    except rospy.ROSInterruptException:
        pass
```  