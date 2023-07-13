# Lazo Cerrado - Trayectoria Lemniscata

![](https:#github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/dron_cinematico/src/lemniscata_helice/lemniscata.gif)
  
Este script de python es equivalente a ```src/lemniscata_helice_grafica.cpp```, así que este ejecuta las mismas tareas.  
<br><br>

Este nodo ejecuta el controlador para resolver el problema de seguimiento utilizando un dron en gazebo. en la funcion "velocity_controller" tanto la trayectoria deseada como las señales de control son calculadas, para este ejemplo la trayectoria deseada es una lemniscata.  
<br><br>

Es necesario hacer una llamada al servicio ```/enable_motors``` para poder utilizar al dron. Tambien se definen las funciones de despegue y aterrizaje. Tanto las velocidades traslacionales $(V_x, V_y, V_z)$ y la velocidad rotacional $(W_z)$ son publicadas en el topico ```/cmd_vel```. La posicion $(x, y, z)$ y la orientacion utilizando cuterniones son recibidas del topico ```ground_truth/state```.  
<br>

Para aterrizar el dron y terminar este nodo es necesario presionar la tecla ```q```.  
<br><br>
   


## Instrucciones

Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.
```
roslaunch dron_cinematico lanzador_principal.launch
```

Correr el nodo
```
rosrun dron_cinematico lemniscata_helice
```

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
from nav_msgs.msg import Odometry
```

Para publicar velocidades (mensaje)
```python
from geometry_msgs.msg import Twist
```

Para hacer la conversion de cuaternión a ángulo-Euler (fichero)
```python
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

Defino variable global (posición y orientación, usando ángulos de euler)
```python
x=0; y=0; z=0; roll=0; pitch=0; yaw=0
```

Configuro la altura deseada al despegar
```python
takeoff_alt = 1.2
```

Variable para indicar qué tecla se ha activó en el teclado del computador
```python
int key;
```

Temporizador, tiempo inicial, velocidades de traslación máximas [m / s] y velocidades de rotación máximas [rad / s] para obtener una simulacion mas precisa a la real. (los rotores no tienen velocidades infinitas)
```python
t = 0.0; t0 = 0.0; Vxy_max = 1.5;  Vz_max = 0.5; Wz_max = 4;
```

Período de trayectoria, el controlador de ganancia kx = ky = k para los errores de seguimiento 
```python
T = 100.0; k = 0.1;
```

Defino los errores de seguimiento cartesianos, la posicion y orientacion deseada y sus respectiva derivada con respecto al tiempo
```python
ex = 0.0; ey = 0.0; ez = 0; e_yaw = 0
```


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
```
<br><br>


Función para obtener la postura del dron
```python
def poseCallback(msg):
        global x, y, z, roll, pitch, yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
```

Operaciones para convertir de cuaternión a ángulos de Euler
```python
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, yaw) = euler_from_quaternion(quater_list)
```
<br><br>


Función para asignar señales de control $(V_x, V_y, V_z, W_z)$
```python
def movement(x,y,z,turn):
  signals (Vx, Vy, Vz, Wz)
	vel.linear.x = x; vel.linear.y = y
	vel.linear.z = z; vel.angular.z = turn
```
<br><br>


Función de despegue
```python
def takeoff():

  # Asignamos una velocidad de ascenso de 0,2 m/s
  movement(0,0,0.2,0); 
  # Publicamos
  velocity_publisher.publish(vel); 

  rospy.logwarn(" Desired takeoff altitude = %.2f\n Taking off ...\n", takeoff_alt)

  # Utilizando el ciclo se compara si se alcanza la altitud deseada con una tolerancia de 10 cm
	while(z < takeoff_alt-0.1):
		velocity_publisher.publish(vel)
		rate.sleep(); rospy.sleep(0.1)
		key = getKey()
		if(key == 'q'):
			break

  # Se publica velocidad de 0 para mantener vuelo estacionario
  movement(0,0,0,0); 
  velocity_publisher.publish(vel);
}
```
<br><br>


Funcion de aterrizaje
```python
def land():
  # Velocidad de descenso
  movement(0,0,-0.2,0); 
  velocity_publisher.publish(vel);
  print("\n Landing ...\n")

  # Se compara altura y cuando este a 30 cm se asignan velocidad de 0 
	while(z > 0.3):
    velocity_publisher.publish(vel)
		rate.sleep(); rospy.sleep(0.1)

  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}
```
<br><br>


Funcion que genera la trayectoria deseada y las señales de control para el dron
```python
def velocity_controller():
  # Defino las variables globales 
  global ex, ey, ez, e_yaw

  # Defino la trayectoria deseada: Lemniscata por medio de las ecuaciones
  a = 3; b = 1.5; X0 = 0; Y0 = -0.5; Z0 = 1.5; w = 2*pi/T; c = 0.5; d = pi/2

  # Posicion deseada en el espacio 3d
  Xd = X0+a*sin(w*t);
  Yd = Y0+b*sin(2*w*t);
  Zd = Z0+c*sin(w*t); #Note: 1 <= Zd <= 2
  Yawd = d*sin(w*t);

  # Derivadas del tiempo correspondiente
  Xdp = a*w*cos(w*t);
  Ydp = 2*b*w*cos(2*w*t);
  Zdp = c*w*cos(w*t);
  Yawdp = d*w*cos(w*t);

  # Cálculo los errores de seguimiento 
  ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw - Yawd; 

  # Señales de control auxiliares en cordenadas globales
  Ux = Xdp-k*ex; Uy = Ydp-k*ey
  
  # Velocidades traslacionales del dron con respecto al marco de referencia del robot
  Vx = Ux*cos(yaw)+Uy*sin(yaw);
  Vy = -Ux*sin(yaw)+Uy*cos(yaw);

  #Señales de control, estas se calculan directamente ya que son con respecto a la estructura del robot 
  Vz = Zdp-k*ez;
  Wz = Yawdp-k*e_yaw;

  # Se realiza una saturacion ha estas velocidades  (para simular que el dron no alcanza velocidades muy grandes)
	if(abs(Vx)>Vxy_max):
		Vx = Vxy_max*abs(Vx)/Vx; print("Sat Vx\t")
	if(abs(Vy)>Vxy_max):
		Vy = Vxy_max*abs(Vy)/Vy; print("Sat Vy\t")
	if(abs(Vz)>Vz_max):
		Vz = Vz_max*abs(Vz)/Vz; print("Sat Vz\t")
	if(abs(Wz)>Wz_max):
		Wz = Wz_max*abs(Wz)/Wz; print("Sat Wz\t")

  movement(Vx,Vy,Vz,Wz);
  
  # Publico las 4 señales de control 
  velocity_publisher.publish(vel); 
}
```
<br><br>


Función principal donde se realiza el llamado a todas las demás funciones descritas anteriormente
```python
def main_function():
  # Inicializacion del nodo
  rospy.init_node("hector_quadrotor_controller", anonymous=True)
  # Asignar frecuencia (Hz) de ejecucion al nodo
  global rate
	rate = rospy.Rate(50)
  # Se crea un contador
  counter = 0; 

  # Para publicar en el tópico
	global velocity_publisher
  

  # Publica en el topico velocidades utilizando el mensaje Twist 
  velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  
  # Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta 
  rospy.Subscriber('/ground_truth/state',Odometry, poseCallback) 
  

  # Ejecuta y llama la función que llama al servicio enable 
  enable_motors()
  # Despega el dron 
  takeoff()
 
  # Imprimo mensaje de informacion
  print("Press 'q' to land the drone and finish the node\n")
  # Mensaje de advertencia
  rospy.logwarn("To start the movement, the simulation must be running\n\n")

  # Obtengo el tiempo inicial 
  global t, t0
	t0 = rospy.Time.now().to_sec()

	while(1):
    # Calculo el tiempo del controlador   
    t = rospy.Time.now().to_sec()-t0
    #Calculo las señales de control
    velocity_controller()

    # Divisor de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
		if counter == 50: #Frequency divisor
      # Imprimir en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
			rospy.loginfo("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw)
      # Reseteo el contador
			counter = 0
		else:
			counter += 1    

    # La función spinOnce() no existe en python
    # Comando para esperar el resto del tiempo para completar la frecuencia del bucle 
		rate.sleep()
		key = getKey()
		if(key == 'q'): #b'\x1b' is 'ESC' key
			break
  
  #Funcion aterrizar el dron 
  land(); 
  print("\n Node finished\n");
  return 0;
}
```
<br><br>


Función que se ejecuta de manera automática al llamar al archivo ```.py```en terminal
```python
if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
      # Ejecutar la función
    	main_function()  
	
    except rospy.ROSInterruptException:
        pass
```
