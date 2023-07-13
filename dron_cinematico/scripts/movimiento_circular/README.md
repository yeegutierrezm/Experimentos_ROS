# Lazo Abierto - Movimiento Circular

![](https:#github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/dron_cinematico/src/movimiento_circular/Movimiento_Circular.gif)

Este script de python es equivalente a ```src/movimiento_circular.cpp```, así que este ejecuta las mismas tareas.  
<br><br>

Este codigo es un ejemplo para publicar velocidades a un robot simulado y suscribirse a su odometria. 
<br><br>



## Descripción:
Este nodo publica velocidades en lazo abierto al dron simulado en gazebo, y se suscribe a su odometria para obtener la posicion y la orientacion del cuatrirrotor.  
<br>


Es necesario hacer una llamada al servicio ```/enable_motors``` para poder utilizar al dron. Tambien se definen las funciones de despegue y aterrizaje. Tanto las velocidades traslacionales $(V_x, V_y, V_z)$ y la velocidad rotacional $(W_z)$ son publicadas en el topico ```/cmd_vel```. La posicion $(x, y, z)$ y la orientacion utilizando cuterniones son recibidas del topico ```ground_truth/state```.  
<br>

Para aterrizar el dron y terminar este nodo es necesario presionar la tecla ```ESC```.
<br><br>



## Instrucciones:

Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

```
roslaunch dron_cinematico lanzador_principal.launch
```

Correr el nodo
```
rosrun dron_cinematico movimiento_circular
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
import rospy
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
Crear una instancia de Twist
```python
vel = Twist() 
```

Funcion para utilizar eventos de teclados en Linux
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


Función principal donde se realiza el llamado a todas las demás funciones descritas anteriormente
```python
def main_function():
    # Inicializacion del nodo
    rospy.init_node("hector_quadrotor_simple_movement", anonymous=True)
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

    # Movimiento circular en el plano horizontal
    movement(0.4,0,0,0.5)

    # Imprimo mensaje de informacion
    print("Press 'q' to land the drone and finish the node\n")
    # Mensaje de advertencia
    rospy.logwarn("To start the movement, the simulation must be running\n\n")


    while(1):
        # Publicar las velocidades
        velocity_publisher.publish(vel)

        # Divisor de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
	if counter == 50:
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
