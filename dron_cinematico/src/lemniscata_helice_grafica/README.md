# Lazo Cerrado - Trayectoria Lemniscata y Gráfica

![](https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/dron_cinematico/src/lemniscata_helice_grafica/lemniscata_grafica.gif)

## Descripcion
Este nodo ejecuta el controlador para resolver el problema de seguimiento utilizando un dron en gazebo.  También tanto la trayectoria deseada como la trayectoria real del dron son publicadas en los tópicos ```/tr_path``` y ```/path```, respectivamente, para su visualización en Rviz.  
<br><br>

En la función ```velocity_controller``` la trayectoria deseada es generada y las señales de control son calculadas.  Para este ejemplo, la trayectoria deseada es una leminscata.  
<br><br>

Es necesario hacer una llamada al servicio ```/enable_motors``` para poder utilizar al dron. Tambien se definen las funciones de despegue y aterrizaje. Tanto las velocidades traslacionales $(V_x, V_y, V_z)$ y la velocidad rotacional $(W_z)$ son publicadas en el topico ```/cmd_vel```. La posicion $(x, y, z)$ y la orientacion utilizando cuterniones son recibidas del topico ```ground_truth/state```.  
<br>



## Instrucciones
Para aterrizar el dron y finalizar este nodo, presiones la tecla ```ESC```.  
<br>

En diferentes terminales:

Lanza un cuatrirrotor (my simplified drone) en Gazebo usando un mundo vacío. La simulacióm empieza pausada.
> roslaunch my_hector_uavs my_hector_uav.launch

Corre este nodo
> rosrun my_hector_uavs hector_quadrotor_controller_paths
	
Corre Rviz con mi archivo de configuración para visualizar el dron y las trajectorias
> roslaunch my_hector_uavs visualization_paths.launch  

<br><br>









## Explicación de las librerías

Para hacer la llamada al servicio ```roservice info/enable_motors```
```cpp
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" 
```

Para suscribirle a la geometria del dron (mensaje)
```cpp
#include "nav_msgs/Odometry.h"
```

Para publicar velocidades (mensaje)
```cpp
#include "geometry_msgs/Twist.h"
```

Para hacer la conversion de cuaternión a ángulo-Euler (fichero)
```cpp
#include "tf/transform_broadcaster.h"
```

Para visualizar trayectorias en Rviz
```cpp
#include "nav_msgs/Path.h" 				
```

Las últimas librerías se necesitan para utilizar eventos del teclado
```cpp
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
```
<br><br>







## Explicación del código
Se crea un namespace para no utilizar std al inicio de cada clase
```cpp
using namespace std;
```

Define objetos globales
```cpp
ros::Publisher  velocity_publisher;
ros::Subscriber pose_subscriber;
geometry_msgs::Twist vel;
ros::ServiceClient enable_motors_client;
```

Defino variable global (posición y orientación, usando ángulos de euler)
```cpp
double x, y, z, roll, pitch, yaw;
```

Configuro la altura deseada al despegar
```cpp
float takeoff_alt = 1.2;
```

Variable para indicar qué tecla se ha activó en el teclado del computador
```cpp
int key;
```

Crear objetos globales para definir la posición del dron y la trayectoria deseada usando mensajes de tipo ```geometry_msgs/Twist```
```cpp
geometry_msgs::PoseStamped drone_pose, tr_pose; 
```

Define objetos para publicar la trayectoria del dron y la trayectoria deseada en Rviz
```cpp
ros::Publisher drone_path_pub, tr_path_pub;
```

Objetos trayectoria de tipo nav_msgs/Path
```cpp
nav_msgs::Path drone_path_msg, tr_path_msg; 
```

Crear el título del marco global. El marco global es requerido por Rviz para definir el MISMO marco para cualquier robot
```cpp
const string global_frame = "/world";
```

Temporizador, tiempo inicial, velocidades de traslación máximas [m / s] y velocidades de rotación máximas [rad / s] para obtener una simulacion mas precisa a la real. (los rotores no tienen velocidades infinitas)
```cpp
double t, t0, Vxy_max = 1.5, Vz_max = 0.5, Wz_max = 4;
```

Período de trayectoria, el controlador de ganancia kx = ky = k para los errores de seguimiento 
```cpp
double T = 100, k = 0.1;
```

Defino los errores de seguimiento cartesianos, la posicion y orientacion deseada y sus respectiva derivada con respecto al tiempo
```cpp
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp;
```


Funcion para utilizar eventos de teclados en Linux
```cpp
int kbhit(void){
  struct termios oldt, newt;
  int ch, oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}
```
<br><br>


Función para llamar al servicio ```/enable_motors```, necesaria para mover el dron
```cpp
void enable_motors(){
  ROS_WARN_ONCE("Calling the enable_motors service...\n");
  hector_uav_msgs::EnableMotors enable_request;
  enable_request.request.enable = true;
  enable_motors_client.call(enable_request);

  // Muestra la respuesta del servicio
  if(enable_request.response.success){ ROS_WARN_ONCE("Enable motors successful\n"); } 
  else{ cout << "Enable motors failed" << endl; }
}
```
<br><br>


Función para obtener la postura del dron
```cpp
void poseCallback(const nav_msgs::Odometry::ConstPtr msg){
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;
```

Operaciones para convertir de cuaternión a ángulos de Euler
```cpp
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  // Los ángulos de Euler están dados en Radianes
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw); 
```

Copia la posición el dron al objeto ```drone_pose```
```cpp
  drone_pose.pose.position = msg->pose.pose.position;
```

Agregar la nueva posición a la lista
```cpp
  drone_path_msg.poses.push_back(drone_pose);
}
```
<br><br>


Función para asignar señales de control $(V_x, V_y, V_z, W_z)$
```cpp
void movement(float x, float y, float z, float turn){
  vel.linear.x = x;       vel.linear.y = y;
  vel.linear.z = z;       vel.angular.z = turn;
}
```
<br><br>


Función de despegue
```cpp
void takeoff(){

  //asiganamos una velocidad de ascenso de 0,2 m/s
  movement(0,0,0.2,0); 
  //Publicamos
  velocity_publisher.publish(vel); 

  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";

  //Utilizando el ciclo se compara si se alcanza la altitud deseada con una tolerancia de 10 cm
  while(z < takeoff_alt-0.1){  
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();

    //Se verifica si se presiona la tecla Esc para salir del nodo
    if(kbhit())	key = getchar(); 
    if(key == 27) break;
  }

  //se publica velocidad de 0 para mantener vuelo estacionario
  movement(0,0,0,0); 
  velocity_publisher.publish(vel);
}
```
<br><br>


Funcion de aterrizaje
```cpp
void land(){
  //Velocidad de descenso
  movement(0,0,-0.2,0); 
  velocity_publisher.publish(vel);

  cout << "\n Landing ...\n";

  // se compara altura y cuando este a 30 cm se asignan velocidad de 0 
  while(z > 0.3){ 
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}
```
<br><br>


Funcion que genera la trayectoria deseada y las señales de control para el dron
```cpp
void velocity_controller(){
  // Defino las variables locales 
  double Vx, Vy, Vz, Wz;

  // Defino la trayectoria deseada: Lemniscata por medio de las ecuaciones
  double a = 3, b = 1.5, X0 = 0, Y0 = -0.5, Z0 = 1.5, w = 2*M_PI/T, c = 0.5, d = M_PI/2;

  // Posicion deseada en el espacio 3d
  Xd = X0+a*sin(w*t);
  Yd = Y0+b*sin(2*w*t);
  Zd = Z0+c*sin(w*t); //Note: 1 <= Zd <= 2
  Yawd = d*sin(w*t);

  // Derivadas del tiempo correspondiente
  Xdp = a*w*cos(w*t);
  Ydp = 2*b*w*cos(2*w*t);
  Zdp = c*w*cos(w*t);
  Yawdp = d*w*cos(w*t);

  // Asignar la posición deseada al oobjeto "tr_pose"
  tr_pose.pose.position.x = Xd; tr_pose.pose.position.y = Yd; tr_pose.pose.position.z = Zd;
  // Agregar el nuevo punto a la lista
  tr_path_msg.poses.push_back(tr_pose); 


  // Cálculo los errores de seguimiento 
  ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw - Yawd; 

  // Controlador Cinemático. Señales de control auxiliares en cordenadas globales
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  //Velocidades traslacionales del dron con respecto al marco de referencia del robot
  Vx = Ux*cos(yaw)+Uy*sin(yaw);
  Vy = -Ux*sin(yaw)+Uy*cos(yaw);

  // Controlador Cinemático. Señales de control, Vz y Wz son calculadas directamente, ya que ellas son dadas con respecto al marco del robot 
  Vz = Zdp-k*ez;
  Wz = Yawdp-k*e_yaw;

  //Se realiza una saturacion ha estas velocidades  (para simular que el dron no alcanza velocidades muy grandes )
  if(abs(Vx)>Vxy_max){Vx = Vxy_max*abs(Vx)/Vx; printf("Sat Vx\t");}
  if(abs(Vy)>Vxy_max){Vy = Vxy_max*abs(Vy)/Vy; printf("Sat Vy\t");}
  if(abs(Vz)>Vz_max){Vz = Vz_max*abs(Vz)/Vz; printf("Sat Vz\t");}
  if(abs(Wz)>Wz_max){Wz = Wz_max*abs(Wz)/Wz; printf("Sat Wz\t");}

  movement(Vx,Vy,Vz,Wz);
  //publico las 4 señales de control 
  velocity_publisher.publish(vel); 
}
```
<br><br>


```cpp
int main(int argc, char **argv)
{
  // Inicializar el nodo
  ros::init(argc, argv, "hector_quadrotor_controller_paths"); 
  // Crear el nodo Handle 
  ros::NodeHandle n; 
  // Asignar frecuencia (Hz) de ejecucion al nodo
  int freq = 50; 
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;


  // Se configura  y llama al servicio
  enable_motors_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  // Publica en el topico velocidades utilizando el mensaje geometry 
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
  // Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta
  pose_subscriber = n.subscribe("/ground_truth/state", 1, poseCallback);  

  // Publicar las trayectorias
  drone_path_pub = n.advertise<nav_msgs::Path>("/path", 10);
  tr_path_pub = n.advertise<nav_msgs::Path>("/tr_path", 10);

  //Importante: Asignación del MIMSMO marco de referencia para TODOS los robots
  drone_path_msg.header.frame_id = tr_path_msg.header.frame_id = global_frame;


  //Ejecuta las funciones "enable_motors" y "takeoff"
  enable_motors(); 
  takeoff();
 
  // Imprimo mensaje de informacion
  printf("Press 'ESC' to land the drone and finish the node");
  printf("To clean up both paths, press 'c' key\n");
  // Mensaje de advertencia
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); 
 
  // Obtengo el tiempo inical 
  t0 = ros::Time::now().toSec();

  do{
    // Calculo el tiempo del controlador
    t = ros::Time::now().toSec()-t0; //printf("t: %.1f\n",t);      
    // Calculo las señales de control
    velocity_controller(); 

    // Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
    if(counter == 50){ 
      // Imprimier en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      printf("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw); 
      //Resetear el contador
      counter = 0; 
    }else counter++;

    // Publica los mensajes "path"
    drone_path_pub.publish(drone_path_msg); tr_path_pub.publish(tr_path_msg);

    //Requerido para recibir funciones de devolución de llamada (call back)
    ros::spinOnce();    
    //Comando para esperar el resto del tiempo para completar la frecuencia del bucle
    loop_rate.sleep();   
    if(kbhit())	key = getchar();

    // Limpiar ambas trayectorias
    if(key == 'c'){ 
      // Limpiar mensajes de trayectoria
      drone_path_msg.poses.clear(); tr_path_msg.poses.clear(); 
      ROS_WARN_ONCE("Clear paths\n");
      //Resetear la tecla
      key = 0; 
    }
    if(key == 27) finish = true;
  }while(finish == false); 
  //Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros
  
  // Ejecuta la función Aterrizar dron
  land(); 
  printf("\n Node finished\n");
  return 0;
}
```
