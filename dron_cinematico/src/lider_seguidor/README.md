# Lazo Cerrado - Trayectoria, Líder-Seguidor

![](https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/dron_cinematico/src/lider_seguidor/lider_seguidor.gif)

Este nodo ejecuta el controlador para resolver el problema del control de formacion usando el esquema lider-seguidor con 2 drones simulados en gazebo. El dron lider sigue su trayectoria deseada utilizando la funcion ```velocity controller``` y entonces el robot seguidor sigue la trayectoria desfasada del robot lider.  
<br><br>

Primero se hace una llamda al servicio ```/enable_motors``` para cada uno d elos drones. Se definden las funciones de despegue y aterrizaje para ambos drones.  
<br>

Para aterrizar ambos drones y terminar este nodo, se necesita presioanr la tecla ```ESC```.  
<br><br> 

## Instrucciones:

En diferentes terminales:

Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

> roslaunch my_hector_uavs two_hector_uavs.launch

Corre el nodo
> rosrun my_hector_uavs two_hector_quadrotors_lf  

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

Se crea una estructura personaliazda para definir la posicion y orientacion global del dron utilizando los angulos de euler
```cpp
struct robot_pose{ double x, y, z, roll, pitch, yaw; }; 
```

Define objetos globales
```cpp
ros::Publisher  velocity_publisher;
ros::Subscriber pose_subscriber;
geometry_msgs::Twist vel;
ros::ServiceClient enable_motors_client;
```

Definir variables globales (posición y orientación, usando mi estructura personalizada)
```cpp
robot_pose r_pose1, r_pose2; 
```

Variable para indicar qué tecla se ha activó en el teclado del computador
```cpp
int key;
```

Configuro la altura deseada al despegar
```cpp
float takeoff_alt = 0.5;


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

Señales de control para el lider 
```cpp
double Vxl, Vyl, Vzl, Wzl; 
```

Defino variables robot seguidor. Error de seguimiento y estado de formacion deseado 
```cpp
double exf, eyf, ezf, e_yawf, s_d = 1.5, alp_d = M_PI, s_zd = 0; 
```

Señales de control para el seguidor
```cpp
double Vxf, Vyf, Vzf, Wzf;  
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


Función para llamar al servicio ```/enable_motors```, necesaria para mover a los 2 drones
```cpp
void enable_motors(){
  ROS_WARN_ONCE("Calling the enable_motors services ...\n");
   // Llamo a los dos sercicios enable para cada dron 
  hector_uav_msgs::EnableMotors enable_request1, enable_request2;

  enable_request1.request.enable = true;
  enable_motors_client1.call(enable_request1);
  if(enable_request1.response.success){ cout << "uav1: enable motors successful\n"; }
  else{ cout << "uav1: enable motors failed" << endl; }

  enable_request2.request.enable = true;
  enable_motors_client2.call(enable_request2);
  if(enable_request2.response.success){ cout << "uav2: enable motors successful\n"; }
  else{ cout << "uav2: enable motors failed" << endl; }
}
```
<br><br>


Función para obtener la postura del dron 1
```cpp
void poseCallback1(const nav_msgs::Odometry::ConstPtr msg){
  r_pose1.x = msg->pose.pose.position.x;
  r_pose1.y = msg->pose.pose.position.y;
  r_pose1.z = msg->pose.pose.position.z;
```

Operaciones para convertir de cuaternión a ángulos de Euler
```cpp
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  // Los ángulos de Euler se expresan en radianes
  tf::Matrix3x3(quater).getRPY(r_pose1.roll, r_pose1.pitch, r_pose1.yaw);  
}
```

Función para obtener la postura del dron 2
Operaciones para convertir de cuaternión a ángulos de Euler
```cpp
void poseCallback2(const nav_msgs::Odometry::ConstPtr msg){
  r_pose2.x = msg->pose.pose.position.x;
  r_pose2.y = msg->pose.pose.position.y;
  r_pose2.z = msg->pose.pose.position.z;
```

```cpp
Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  // Los ángulos de Euler se expresan en radianes
  tf::Matrix3x3(quater).getRPY(r_pose2.roll, r_pose2.pitch, r_pose2.yaw);  
}
```
<br><br>



Función para asignar señales de control y publicarlas, drone 1
```cpp
void move1(float x, float y, float z, float turn){ 
  vel1.linear.x = x; vel1.linear.y = y; vel1.linear.z = z; vel1.angular.z = turn;
  vel_pub1.publish(vel1);
}
```

Función para asignar señales de control y publicarlas, drone 2
```cpp
void move2(float x, float y, float z, float turn){ 
  vel2.linear.x = x; vel2.linear.y = y; vel2.linear.z = z; vel2.angular.z = turn;
  vel_pub2.publish(vel2);
}
```
<br><br>


Función de despegue de los 2 drones
```cpp
void takeoff(){
  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";

  // Quadrotor 1 - Asigno velocidades de ascenso
  move1(0,0,0.3,0); 
  
  //Compare si se alcanza la altitud deseada
  while(r_pose1.z < takeoff_alt-0.1){ 
    vel_pub1.publish(vel1);
    ros::spinOnce(); ros::Duration(0.1).sleep();
    //Compruebe cualquier pulsación de tecla
    if(kbhit())	key = getchar(); 
    if(key == 27) break;
  }
  //Detener
  move1(0,0,0,0); 

  //Quadrotor 2 - Asigno velocidades de ascenso
  move2(0,0,0.3,0); 

  //Compare si se alcanza la altitud deseada
  while(r_pose2.z < takeoff_alt-0.1){ 
    vel_pub2.publish(vel2);
    // Importante: las velocidades del dron 1 deben seguir publicándose para evitar un aterrizaje no deseado
    vel_pub1.publish(vel1); 
    ros::spinOnce(); ros::Duration(0.1).sleep();
    //Compruebe cualquier pulsación de tecla
    if(kbhit())	key = getchar(); 
    if(key == 27) break;
  }
  move2(0,0,0,0); 
  // Detener
}
```
<br><br>


Funcion de aterrizaje de todos los drones 
```cpp
void land(){ 
  cout << "\n Landing ...\n";

  //Asigno velocidades de descenso
  move1(0,0,-0.3,0); move2(0,0,-0.3,0); 

  while(r_pose1.z > 0.3 || r_pose2.z > 0.3){
    vel_pub1.publish(vel1);  vel_pub2.publish(vel2);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  //Apaga todos los drones
  move1(0,0,0,0);  move2(0,0,0,0); 
}
```
<br><br>


## Función leader_vel_control()
Función para generar la trayectoria deseada y calcular el control de señales del robot líder
```cpp
void leader_vel_control(){
  // Definir la trayectoria deseada:
  double X0 = 0, Y0 = 0, Z0 = 0.5, radius = 3, w = 2*M_PI/T, Vzd = 0.02;
```

Posición deseada en el espacio 3D
```cpp
  Xd = X0+radius*sin(w*t);
  Yd = Y0+radius*cos(w*t);
  //aumento lineal
  Zd = Z0+Vzd*t; 
  //comportamiento sinusoidal
  Yawd = sin(w*t); 
```
Derivadas de tiempo correspondientes
```cpp
  Xdp = radius*w*cos(w*t);
  Ydp = -radius*w*sin(w*t);
  Zdp = Vzd;
  Yawdp = w*cos(w*t);
```

Calcular errores de seguimiento
```cpp
  ex = r_pose1.x-Xd; ey = r_pose1.y-Yd; ez = r_pose1.z-Zd; e_yaw = r_pose1.yaw - Yawd; 
```

Controlador cinematográfico. Controles auxiliares, en coordenadas globales (variables locales)
```cpp 
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
```

Velocidades de traslación con respecto a la estructura del robot
```cpp
  Vxl = Ux*cos(r_pose1.yaw)+Uy*sin(r_pose1.yaw);
  Vyl = -Ux*sin(r_pose1.yaw)+Uy*cos(r_pose1.yaw);
```

Controlador cinematográfico. Nota: Vz y Wz se calculan directamente ya que se dan con respecto a la estructura del robot.
```cpp  
  Vzl = Zdp-k*ez;
  Wzl = Yawdp-k*e_yaw;
```

 Velocidad de saturacion
```cpp  
  if(abs(Vxl)>Vxy_max){Vxl = Vxy_max*abs(Vxl)/Vxl; printf("Sat Vxl\t");}
  if(abs(Vyl)>Vxy_max){Vyl = Vxy_max*abs(Vyl)/Vyl; printf("Sat Vyl\t");}
  if(abs(Vzl)>Vz_max){Vzl = Vz_max*abs(Vzl)/Vzl; printf("Sat Vzl\t");}
  if(abs(Wzl)>Wz_max){Wzl = Wz_max*abs(Wzl)/Wzl; printf("Sat Wzl\t");}
```
Publica las 4 señales de control
```cpp
  move1(Vxl,Vyl,Vzl,Wzl);
}
```
<br><br>


## Función follower_vel_control()

Función para calcular el control de señales del robot seguidor.

Nota: la posición deseada del seguidor y la orientación de guiñada vienen dadas por los del líder.

```cpp
void follower_vel_control(){
  //Crea algunas variables locales
  double Xfd, Yfd, Xfdp, Yfdp, xlp, ylp, Ux, Uy; 
```

Posición deseada del Seguidor con respecto al marco global
```cpp  
  Xfd = r_pose1.x+s_d*cos(r_pose1.yaw+alp_d);
  Yfd = r_pose1.y+s_d*sin(r_pose1.yaw+alp_d);
```

Velocidades de traslación del dron líder en el marco global
```cpp  
  xlp = Vxl*cos(r_pose1.yaw)-Vyl*sin(r_pose1.yaw);
  ylp = Vxl*sin(r_pose1.yaw)+Vyl*cos(r_pose1.yaw);
```

Derivadas de tiempo deseadas correspondientes
```cpp  
  Xfdp = xlp-Wzl*s_d*sin(r_pose1.yaw+alp_d);
  Yfdp = ylp+Wzl*s_d*cos(r_pose1.yaw+alp_d);
```

Calcular los errores de seguimiento (con respecto al marco global)
```cpp  
  exf = r_pose2.x-Xfd; 			eyf = r_pose2.y-Yfd; 
  ezf = r_pose2.z-(r_pose1.z+s_zd); 	e_yawf = r_pose2.yaw - r_pose1.yaw; //Tanto la altitud como la orientación son iguales a las del drone líder.
```

Controlador cinematico. Controles auxiliares, en coordenadas globales
```cpp
  Ux = Xfdp-k*exf; 	Uy = Yfdp-k*eyf;
 ```

Velocidades de traslación con respecto a la estructura del robot
```cpp
  Vxf = Ux*cos(r_pose2.yaw)+Uy*sin(r_pose2.yaw);
  Vyf = -Ux*sin(r_pose2.yaw)+Uy*cos(r_pose2.yaw);
```

Velocidades verticales y de rotación
```cpp
  Vzf = Vzl-k*ezf;
  Wzf = Wzl-k*e_yawf;
```

Velocidad de saturacion
```cpp  
  if(abs(Vxf)>Vxy_max){Vxf = Vxy_max*abs(Vxf)/Vxf; printf("Sat Vxf\t");}
  if(abs(Vyf)>Vxy_max){Vyf = Vxy_max*abs(Vyf)/Vyf; printf("Sat Vyf\t");}
  if(abs(Vzf)>Vz_max){Vzf = Vz_max*abs(Vzf)/Vzf; printf("Sat Vzf\t");}
  if(abs(Wzf)>Wz_max){Wzf = Wz_max*abs(Wzf)/Wzf; printf("Sat Wzf\t");}
```

Publica las 4 señales de control
```cpp  
  move2(Vxf,Vyf,Vzf,Wzf); 
}
```
<br><br>


## Función Principal main()

```cpp  
int main(int argc, char **argv)
{    
  //Inicializo el nodo
  ros::init(argc, argv, "two_hector_quadrotors_lf"); 
  //Creo el identificador del nodo 
  ros::NodeHandle n; 
  //frecuencia del nodo (Hz)
  int freq = 50; 
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;
```

Llamo al servicio
```cpp   
  enable_motors_client1 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav1/enable_motors"); 
  enable_motors_client2 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav2/enable_motors");
```

Publicar en el topico
```cpp   
  vel_pub1 = n.advertise<geometry_msgs::Twist>("uav1/cmd_vel", 1); 
  vel_pub2 = n.advertise<geometry_msgs::Twist>("uav2/cmd_vel", 1);
```

Suscribirse al  topic
```cpp   
  pose_sub1 = n.subscribe("uav1/ground_truth/state", 1, poseCallback1); 
  pose_sub2 = n.subscribe("uav2/ground_truth/state", 1, poseCallback2);
```

Ejecuta las funciones "enable_motors" y "takeoff" (para todos los robots)
```cpp
  enable_motors();
  takeoff();
```

```cpp
  printf(" Press 'ESC' to land the drones and finish the node\n");
  //Mensaje de advertencia
  ROS_WARN_ONCE(" To start the movement, the simulation must be running\n\n"); 
```

Obtener el tiempo inicial
```cpp   
  t0 = ros::Time::now().toSec(); 
```

```cpp 
  do{
    //Calculo el tiempo del controlador  
    t = ros::Time::now().toSec()-t0;    
    //Calculo las señales de control del robot lider
    leader_vel_control(); 
    //Calculo las señales de control del robot seguidor
    follower_vel_control(); 
```

Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
```cpp    
    if(counter == 100){ 
      // Imprimier en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      printf("\nex: %.3f\tey: %.3f\tez: %.2f\te_yaw: %.2f\n", ex,ey,ez,e_yaw); 
      //Errores de formación del seguidor de impresión y error de orientación
      printf("exf: %.3f\teyf: %.3f\tezf: %.2f\te_yawf: %.2f\n", exf,eyf,ezf,e_yawf); 
      
      // Reseteo el contador
      counter = 0; 
    }else counter++;
```

Requerido para recibir funciones de devolución de llamada (call back)
```cpp  
    ros::spinOnce();    
    //Comando para esperar el resto del tiempo para completar la frecuencia del bucle
    loop_rate.sleep();  
    if(kbhit())	key = getchar();

    if(key == 27) finish = true;
  }while(finish == false); 
  //Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros 
```

```cpp 
  //Funcion aterrizar el dron 
  land(); 
  printf("\n Node finished\n");
  return 0;
}
```
