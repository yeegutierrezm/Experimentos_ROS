# Lazo Cerrado - Trayectoria Lemniscata

![](https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/dron_cinematico/src/lemniscata_helice/lemniscata.gif)
   
Este nodo ejecuta el controlador para resolver el problema de seguimiento utilizando un dron en gazebo. en la funcion "velocity_controller" tanto la trayectoria deseada como las señales de control son calculadas, para este ejemplo la trayectoria deseada es una lemniscata.  
<br><br>

Es necesario hacer una llamada al servicio ```/enable_motors``` para poder utilizar al dron. Tambien se definen las funciones de despegue y aterrizaje. Tanto las velocidades traslacionales $(V_x, V_y, V_z)$ y la velocidad rotacional $(W_z)$ son publicadas en el topico ```/cmd_vel```. La posicion $(x, y, z)$ y la orientacion utilizando cuterniones son recibidas del topico ```ground_truth/state```.  
<br>

Para aterrizar el dron y terminar este nodo es necesario presionar la tecla ```ESC```.  
<br><br>
   


## Instrucciones

Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

> roslaunch my_hector_uavs my_hector_uav.launch

Correr el nodo
> rosrun my_hector_uavs hector_quadrotor_controller

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

  // Cálculo los errores de seguimiento 
  ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw - Yawd; 

  //Señales de control auxiliares en cordenadas globales
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  //Velocidades traslacionales del dron con respecto al marco de referencia del robot
  Vx = Ux*cos(yaw)+Uy*sin(yaw);
  Vy = -Ux*sin(yaw)+Uy*cos(yaw);

  //Señales de control, estas se calculan directamente ya que son con respecto a la estructura del robot 
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


Función principal donde se realiza el llamado a todas las demás funciones descritas anteriormente
```cpp
int main(int argc, char **argv)
{
  // Inicializacion del nodo
  ros::init(argc, argv, "hector_quadrotor_simple_movement");
  // Crear el nodo encargado  
  ros::NodeHandle n; 
  // Asignar frecuencia (Hz) de ejecucion al nodo
  int freq = 50; 
  ros::Rate loop_rate(freq);
  // Se crea un contador
  int counter = 0; 
  // Indica si el nodo a terminado
  bool finish = false; 


  // Se configura  y llama al servicio
  enable_motors_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors"); 
  // Publica en el topico velocidades utilizando el mensaje geometry 
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);  
  // Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta 
  pose_subscriber = n.subscribe("/ground_truth/state", 1, poseCallback); 


  // Ejecuta y llama la funcion que llama al servicio enable 
  enable_motors(); 
  // Despega el dron 
  takeoff();
 
  // Imprimo mensaje de informacion
  printf("Press 'ESC' to land the drone and finish the node\n"); 
  // Mensaje de advertencia
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); 

  // Obtengo el tiempo inical 
  t0 = ros::Time::now().toSec(); 

  do{
    // Calculo el tiempo del controlador   
    t = ros::Time::now().toSec()-t0; 
    //Calculo las señales de control
    velocity_controller(); 

    //Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
    if(counter == 50){ 
      // Imprimir en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      printf("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw); 
      //Reseteo el contador
      counter = 0; 
    }else counter++;

    //Requerido para recibir funciones de devolución de llamada (call back)
    ros::spinOnce();   
    //Comando para esperar el resto del tiempo para completar la frecuencia del bucle 
    loop_rate.sleep();  
    if(kbhit())	key = getchar();

    if(key == 27) finish = true;
  }while(finish == false); 
  //Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros 
  
  //Funcion aterrizar el dron 
  land(); 
  printf("\n Node finished\n");
  return 0;
}
```
