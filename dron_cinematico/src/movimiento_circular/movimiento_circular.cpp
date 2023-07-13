/* 

Resumen:
Este codigo es un ejemplo para publicar velocidades a un robot simulado y suscribirse a su odometria.

Descripcion:
Este nodo publica velocidades en lazo abierto al dron simulado en gazebo, y se suscribe a su odometria para obtener la posicion y la orientacion del cuatrirrotor.

1. Es necesario hacer una llamada al servicio "/enable_motors" para poder utilizar al dron. Tambien se definen las funciones de despegue y aterrizaje. Tanto las velocidades traslacionales (Vx, Vy, Vz) y la velocidad rotacional (Wz) son publicadas en el topico "/cmd_vel". La posicion (x, y, z) y la orientacion utilizando cuterniones son recibidas del topico "ground_truth/state"


Para aterrizar el dron y terminar este nodo es necesario presionar la tecla ESC

Instrucciones:

$ roslaunch my_hector_uavs my_hector_uav.launch
	# Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

$ rosrun my_hector_uavs hector_quadrotor_simple_movement
	# Correr el nodo


Se deben agregar los archivos de biblioteca
*/
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" //Hacer la llamada al servicio --> roservice info/enable_motors
#include "nav_msgs/Odometry.h" // Para suscribirle a la geometria del dron (mensaje)
#include "geometry_msgs/Twist.h" //Para publicar velocidades (mensaje)
#include "tf/transform_broadcaster.h" //Para hacer la conversion de cuaternion a angulo-Euler (fichero)
#include <iostream>
//los ultimos 3 se necesitan para utilizar eventos del teclado.
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//Se crea un namespace para no utilizar std al inicio de cada clase
using namespace std;

//Define objetos globales
ros::Publisher  velocity_publisher;
ros::Subscriber pose_subscriber;
geometry_msgs::Twist vel;
ros::ServiceClient enable_motors_client;

int key; //Variable para guardar el valor de la clave
double x, y, z, roll, pitch, yaw; //Defino variable global (posicion y orientacion, usando euler-angulo)
float takeoff_alt = 1.2; // configuro la altura deseada al despegar

int kbhit(void){ //Funcion para utilizar eventos de teclados en Linux
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

void enable_motors(){ //Función para llamar al servicio "/ enable_motors", necesaria para mover el dron
  ROS_WARN_ONCE("Calling the enable_motors service...\n");
  hector_uav_msgs::EnableMotors enable_request;
  enable_request.request.enable = true;
  enable_motors_client.call(enable_request);

  if(enable_request.response.success){ ROS_WARN_ONCE("Enable motors successful\n"); } // muestra la respuesta del servicio
  else{ cout << "Enable motors failed" << endl; }
}

void poseCallback(const nav_msgs::Odometry::ConstPtr msg){ //Función de devolución de llamada (call back) para obtener la postura del dron.
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;
  //Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);  //Los ángulos de Euler se expresan en radianes.
}

void movement(float x, float y, float z, float turn){ //Función para asignar señales de control (Vx, Vy, Vz, Wz)
  vel.linear.x = x;       vel.linear.y = y;
  vel.linear.z = z;       vel.angular.z = turn;
}

void takeoff(){ //Funcion de despegue
  movement(0,0,0.2,0); //asiganamos una velocidad de ascenso de 0,2 m/s
  velocity_publisher.publish(vel); //Publicamos

  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";
  while(z < takeoff_alt-0.1){ //Utilizando el ciclo se compara si se alcanza la altitud deseada con una tolerancia de 10 cm 
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); //Se verifica si se presiona la tecla Esc para salir del nodo
    if(key == 27) break;
  }
  movement(0,0,0,0); //se publica velocidad de 0 para mantener vuelo estacionario
  velocity_publisher.publish(vel);
}

void land(){ //Funcion de aterrizaje
  movement(0,0,-0.2,0); //Velocidad de descenso
  velocity_publisher.publish(vel);

  cout << "\n Landing ...\n";
  while(z > 0.3){ // se compara altura y cuando este a 30 cm se asignan velocidad de 0 
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_quadrotor_simple_movement");  //Inicializacion del nodo
  ros::NodeHandle n; //Crear el nodo encargado
  int freq = 50; //Asignar frecuencia (Hz) de ejecucion al nodo
  ros::Rate loop_rate(freq);
  int counter = 0; // Se crea un contador
  bool finish = false; //Indica si el nodo a termiando

  enable_motors_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors"); // Se configura  y llama al servicio
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // Publica en el topico velocidades utilizando el mensaje geometry  
  pose_subscriber = n.subscribe("/ground_truth/state", 1, poseCallback); //Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta 

  enable_motors(); //Ejecuta y llama la funcion que llama al servicio enable 
  takeoff(); //Despega el dron 

  //Movimiento circular en el plano horizontal.
  movement(0.4,0,0,0.5);
 
  printf("Press 'ESC' to land the drone and finish the node\n"); // imprimo mensaje de informacion
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); //Mensaje de advertencia

  do{
    velocity_publisher.publish(vel); //Publico las velocidades
    if(counter == 50){ //Divisior de frecuencia para no imprimir todo el tiempo
      printf("x: %.3f y: %.3f z: %.3f roll: %.3f pitch: %.3f yaw: %.3f\n", x,y,z, roll,pitch,yaw);
      //Imprimir en la terminal la posición [m] y orientación [rad]
      counter = 0; //Reiniciar contador
    }else counter++;

    ros::spinOnce();    //Requerido para recibir funciones de devolución de llamada (call back)
    loop_rate.sleep();  //Comando para esperar el resto del tiempo para completar la frecuencia del bucle
    if(kbhit())	key = getchar(); //Se compara si se ha presionado alguna tecla

    if(key == 27) finish = true; //ESC es la tecla = 27 in codigo Ascii, si se presiona Esc la bandera cambia
  }while(finish == false); //Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros 
  
  land(); //aterrizar el dron 
  printf("\n Node finished\n");
  return 0;
}

