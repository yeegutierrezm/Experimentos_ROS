/* 
@descripcion:

Este nodo ejecuta el controlador para resolver el problema del control de formacion usando el esquema lider-seguidor con 2 drones simulados en gazebo. El dron lider sigue su trayectoria deseada utilizando la funcion "velocity controller" y entonces el robot seguidor sigue la trayectoria desfasada del robot lider. 

primero se hace una llamda al servicio "/enable_motors" para cada uno d elos drones. Se definden las funciones de despegue y aterrizaje para ambos drones.

-- para aterrizar ambos drones y termianr este nodo, se necesita presioanr la tecla 'ESC'.

Instrucciones:
En diferentes terminales:
$ roslaunch my_hector_uavs two_hector_uavs.launch
	# Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

$ rosrun my_hector_uavs two_hector_quadrotors_lf
	# Corre el nodo
*/

// se agregan los archivos de cabecera 
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" //Hacer la llamada al servicio --> roservice info/enable_motors
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
//los ultimos 3 se necesitan para utilizar eventos del teclado.
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;
// se crea una estructura personaliazda para definir la posicion y orientacion global del dron utilizando los angulos de euler
struct robot_pose{ double x, y, z, roll, pitch, yaw; };  

//Se definen objetos globales 
ros::Publisher  vel_pub1, vel_pub2; //Publicar velocidades
ros::Subscriber pose_sub1, pose_sub2; //Suscribirse a la odometria 
geometry_msgs::Twist vel1, vel2;
ros::ServiceClient enable_motors_client1, enable_motors_client2; // Hacer la llamda al servicio enble motors

int key; //Variable para guardar el valor de la clave
robot_pose r_pose1, r_pose2; //Definir variables globales (posición y orientación, usando mi estructura personalizada)
float takeoff_alt = 0.5; // Altitud inicial deseada

double t, t0, Vxy_max = 2, Vz_max = 0.5, Wz_max = 8; //Temporizador, tiempo inicial, velocidades de traslación máximas [m / s] y velocidades de rotación máximas [rad / s]

//Variables de drones líderes
double T = 100, k = 0.1; //Período de trayectoria, ganancia del controlador  kx = ky = k
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; //Seguimiento de errores, posiciones deseadas, orientación deseada y derivada del tiempo, respectivamente
double Vxl, Vyl, Vzl, Wzl; //señales de control para el lider 

//defino variables robot seguidor
double exf, eyf, ezf, e_yawf, s_d = 1.5, alp_d = M_PI, s_zd = 0; // error de seguimiento y estado de formacion deseado 
double Vxf, Vyf, Vzf, Wzf;  //Señales de control


int kbhit(void){ // funcion función de Windows implementada para Linux
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

void enable_motors(){ //Se llama a la funcion "/enable_motors" ,servicio necesario para mover al dron
  ROS_WARN_ONCE("Calling the enable_motors services ...\n");
  hector_uav_msgs::EnableMotors enable_request1, enable_request2; // llamo a los dos sercicios enable para cada dron 

  enable_request1.request.enable = true;
  enable_motors_client1.call(enable_request1);
  if(enable_request1.response.success){ cout << "uav1: enable motors successful\n"; }
  else{ cout << "uav1: enable motors failed" << endl; }

  enable_request2.request.enable = true;
  enable_motors_client2.call(enable_request2);
  if(enable_request2.response.success){ cout << "uav2: enable motors successful\n"; }
  else{ cout << "uav2: enable motors failed" << endl; }
}

void poseCallback1(const nav_msgs::Odometry::ConstPtr msg){ //Función de devolución de llamada para obtener la postura del dron 1
  r_pose1.x = msg->pose.pose.position.x;
  r_pose1.y = msg->pose.pose.position.y;
  r_pose1.z = msg->pose.pose.position.z;
  //Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(r_pose1.roll, r_pose1.pitch, r_pose1.yaw);  //Los ángulos de Euler se expresan en radianes.
}

void poseCallback2(const nav_msgs::Odometry::ConstPtr msg){ //Función de devolución de llamada para obtener la postura del dron 2
  r_pose2.x = msg->pose.pose.position.x;
  r_pose2.y = msg->pose.pose.position.y;
  r_pose2.z = msg->pose.pose.position.z;
  //Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(r_pose2.roll, r_pose2.pitch, r_pose2.yaw);  //Los ángulos de Euler se expresan en radianes.
}

void move1(float x, float y, float z, float turn){ //Función para asignar señales de control y publicarlas, drone 1
  vel1.linear.x = x; vel1.linear.y = y; vel1.linear.z = z; vel1.angular.z = turn;
  vel_pub1.publish(vel1);
}
void move2(float x, float y, float z, float turn){ ///Función para asignar señales de control y publicarlas, drone 2
  vel2.linear.x = x; vel2.linear.y = y; vel2.linear.z = z; vel2.angular.z = turn;
  vel_pub2.publish(vel2);
}

void takeoff(){ //Función de despegue de todos los drones.
  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";

  //Quadrotor 1
  move1(0,0,0.3,0); //Asigno velocidades de ascenso
  
  while(r_pose1.z < takeoff_alt-0.1){ //Compare si se alcanza la altitud deseada
    vel_pub1.publish(vel1);
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); //Compruebe cualquier pulsación de tecla
    if(key == 27) break;
  }
  move1(0,0,0,0); //Detener

  //Quadrotor 2
  move2(0,0,0.3,0); //Asigno velocidades de ascenso

  while(r_pose2.z < takeoff_alt-0.1){ //Compare si se alcanza la altitud deseada
    vel_pub2.publish(vel2);
    vel_pub1.publish(vel1); // Importante: las velocidades del dron 1 deben seguir publicándose para evitar un aterrizaje no deseado
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); //Compruebe cualquier pulsación de tecla
    if(key == 27) break;
  }
  move2(0,0,0,0); //detener
}

void land(){ //Funcion de aterrizaje de todos los drones 
  cout << "\n Landing ...\n";

  move1(0,0,-0.3,0); move2(0,0,-0.3,0); //Asigno velocidades de descenso

  while(r_pose1.z > 0.3 || r_pose2.z > 0.3){
    vel_pub1.publish(vel1);  vel_pub2.publish(vel2);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  move1(0,0,0,0);  move2(0,0,0,0); //Apaga todos los drones
}

void leader_vel_control(){ //Función para generar la trayectoria deseada y calcular el control de señales del robot líder
  // Definir la trayectoria deseada:
  double X0 = 0, Y0 = 0, Z0 = 0.5, radius = 3, w = 2*M_PI/T, Vzd = 0.02;

  //Posición deseada en el espacio 3D
  Xd = X0+radius*sin(w*t);
  Yd = Y0+radius*cos(w*t);
  Zd = Z0+Vzd*t; //aumento lineal
  Yawd = sin(w*t); //comportamiento sinusoidal

  //Derivadas de tiempo correspondientes
  Xdp = radius*w*cos(w*t);
  Ydp = -radius*w*sin(w*t);
  Zdp = Vzd;
  Yawdp = w*cos(w*t);


  ex = r_pose1.x-Xd; ey = r_pose1.y-Yd; ez = r_pose1.z-Zd; e_yaw = r_pose1.yaw - Yawd; //Calcular errores de seguimiento

  //Controlador cinematográfico. Controles auxiliares, en coordenadas globales (variables locales)
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  //Velocidades de traslación con respecto a la estructura del robot
  Vxl = Ux*cos(r_pose1.yaw)+Uy*sin(r_pose1.yaw);
  Vyl = -Ux*sin(r_pose1.yaw)+Uy*cos(r_pose1.yaw);

  // Controlador cinematográfico. Nota: Vz y Wz se calculan directamente ya que se dan con respecto a la estructura del robot.
  Vzl = Zdp-k*ez;
  Wzl = Yawdp-k*e_yaw;

  //Velocidad de saturacion
  if(abs(Vxl)>Vxy_max){Vxl = Vxy_max*abs(Vxl)/Vxl; printf("Sat Vxl\t");}
  if(abs(Vyl)>Vxy_max){Vyl = Vxy_max*abs(Vyl)/Vyl; printf("Sat Vyl\t");}
  if(abs(Vzl)>Vz_max){Vzl = Vz_max*abs(Vzl)/Vzl; printf("Sat Vzl\t");}
  if(abs(Wzl)>Wz_max){Wzl = Wz_max*abs(Wzl)/Wzl; printf("Sat Wzl\t");}

  move1(Vxl,Vyl,Vzl,Wzl); //Publica las 4 señales de control
}

void follower_vel_control(){ // Función para calcular el control de señales del robot seguidor
  //Nota: la posición deseada del seguidor y la orientación de guiñada vienen dadas por los del líder.
  
  double Xfd, Yfd, Xfdp, Yfdp, xlp, ylp, Ux, Uy; //Crea algunas variables locales

  //Seguidor de la posición deseada con respecto al marco global
  Xfd = r_pose1.x+s_d*cos(r_pose1.yaw+alp_d);
  Yfd = r_pose1.y+s_d*sin(r_pose1.yaw+alp_d);

  //Velocidades de traslación del dron líder en el marco global
  xlp = Vxl*cos(r_pose1.yaw)-Vyl*sin(r_pose1.yaw);
  ylp = Vxl*sin(r_pose1.yaw)+Vyl*cos(r_pose1.yaw);

  //Derivadas de tiempo deseadas correspondientes
  Xfdp = xlp-Wzl*s_d*sin(r_pose1.yaw+alp_d);
  Yfdp = ylp+Wzl*s_d*cos(r_pose1.yaw+alp_d);

  //Calcular los errores de seguimiento (con respecto al marco global)
  exf = r_pose2.x-Xfd; 			eyf = r_pose2.y-Yfd; 
  ezf = r_pose2.z-(r_pose1.z+s_zd); 	e_yawf = r_pose2.yaw - r_pose1.yaw; //Tanto la altitud como la orientación son iguales a las del drone líder.
  
  //Controlador cinematico. Controles auxiliares, en coordenadas globales
  Ux = Xfdp-k*exf; 	Uy = Yfdp-k*eyf;
  
  // Velocidades de traslación con respecto a la estructura del robot
  Vxf = Ux*cos(r_pose2.yaw)+Uy*sin(r_pose2.yaw);
  Vyf = -Ux*sin(r_pose2.yaw)+Uy*cos(r_pose2.yaw);

  //Velocidades verticales y de rotación
  Vzf = Vzl-k*ezf;
  Wzf = Wzl-k*e_yawf;

  //Velocidad de saturacion
  if(abs(Vxf)>Vxy_max){Vxf = Vxy_max*abs(Vxf)/Vxf; printf("Sat Vxf\t");}
  if(abs(Vyf)>Vxy_max){Vyf = Vxy_max*abs(Vyf)/Vyf; printf("Sat Vyf\t");}
  if(abs(Vzf)>Vz_max){Vzf = Vz_max*abs(Vzf)/Vzf; printf("Sat Vzf\t");}
  if(abs(Wzf)>Wz_max){Wzf = Wz_max*abs(Wzf)/Wzf; printf("Sat Wzf\t");}

  move2(Vxf,Vyf,Vzf,Wzf); //Publica las 4 señales de control
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "two_hector_quadrotors_lf");  //Inicializo el nodo
  ros::NodeHandle n; //Creo el identificador del nodo
  int freq = 50; //frecuencia del nodo (Hz)
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;

  enable_motors_client1 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav1/enable_motors"); //llamo al servicio
  enable_motors_client2 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav2/enable_motors");
  vel_pub1 = n.advertise<geometry_msgs::Twist>("uav1/cmd_vel", 1); //publicar en el topico
  vel_pub2 = n.advertise<geometry_msgs::Twist>("uav2/cmd_vel", 1);
  pose_sub1 = n.subscribe("uav1/ground_truth/state", 1, poseCallback1); //suscribirse al  topic
  pose_sub2 = n.subscribe("uav2/ground_truth/state", 1, poseCallback2);

  enable_motors(); //Ejecute las funciones "enable_motors" y "takeoff" (para todos los robots)

  takeoff();
 
  printf(" Press 'ESC' to land the drones and finish the node\n");
  ROS_WARN_ONCE(" To start the movement, the simulation must be running\n\n"); //Mensaje de advertencia


  t0 = ros::Time::now().toSec(); //Obtener el tiempo inicial

  do{
    t = ros::Time::now().toSec()-t0; //Calculo el tiempo del controlador     
    leader_vel_control(); //Calculo las señales de control del robot lider
    follower_vel_control(); //Calculo las señales de control del robot seguidor

    if(counter == 100){ //Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
      printf("\nex: %.3f\tey: %.3f\tez: %.2f\te_yaw: %.2f\n", ex,ey,ez,e_yaw); // Imprimier en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      printf("exf: %.3f\teyf: %.3f\tezf: %.2f\te_yawf: %.2f\n", exf,eyf,ezf,e_yawf); //Errores de formación del seguidor de impresión y error de orientación
      
      counter = 0; // Reseteo el contador
    }else counter++;

    ros::spinOnce();    //Requerido para recibir funciones de devolución de llamada (call back)
    loop_rate.sleep();  //Comando para esperar el resto del tiempo para completar la frecuencia del bucle
    if(kbhit())	key = getchar();

    if(key == 27) finish = true;
  }while(finish == false); //Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros 
  
  land(); //Funcion aterrizar el dron 
  printf("\n Node finished\n");
  return 0;
}