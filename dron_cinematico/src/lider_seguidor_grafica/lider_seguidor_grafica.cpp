/* 
Descripción:
Con este nodo, la trayectoria deseada del dron líder y la trayectoria del dron seguidor son publicadas en el tópico correspondiente para su visualización en Rviz.

Este nodo ejecuta el controlador para resolver el problema de control de formación, usando el esquema líder-seguidor, con 2 drones simulados en Gazebo. El dron líder desplega su trayectoria deseada (usando el controlador de velocidad), y luego el dron seguidor sigue la trayectoria del líder con un offset. 

El servicio "/enable_motors" de cada dron es llamado. Las funciones de aterrizaje y despegue son definidas.

INSTRUCCIONES
Para aterrizar los drones y finalizar este nodo, presiones la tecla 'ESC'.

En diferentes terminales:
$ roslaunch my_hector_uavs two_hector_uavs.launch
	$ Lanza 2 cuadtrirrotores en Gazebo usando un mundo vacío. Nota: La simulación inica pausada.

$ rosrun my_hector_uavs two_hector_quadrotors_lf_paths
	# Corre este nodo

$ roslaunch my_hector_uavs two_visualization_paths.launch
	# Corre Rviz con mi archivo de configuración para visualizar los 2 drones y sus trayectorias

*/
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" 	//Hacer la llamada al servicio --> roservice info/enable_motors
#include "nav_msgs/Odometry.h" 				//Para suscribirle a la geometria del dron (mensaje)
#include "geometry_msgs/Twist.h"			//Para publicar velocidades (mensaje)
#include "tf/transform_broadcaster.h"		//Para hacer la conversion de cuaternion a angulo-Euler (fichero)
#include "nav_msgs/Path.h" 					// Para visualizar trayectorias en Rviz
// Los ultimos 3 se necesitan para utilizar eventos del teclado (función kbhit implemetada en Linux).
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

// Crear una estructura personalizada para definir la posición de robot (posición y orientación, usando ángulos de Euler)
struct robot_pose{ double x, y, z, roll, pitch, yaw; };  

// Define objetos globales
ros::Publisher  vel_pub1, vel_pub2;
ros::Subscriber pose_sub1, pose_sub2;
geometry_msgs::Twist vel1, vel2;
ros::ServiceClient enable_motors_client1, enable_motors_client2;

int key; // Variable para guardar el valor de la tecla
robot_pose r_pose1, r_pose2; //Define variables globales (posición y orientación, usando estructura personalizada)
float takeoff_alt = 0.5; // Altura inicial deseada

geometry_msgs::PoseStamped drone_pose1, drone_pose2, tr_pose; // / Crear objetos globales para definir la posición de los drones y la trayectoria deseada usando mensajes de tipo geometry_msgs/Twist

// Define objetos para publicar la trayectoria del dron y la trayectoria deseada en Rviz
ros::Publisher drone_path_pub1, drone_path_pub2, tr_path_pub;
nav_msgs::Path drone_path_msg1, drone_path_msg2, tr_path_msg; //Paths objects of nav_msgs/Path messages type

// Crear el título del marco global. El marco global es requerido por Rviz para definir el MISMO marco para cualquier robot
const string global_frame = "world";

double t, t0, Vxy_max = 2, Vz_max = 0.5, Wz_max = 8; // Temporizador, tiempo inicial, velocidades traslacionales máximas [m/s] y máxima velocidad rotacional [rad/s]

// Variables del dron Líder
double T = 100, k = 0.1; // Período de tayectoria, ganancias de controlador kx = ky = k
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; // Errores de seguimiento, posiciones deseadas, orientación deseada y sus derivadas con respecto al tiempo, respectivamente
double Vxl, Vyl, Vzl, Wzl; // Señales de Control

// Variables del dron Seguidor
double exf, eyf, ezf, e_yawf, s_d = 1.5, alp_d = M_PI, s_zd = 0; //Tracking errors and desired formation states
double Vxf, Vyf, Vzf, Wzf;  //Control signals


int kbhit(void){ // Funcion para utilizar eventos de teclado
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

void enable_motors(){ // Función para llamar al servicio "/ enable_motors", necesaria para mover el dron
  ROS_WARN_ONCE("Calling the enable_motors services ...\n");
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

void poseCallback1(const nav_msgs::Odometry::ConstPtr msg){ // Función de devolución de llamada para obtener la posición del dron 1
  r_pose1.x = msg->pose.pose.position.x;
  r_pose1.y = msg->pose.pose.position.y;
  r_pose1.z = msg->pose.pose.position.z;
  // Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(r_pose1.roll, r_pose1.pitch, r_pose1.yaw);  // Los ángulos de Euler están dados en Radianes

  drone_pose1.pose.position = msg->pose.pose.position; // Copia la posición el dron al objeto "drone_pose1"
  drone_path_msg1.poses.push_back(drone_pose1); // Agregar la nueva posición a la lista

  // Asignación de los TF (usando para Rviz únicamente)
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(r_pose1.x, r_pose1.y, r_pose1.z) );
  transform.setRotation(quater);
  string robot_frame = "uav1/base_link";
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame, robot_frame));
}

void poseCallback2(const nav_msgs::Odometry::ConstPtr msg){ // Función de devolución de llamada para obtener la posición del dron 2
  r_pose2.x = msg->pose.pose.position.x;
  r_pose2.y = msg->pose.pose.position.y;
  r_pose2.z = msg->pose.pose.position.z;
  // Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(r_pose2.roll, r_pose2.pitch, r_pose2.yaw);  // Los ángulos de Euler están dados en Radianes

  drone_pose2.pose.position = msg->pose.pose.position; // Copia la posición el dron al objeto "drone_pose2"
  drone_path_msg2.poses.push_back(drone_pose2); // Agregar la nueva posición a la lista

  // Asignación de los TF (usando para Rviz únicamente)
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(r_pose2.x, r_pose2.y, r_pose2.z) );
  transform.setRotation(quater);
  string robot_frame = "uav2/base_link";
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame, robot_frame));
}

void move1(float x, float y, float z, float turn){ // Función para asignar señales de control y publicarlas, dron 1
  vel1.linear.x = x; vel1.linear.y = y; vel1.linear.z = z; vel1.angular.z = turn;
  vel_pub1.publish(vel1);
}
void move2(float x, float y, float z, float turn){ // Función para asignar señales de control y publicarlas, dron 2
  vel2.linear.x = x; vel2.linear.y = y; vel2.linear.z = z; vel2.angular.z = turn;
  vel_pub2.publish(vel2);
}

void takeoff(){ /// Función de Despegue de todos los drones
  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";

  //Quadrotor 1
  move1(0,0,0.3,0);
  
  while(r_pose1.z < takeoff_alt-0.1){ // Comparar si se ha logrado la altura deseada
    vel_pub1.publish(vel1);
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); // Chequiar si alguna tecla es presionada
    if(key == 27) break;
  }
  move1(0,0,0,0); // Detenerse

  //Quadrotor 2
  move2(0,0,0.3,0);

  while(r_pose2.z < takeoff_alt-0.1){ // Comparar si se ha logrado la altura deseada
    vel_pub2.publish(vel2); 
    vel_pub1.publish(vel1); //Importante: Las velocidades del dron 1 deben seguir siendo publicadas para evitar un aterrizaje no deseado
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar(); // Chequiar si alguna tecla es presionada
    if(key == 27) break;
  }
  move2(0,0,0,0); // Detenerse
}

void land(){ // Función de Aterrizaje de todos los drones
  cout << "\n Landing ...\n";

  move1(0,0,-0.3,0); move2(0,0,-0.3,0);

  while(r_pose1.z > 0.3 || r_pose2.z > 0.3){
    vel_pub1.publish(vel1);  vel_pub2.publish(vel2);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  move1(0,0,0,0);  move2(0,0,0,0); // Apagar todos los drones
}

void leader_vel_control(){ // Función para genear la trayectoria deseada y calcular las señales de control del robot líder
  //Define the desired trajectory: Helix
  double X0 = 0, Y0 = 0, Z0 = 0.5, radius = 3, w = 2*M_PI/T, Vzd = 0.02;

  // Posición deseada en el espacio 3D
  Xd = X0+radius*sin(w*t);
  Yd = Y0+radius*cos(w*t);
  Zd = Z0+Vzd*t; //linear increasing
  Yawd = sin(w*t); //sinusoidal behavior

  // Derivadas con respecto al tiempo
  Xdp = radius*w*cos(w*t);
  Ydp = -radius*w*sin(w*t);
  Zdp = Vzd;
  Yawdp = w*cos(w*t);

  // Asignar la posición deseada al objeto "tr_pose"
  tr_pose.pose.position.x = Xd; tr_pose.pose.position.y = Yd; tr_pose.pose.position.z = Zd;
  tr_path_msg.poses.push_back(tr_pose); // Agregar el nuevo punto a la lista


  ex = r_pose1.x-Xd; ey = r_pose1.y-Yd; ez = r_pose1.z-Zd; e_yaw = r_pose1.yaw - Yawd; // Calcular errores de seguimiento

  // Controlador Cinemático. Controles Auxiliares, en coordenadas globales (variables locales)
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  // Velocidades traslacionales con respecto al marco del robot
  Vxl = Ux*cos(r_pose1.yaw)+Uy*sin(r_pose1.yaw);
  Vyl = -Ux*sin(r_pose1.yaw)+Uy*cos(r_pose1.yaw);

  // Controlador Cinemático. Nota: Vz y Wzson calculadas directamente,ya que ellas son dadas con respecto al marco del robot
  Vzl = Zdp-k*ez;
  Wzl = Yawdp-k*e_yaw;

  // Velocidades de saturación
  if(abs(Vxl)>Vxy_max){Vxl = Vxy_max*abs(Vxl)/Vxl; printf("Sat Vxl\t");}
  if(abs(Vyl)>Vxy_max){Vyl = Vxy_max*abs(Vyl)/Vyl; printf("Sat Vyl\t");}
  if(abs(Vzl)>Vz_max){Vzl = Vz_max*abs(Vzl)/Vzl; printf("Sat Vzl\t");}
  if(abs(Wzl)>Wz_max){Wzl = Wz_max*abs(Wzl)/Wzl; printf("Sat Wzl\t");}

  move1(Vxl,Vyl,Vzl,Wzl); // Publicar las 4 señales de control
}

void follower_vel_control(){ // Función calcular las señales de control del robot seguidor
  // Nota: La posición deseada del seguidor y la orientación deseada son dadas por las del líder
  
  double Xfd, Yfd, Xfdp, Yfdp, xlp, ylp, Ux, Uy; // Crear variables locales

  // Posición deseada del Seguidor con respecto al marco global
  Xfd = r_pose1.x+s_d*cos(r_pose1.yaw+alp_d);
  Yfd = r_pose1.y+s_d*sin(r_pose1.yaw+alp_d);
   
  // Velocidades traslacionales del líder en el marco global
  xlp = Vxl*cos(r_pose1.yaw)-Vyl*sin(r_pose1.yaw);
  ylp = Vxl*sin(r_pose1.yaw)+Vyl*cos(r_pose1.yaw);

  // Derivadas con respecto al tiempo
  Xfdp = xlp-Wzl*s_d*sin(r_pose1.yaw+alp_d);
  Yfdp = ylp+Wzl*s_d*cos(r_pose1.yaw+alp_d);
	
  // Calcular errores de seguimiento (con respecto al marco global)
  exf = r_pose2.x-Xfd; 			eyf = r_pose2.y-Yfd;
  ezf = r_pose2.z-(r_pose1.z+s_zd); 	e_yawf = r_pose2.yaw - r_pose1.yaw; // La altura y la orientación son iguales a las del Líder
  
  // Controlador Cinemático. Controles Auxiliares, en coordenadas globales
  Ux = Xfdp-k*exf; 	Uy = Yfdp-k*eyf;
  
  // Velocidades traslacionales con respecto al marco del robot
  Vxf = Ux*cos(r_pose2.yaw)+Uy*sin(r_pose2.yaw);
  Vyf = -Ux*sin(r_pose2.yaw)+Uy*cos(r_pose2.yaw);

  // Velocidades vertical y rotacional
  Vzf = Vzl-k*ezf;
  Wzf = Wzl-k*e_yawf;

  // Velocidades de saturación
  if(abs(Vxf)>Vxy_max){Vxf = Vxy_max*abs(Vxf)/Vxf; printf("Sat Vxf\t");}
  if(abs(Vyf)>Vxy_max){Vyf = Vxy_max*abs(Vyf)/Vyf; printf("Sat Vyf\t");}
  if(abs(Vzf)>Vz_max){Vzf = Vz_max*abs(Vzf)/Vzf; printf("Sat Vzf\t");}
  if(abs(Wzf)>Wz_max){Wzf = Wz_max*abs(Wzf)/Wzf; printf("Sat Wzf\t");}

  move2(Vxf,Vyf,Vzf,Wzf); // Publicar las 4 señales de control
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "two_hector_quadrotors_lf_paths");  // Inicializar el nodo
  ros::NodeHandle n; // Crear el nodo Handle
  int freq = 50; // Asignar frecuencia (Hz) de ejecucion al nodo
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;

  enable_motors_client1 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav1/enable_motors"); // Se configura  y llama al servicio
  enable_motors_client2 = n.serviceClient<hector_uav_msgs::EnableMotors>("uav2/enable_motors");
  vel_pub1 = n.advertise<geometry_msgs::Twist>("uav1/cmd_vel", 1); // Publica en el topico velocidades utilizando el mensaje geometry 
  vel_pub2 = n.advertise<geometry_msgs::Twist>("uav2/cmd_vel", 1);
  pose_sub1 = n.subscribe("uav1/ground_truth/state", 1, poseCallback1); //Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta 
  pose_sub2 = n.subscribe("uav2/ground_truth/state", 1, poseCallback2);

  // Publicar las trayectorias
  drone_path_pub1 = n.advertise<nav_msgs::Path>("uav1/path", 10);
  drone_path_pub2 = n.advertise<nav_msgs::Path>("uav2/path", 10);
  tr_path_pub = n.advertise<nav_msgs::Path>("/tr_path", 10);

  //Importante: Asignación del MISMO marco de referencia para TODOS los robots
  drone_path_msg1.header.frame_id = drone_path_msg2.header.frame_id = tr_path_msg.header.frame_id = global_frame;

  enable_motors(); //Ejecuta las funciones "enable_motors" y "takeoff" (para todos los robots)
  takeoff();
 
  printf(" Press 'ESC' to land the drones and finish the node\n");
  printf(" To clean up all paths, press 'c' key\n");
  ROS_WARN_ONCE(" To start the movement, the simulation must be running\n\n"); // Mensaje de advertencia

  t0 = ros::Time::now().toSec(); // Obtener el tiempo inicial 

  do{
    t = ros::Time::now().toSec()-t0; //printf("t: %.1f\n",t); // Calculo el tiempo del controlador   
    leader_vel_control(); // Calculo las señales de control del robot Líder
    follower_vel_control(); // Calculo las señales de control del robot Seguidor

    if(counter == 100){ // Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
      printf("\nex: %.3f\tey: %.3f\tez: %.2f\te_yaw: %.2f\n", ex,ey,ez,e_yaw);  // Imprimir en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      printf("exf: %.3f\teyf: %.3f\tezf: %.2f\te_yawf: %.2f\n", exf,eyf,ezf,e_yawf); // Imprimir los errores de formación y errores de orientación del Seguidor
      
      counter = 0; // Resetear el contador
    }else counter++;

    // Publica los mensajes "path"
    drone_path_pub1.publish(drone_path_msg1); drone_path_pub2.publish(drone_path_msg2); tr_path_pub.publish(tr_path_msg);

    ros::spinOnce();    /// Requerido para recibir funciones de devolución de llamada (call back)
    loop_rate.sleep();  // Comando para esperar el resto del tiempo para completar la frecuencia del bucle
    if(kbhit())	key = getchar();

    if(key == 'c'){ // Limpiar ambas trayectorias
      drone_path_msg1.poses.clear(); drone_path_msg2.poses.clear(); tr_path_msg.poses.clear(); // Limpiar mensajes de trayectoria
      printf(" Clear paths\n");
      key = 0; // Resetear la tecla
    }
    if(key == 27) finish = true;
  }while(finish == false); // Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros
  
  land(); // Ejecuta la función Aterrizar dron
  printf("\n Node finished\n");
  return 0;
}

