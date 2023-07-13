/* 
Descripcion:
Este nodo ejecuta el controlador para esolver el problema de seguimiento utilizando un dron en gazebo. 
También tanto la trayectoria deseada como la trayectoria real de dron si publicadas en los tópicos "/tr_path" y "/path", respectivamente, para su visualización en Rviz.

En la función "velocity_controller" la trayectoria deseada es generada y las señales de control son calculadas. 
Para este ejemplo, la trayectoria deseada es una leminscata.

Primero, el servicio "/enable_motors" es llamado para mover el cuadrirrotor simulado. 
Las funciones de Despegue y Aterrizaje son definidas.

Las velocidades traslacionales (Vx, Vy, Vz) y la velocidad rotacional (Wz) son publicadas en el tópico "/cmd_vel".
La posición (x,y,z) y la orientación (quaternion) son recibidas desde el tópico "/ground_truth/state".



INSTRUCCIONES
Para aterrizar el dron y finalizar este nodo, presiones la tecla 'ESC'.

En diferentes terminales:
$ roslaunch my_hector_uavs my_hector_uav.launch
	# Lanza un cuatrirrotor (my simplified drone) en Gazebo usando un mundo vacío. 
	  Nota: La simulacióm emoieza pausada.

$ rosrun my_hector_uavs hector_quadrotor_controller_paths
	# Corre este nodo
	
$ roslaunch my_hector_uavs visualization_paths.launch
	# Corre Rviz con mi archivo de configuración para visualizar el dron y las trajectorias

*/
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" 	// Hacer la llamada al servicio --> roservice info/enable_motors
#include "nav_msgs/Odometry.h"				// Para suscribirle a la geometria del dron (mensaje)
#include "geometry_msgs/Twist.h"			// Para publicar velocidades (mensaje)
#include "tf/transform_broadcaster.h"		// Para hacer la conversion de cuaternion a angulo-Euler (fichero)
#include "nav_msgs/Path.h" 					// Para visualizar trayectorias en Rviz
// Los ultimos 3 se necesitan para utilizar eventos del teclado (función kbhit implemetada en Linux).
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

// Define objetos globales
ros::Publisher  velocity_publisher;
ros::Subscriber pose_subscriber;
geometry_msgs::Twist vel;
ros::ServiceClient enable_motors_client;

double x, y, z, roll, pitch, yaw; // Defino variables globales (posicion y orientacion, usando ángulos de Euler)
float takeoff_alt = 1.2; // Altura inicial deseada
int key;


geometry_msgs::PoseStamped drone_pose, tr_pose; // Crear objetos globales para definir la posición del dron y la trayectoria deseada usando mensajes de tipo geometry_msgs/Twist

// Define objetos para publicar la trayectoria del dron y la trayectoria deseada en Rviz
ros::Publisher drone_path_pub, tr_path_pub;
nav_msgs::Path drone_path_msg, tr_path_msg; // Objetos trayectoria de tipo nav_msgs/Path


// Crear el título del marco global. El marco global es requerido por Rviz para definir el MISMO marco para cualquier robot
const string global_frame = "/world";

double t, t0, Vxy_max = 1, Vz_max = 1.0, Wz_max = 4; /// Temporizador, tiempo inicial, velocidades traslacionales máximas [m/s] y máxima velocidad rotacional [rad/s]

double T = 100, k = 0.1; // Período de tayectoria, ganancias de controlador kx = ky = k
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; // Errores de seguimiento, posiciones deseadas, orientación deseada y sus derivadas con respecto al tiempo, respectivamente

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
  ROS_WARN_ONCE("Calling the enable_motors service...\n");
  hector_uav_msgs::EnableMotors enable_request;
  enable_request.request.enable = true;
  enable_motors_client.call(enable_request);

  if(enable_request.response.success){ ROS_WARN_ONCE("Enable motors successful\n"); }
  else{ cout << "Enable motors failed" << endl; }
}

void poseCallback(const nav_msgs::Odometry::ConstPtr msg){ // Función de devolución de llamada para obtener la posición del dron
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;
  // Operaciones para convertir de cuaternión a ángulos de Euler
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);  // Los ángulos de Euler están dados en Radianes

  drone_pose.pose.position = msg->pose.pose.position; // Copia la posición el dron al objeto "drone_pose"
  drone_path_msg.poses.push_back(drone_pose); // Agregar la nueva posición a la lista
}

void movement(float x, float y, float z, float turn){ // Función para asignar señales de control (Vx, Vy, Vz, Wz)
  vel.linear.x = x;       vel.linear.y = y;
  vel.linear.z = z;       vel.angular.z = turn;
}

void takeoff(){ //Takeoff function
  movement(0,0,0.2,0);
  velocity_publisher.publish(vel);

  cout << " Desired takeoff altitude = " << takeoff_alt << "\n Taking off ...\n";
  while(z < takeoff_alt-0.1){ // Comparar si se ha logrado la altura deseada
    velocity_publisher.publish(vel); 
    ros::spinOnce(); ros::Duration(0.1).sleep();
    if(kbhit())	key = getchar();
    if(key == 27) break;
  }
  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}

void land(){ // Función de Aterrizaje
  movement(0,0,-0.2,0);
  velocity_publisher.publish(vel);

  cout << "\n Landing ...\n";
  while(z > 0.3){
    velocity_publisher.publish(vel);
    ros::spinOnce(); ros::Duration(0.1).sleep();
  }
  movement(0,0,0,0);
  velocity_publisher.publish(vel);
}

void velocity_controller(){ // Función para generar la trayectoria deseada y para calcular las señales de control
  double Vx, Vy, Vz, Wz; // Define las señales de control signals del drone

  // Trayectoria deseada: Lemniscata
  double a = 3, b = 1.5, X0 = 0, Y0 = -0.5, Z0 = 1.5, w = 2*M_PI/T, c = 0.5, d = M_PI/2;
  // Posición deseada en el espacio 3D
  Xd = X0+a*sin(w*t);
  Yd = Y0+b*sin(2*w*t);
  Zd = Z0+c*sin(w*t); //Nota: 1 <= Zd <= 2
  Yawd = d*sin(w*t);

  // Derivadas con respecto al tiempo
  Xdp = a*w*cos(w*t);
  Ydp = 2*b*w*cos(2*w*t);
  Zdp = c*w*cos(w*t);
  Yawdp = d*w*cos(w*t);

  // Asignar la posición deseada al oobjeto "tr_pose"
  tr_pose.pose.position.x = Xd; tr_pose.pose.position.y = Yd; tr_pose.pose.position.z = Zd;
  tr_path_msg.poses.push_back(tr_pose); // Agregar el nuevo punto a la lista


  ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw - Yawd; // Calcular errores de seguimiento

  // Controlador Cinemático. Controles Auxiliares, en coordenadas globales
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  // Velocidades traslacionales con respecto al marco del robot
  Vx = Ux*cos(yaw)+Uy*sin(yaw);
  Vy = -Ux*sin(yaw)+Uy*cos(yaw);

  // Controlador Cinemático. Nota: Vz y Wzson calculadas directamente,ya que ellas son dadas con respecto al marco del robot
  Vz = Zdp-k*ez;
  Wz = Yawdp-k*e_yaw;

  // Velocidades de saturación
  if(abs(Vx)>Vxy_max){Vx = Vxy_max*abs(Vx)/Vx; printf("Sat Vx\t");}
  if(abs(Vy)>Vxy_max){Vy = Vxy_max*abs(Vy)/Vy; printf("Sat Vy\t");}
  if(abs(Vz)>Vz_max){Vz = Vz_max*abs(Vz)/Vz; printf("Sat Vz\t");}
  if(abs(Wz)>Wz_max){Wz = Wz_max*abs(Wz)/Wz; printf("Sat Wz\t");}

  movement(Vx,Vy,Vz,Wz);
  velocity_publisher.publish(vel); // Publicar las 4 señales de control
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_quadrotor_controller_paths");  // Inicializar el nodo
  ros::NodeHandle n; // Crear el nodo Handle
  int freq = 50; // Asignar frecuencia (Hz) de ejecucion al nodo
  ros::Rate loop_rate(freq);
  int counter = 0;
  bool finish = false;

  enable_motors_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors"); // Se configura  y llama al servicio
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // Publica en el topico velocidades utilizando el mensaje geometry 
  pose_subscriber = n.subscribe("/ground_truth/state", 1, poseCallback); //Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta 

  // Publicar las trayectorias
  drone_path_pub = n.advertise<nav_msgs::Path>("/path", 10);
  tr_path_pub = n.advertise<nav_msgs::Path>("/tr_path", 10);

  //Importante: Asignación del MIMSMO marco de referencia para TODOS los robots
  drone_path_msg.header.frame_id = tr_path_msg.header.frame_id = global_frame;

  enable_motors(); //Ejecuta las funciones "enable_motors" y "takeoff"
  takeoff();
 
  printf("Press 'ESC' to land the drone and finish the node");
  printf("To clean up both paths, press 'c' key\n");
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); // Mensaje de advertencia

  t0 = ros::Time::now().toSec(); // Obtener el tiempo inical  

  do{
    t = ros::Time::now().toSec()-t0; //printf("t: %.1f\n",t); // Calculo el tiempo del controlador     
    velocity_controller(); // Calculo las señales de control

    if(counter == 50){ // Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
      printf("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw); // Imprimier en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      counter = 0; //Resetear el contador
    }else counter++;

    // Publica los mensajes "path"
    drone_path_pub.publish(drone_path_msg); tr_path_pub.publish(tr_path_msg);

    ros::spinOnce();    //Requerido para recibir funciones de devolución de llamada (call back)
    loop_rate.sleep();   //Comando para esperar el resto del tiempo para completar la frecuencia del bucle
    if(kbhit())	key = getchar();

    if(key == 'c'){ // Limpiar ambas trayectorias
      drone_path_msg.poses.clear(); tr_path_msg.poses.clear(); // Limpiar mensajes de trayectoria
      ROS_WARN_ONCE("Clear paths\n");
      key = 0; //Resetear la tecla
    }
    if(key == 27) finish = true;
  }while(finish == false); //Note: No se utiliza ros::ok() porque es necesario seguir utilizando funciones de ros
  
  land(); // Ejecuta la función Aterrizar dron
  printf("\n Node finished\n");
  return 0;
}