/* 
Descripcion:
Este nodo ejecuta el controlador para esolver el problema de seguimiento utilizando un dron en gazebo. en la funcion "velocity_controller" tanto la trayectoria deseada como las señales de control son calculadas, para este ejemplo la trayectoria deseada es una lemniscata 

1. hacemos una llamada al servicio "/enable_motor" para mover el dron simulado tambien se definen las funciones de despegue y aterrizaje. 
Tanto las velocidades translacionaes (vx,vy,vz) y rotacionales (Wz) del dron son publicadas en el topico "/cmd_ vel".
La posicion (x,y,z) y orientacion (quaternion) del dron se reciben del tpico "/ground_truth/state".

Para terminar este nodo es necesario precionar la tecla  ESC y tambien se aterriza el dron  
   


Instrucciones:
En diferentes terminales:
$ roslaunch my_hector_uavs my_hector_uav.launch
	#Lanza un quadrotor simulado en Gazebo usando un mundo vacío. Nota: La simulación comienza en pausa.

$ rosrun my_hector_uavs hector_quadrotor_controller
	# Correr el nodo
*/
#include "ros/ros.h"
#include "hector_uav_msgs/EnableMotors.h" ////Hacer la llamada al servicio --> roservice info/enable_motors
#include "nav_msgs/Odometry.h" //Para suscribirle a la geometria del dron (mensaje)
#include "geometry_msgs/Twist.h"  //Para publicar velocidades (mensaje)
#include "tf/transform_broadcaster.h" //Para hacer la conversion de cuaternion a angulo-Euler (fichero)
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

double x, y, z, roll, pitch, yaw; //Defino variable global (posicion y orientacion, usando euler-angulo)
float takeoff_alt = 1.2; // configuro la altura deseada al despegar
int key;

double t, t0, Vxy_max = 1.5, Vz_max = 0.5, Wz_max = 4; //temporizador, tiempo inicial, velocidades de traslación máximas [m / s] y velocidades de rotación máximas [rad / s] para obtener una simulacion mas precisa a la real. (los rotores no tienen velocidades infinaitas)


double T = 100, k = 0.1; //Período de trayectoria, el controlador de ganancia kx = ky = k para los errores de seguimiento 
double ex, ey, ez, e_yaw, Xd, Yd, Zd, Yawd, Xdp, Ydp, Zdp, Yawdp; // defino los errores de seguimiento cartesianos, la posicion y orientacion deseada y sus respectiva derivada con respecto al tiempo

int kbhit(void){ //Funcion para utilizar eventos de teclado
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
}

void movement(float x, float y, float z, float turn){ // Función para asignar las señales de control (Vx, Vy, Vz, Wz)
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
  movement(0,0,0,0);  //se publica velocidad de 0 para mantener vuelo estacionario
  velocity_publisher.publish(vel);
}

void land(){ //Funcion aterrizaje
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

void velocity_controller(){ // Funcion que genera la trayectoria deseada y las señales de control para el dron 
  double Vx, Vy, Vz, Wz; //Defino las variables locales 

  //Defino la trayectoria deseada: Lemniscata por medio de las ecuaciones
  double a = 3, b = 1.5, X0 = 0, Y0 = -0.5, Z0 = 1.5, w = 2*M_PI/T, c = 0.5, d = M_PI/2;
  //Posicion deseada en el espacio 3d
  Xd = X0+a*sin(w*t);
  Yd = Y0+b*sin(2*w*t);
  Zd = Z0+c*sin(w*t); //Note: 1 <= Zd <= 2
  Yawd = d*sin(w*t);

  //Derivadas del tiempo correspondiente
  Xdp = a*w*cos(w*t);
  Ydp = 2*b*w*cos(2*w*t);
  Zdp = c*w*cos(w*t);
  Yawdp = d*w*cos(w*t);


  ex = x-Xd; ey = y-Yd; ez = z-Zd; e_yaw = yaw - Yawd; //Calculo los errores de seguimiento 

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
  velocity_publisher.publish(vel); //publico las 4 señales de control 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_quadrotor_controller");  //Inicicalizo el nodo
  ros::NodeHandle n; //Creo el nodo
  int freq = 50; //Asignar frecuencia (Hz) de ejecucion al nodo
  ros::Rate loop_rate(freq);
  int counter = 0; // Se crea un contador
  bool finish = false;  //Indica si el nodo a termiando
 
  enable_motors_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors"); // Se configura  y llama al servicio
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);  // Publica en el topico velocidades utilizando el mensaje geometry 
  pose_subscriber = n.subscribe("/ground_truth/state", 1, poseCallback);  //Se suscribe al topico de odometria usando el topico ground y se asigna una funcion de llamada de vuelta 


  enable_motors(); //Ejecuta y llama la funcion que llama al servicio enable 
  takeoff(); //Despega el dron 
 
  printf("Press 'ESC' to land the drone and finish the node");
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); //Mensaje de advertencia

  t0 = ros::Time::now().toSec(); //Obtengo el tiempo inical 

  do{
    t = ros::Time::now().toSec()-t0; //Calculo el tiempo del controlador   
    velocity_controller(); //Calculo las señales de control

    if(counter == 50){ //Divisior de frecuencia para mostrar los errores de seguimiento de forma mas lenta 
      printf("ex: %.3f ey: %.3f ez: %.2f e_yaw: %.3f\n", ex,ey,ez,e_yaw); // Imprimier en la terminal los errores de seguimiento [m] y el error de orientacion [rad]
      counter = 0; //Reseteo el contador
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