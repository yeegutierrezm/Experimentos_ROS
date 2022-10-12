# Construccion del Modelo del Cuadricoptero Qball 2

<div style="text-align: justify">
Este repositorio contiene las carpetas, códigos y explicación de algunos experimentos básicos realizados en el qball 2, y un modelo tridimensional del dron Quanser Qball2 y ROS.
</div>  
<br> 


# Indice
Este tutorial describe los detalles para construir el archivo SDF y las piezas necesarias que se requieren para importar en este:

1. [Piezas Librerias GrabCad](#piezas-de-la-libreria-grabcad)
2. [Archivo Blender.](#archivo-blender)
3. [Creacion de Espacio de Trabajo](#creacion-de-espacio-de-trabajo)  

<br>


# Piezas de La libreria GrabCad:

![Dron blender](/imagenes/Partes_Solidwork.png)

Las partes del dron se encuentran en el siguiente link: 

* [Carpeta archivos Solidwork](http://www.limni.net)

<br>


# Archivo Blender.

![Dron blender](/imagenes/Dron_Blender.png)


El archivo de Blender se encuentra en el sigueinte Link: 

* [Carpeta Archivo Dorn](http://www.limni.net)

<br>


# Creacion de Espacio de Trabajo.

En una nueva terminal

Crear una carpeta en el usuario raiz

~~~ 
mkdir -p ~/Qball/src
~~~

Entramos a la carpeta


~~~ 
cd ~/Qball/src 
~~~

Inicializamos el espacio de trabajo

~~~ 
catkin_init_workspace 
~~~

Salimos de la carpeta src

~~~ 
cd .. 
~~~

Compilamos el espacio de trabajo

catkin_make 
~~~ 
catkin_make 
~~~

Observamos el listado de carpetas que se han creado

~~~
ls
~~~

Siempre que abramos un terminal nuevo debemos ejecutar un fichero de inicialización

~~~
source ~/Qball/devel/setup.bash 
~~~

Otra opción, es crear un fichero de inicialización que ejecute ese comando automáticamente,

~~~
echo "source ~/qball/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc
~~~

Ahora, procedemos a clonar este repositorio, dentro de la carpeta src:



~~~
cd ~/qball/src </code></pre> 

git clone https://github.com/CarlosAlfredoMarin/Experimentos_con_Libreria_Hector
~~~

Compilamos, nuevamente, el espacio de trabajo

~~~
cd ~/qball
catkin_make
~~~

# Construcción de las Partes de Robot

Creamos el paquete de desarrollo, el cual tiene un archivo con las librerías y configuración necesarias.  El paquete tendrá el nombre "Qball".

~~~
catkin_create_pkg Qball std_msgs rospy roscpp urdf
~~~
Ingresamos a la carpeta Qball
~~~~
cd robot1_description
~~~~

Creamos una carpeta "sdf"
~~~
mkdir urdf
~~~

En la carpeta "sdf" es donde alojaremos los archivos de descripción del robot. El archivo qball.sdf contiene la descripción de las siguientes partes del robot:

- Esfera
- Base
- Propel 1
- Propel 2
- Propel 3
- Propel 4

Para definir cada una de estas piezas, es necesario conocer los elementos necesarios para crearlos:

1. [Elemento Link:](#)

    - Visual
    - Colision
    - Inercia

2. [Elemento Joint](#)

Existe una herramienta que permite ver el diagrama de conexiones del robot, usamos el siguiente comando para instalarla:
~~~
sudo apt install liburdfdom-tools
~~~
ejecutamos la herramienta (debemos estar primero ubicados en la carpeta que contiene el archivo .sdf)
~~~
cd urdf
urdf_to_graphiz robot1.urdf
~~~

Ahora, tenemos 2 archivos nuevos creados, en los cuales está la visualización gráfica de los link y joint, abrimos el archivo nuevo de extensión .pdf:

~~~
Imagen
~~~

Para revisar si el archivo URDF tiene errores se corre la siguiente line de codigo, es importante entrar a la carpeta donde se encuentra el archivos .sdf, de lo contrario, genera error.:
~~~
check_urdf robot1.urdf
~~~

# creacion del archivo lanzador

Ahora, vamos a crear un archivo lanzador, con solo llamar este archivo por consola, se ejecutará automáticamente el roscore y se ejecutará Gazebo con nuestro robot en pantalla. Creamos una carpeta llamada launch:
~~~
/robot1_description/
mkdir launch
~~~
Copiar el archivo display.launch, compilamos el espacio de trabajo, abrimos terminal en Brazo_Robot:
~~~
catkin_make
~~~
Ejecutamos el archivo lanzador con el siguiente comando
~~~
roslaunch robot1_description display.launch
~~~

Ejecutamos el archivo lanzador con el siguiente comando

~~~
roslaunch robot1_description display.launch
~~~








