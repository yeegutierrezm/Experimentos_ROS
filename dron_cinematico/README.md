# Experimentos con QBall y ROS
Este repositorio contiene las carpetas, códigos y explicación de algunos experimentos básicos realizados con la librería Hector_Quadrotor, un modelo tridimensional del dron Quanser Qball2 y ROS.

![](https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/blob/main/Qball.png)

# Índice
1. [Instalación Librería Hector](#id1)
2. [Crear el Espacio de Trabajo](#id2)
3. [¿Cómo Lanzar los Experimentos?](#id3)
      - [Lazo Abierto - Movimiento Circular](#id4)
      - [Lazo Cerrado - Trayectoria Lemniscata](#id5)
      - [Lazo Cerrado - Trayectoria Lemniscata y Gráfica](#id6)
      - [Lazo Cerrado - Trayectoria, Líder-Seguidor](#id7)
      - [Lazo Cerrado - Trayectoria, Líder-Seguidor y Gráfica](#id8)  
<br><br>


<div id='id1' />

# Instalación Librería Hector

Para descargar los archivos de este repositorio y poder ejecutarlos en tu propio computador sigue las siguientes instrucciones.  
<br>

Se requieren instalar algunas dependencias de ROS. En una nueva terminal:
```
sudo apt-get install ros-noetic-ros-control ros-noetic-gazebo-ros-control ros-noetic-unique-identifier ros-noetic-tf2-geometry-msgs ros-noetic-laser-geometry ros-noetic-geographic-info ros-noetic-tf-conversions ros-noetic-joy
```

Crear una carpeta en el usuario raíz
```
mkdir -p ~/hector_ws/src
cd hector_ws/src
```

Ahora, procedemos a descargar la librería, dentro de la carpeta ```src```:
```
git clone  https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor
git clone  https://github.com/tu-darmstadt-ros-pkg/hector_localization
git clone  https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
git clone  https://github.com/tu-darmstadt-ros-pkg/hector_models
```

Compilamos, el espacio de trabajo
```
cd ..
catkin_make
echo "source ~/hector_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```  
<br><br>



<div id='id2' />

# Crear el Espacio de Trabajo
En una nueva terminal:  

Crear una carpeta en el usuario raíz
<pre><code>mkdir -p ~/my_ws/src </code></pre>

Entramos a la carpeta
<pre><code>cd ~/my_ws/src </code></pre>

Inicializamos el espacio de trabajo
<pre><code>catkin_init_workspace </code></pre>

Salimos de la carpeta src
<pre><code>cd .. </code></pre>

Compilamos el espacio de trabajo
<pre><code>catkin_make </code></pre>

Observamos el listado de carpetas que se han creado
<pre><code>ls </code></pre>

Siempre que abramos un terminal nuevo debemos ejecutar un fichero de inicialización
<pre><code>source ~/my_ws/devel/setup.bash </code></pre>

Otra opción, es crear un fichero de inicialización que ejecute ese comando automáticamente,
```
echo "source ~/my_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ahora, procedemos a clonar este repositorio, dentro de la carpeta ```src```:
```
cd ~/my_ws/src
git clone https://github.com/CarlosAlfredoMarin/Experimentos_con_Libreria_Hector
```

Compilamos, nuevamente, el espacio de trabajo
```
cd ~/my_ws
catkin_make
```   
<br><br>








<div id='id3' />

# ¿Cómo Lanzar los Experimentos?
Cada una de kas siguientes líneas de código se ejecutan en terminales separadas, por ejemplo, ```Lazo Abierto - Movimiento Circular``` tiene 2 líneas, usted debe abrir 2 terminales, una para cada línea de código.

<div id='id4' />

## Lazo Abierto - Movimiento Circular

[Movimiento Circular]

[Movimiento Circular]: https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/tree/main/dron_cinematico/src/movimiento_circular

```
roslaunch dron_cinematico lanzador_principal.launch
```
```
rosrun dron_cinematico movimiento_circular
```



<div id='id5' />

## Lazo Cerrado - Trayectoria Lemniscata

[Trayectoria Lemniscata]

[Trayectoria Lemniscata]: https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/tree/main/dron_cinematico/src/lemniscata_helice

```
roslaunch dron_cinematico lanzador_principal.launch
```
```
rosrun dron_cinematico lemniscata_helice
```



<div id='id6' />

## Lazo Cerrado - Trayectoria Lemniscata y Gráfica

[Trayectoria Lemniscata y Gráfica]

[Trayectoria Lemniscata y Gráfica]: https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/tree/main/dron_cinematico/src/lemniscata_helice_grafica

```
roslaunch dron_cinematico lanzador_principal.launch
```
```
rosrun dron_cinematico lemniscata_helice_grafica
```
```
roslaunch dron_cinematico visualizacion_graficas.launch
```




<div id='id7' />

## Lazo Cerrado - Trayectoria, Líder-Seguidor

[Trayectoria, Líder-Seguidor]

[Trayectoria, Líder-Seguidor]: https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/tree/main/dron_cinematico/src/lider_seguidor

```
roslaunch dron_cinematico lanzador_2_quadrotores.launch
```
```
rosrun dron_cinematico lider_seguidor
```



<div id='id8' />

## Lazo Cerrado - Trayectoria, Líder-Seguidor y Gráfica

[Trayectoria, Líder-Seguidor y Gráfica]

[Trayectoria, Líder-Seguidor y Gráfica]: https://github.com/CarlosAlfredoMarin/Experimentos_con_QBall_y_ROS/tree/main/dron_cinematico/src/lider_seguidor_grafica

```
roslaunch dron_cinematico lanzador_2_quadrotores.launch
```
```
rosrun dron_cinematico lider_seguidor_grafica
```
```
roslaunch dron_cinematico visualizacion_lider_seguidor_graficas.launch
```
