# Crear el Espacio de Trabajo para el Qball
En una nueva terminal:  

Crear una carpeta en el usuario raíz
<pre><code>mkdir -p ~/Qball/src </code></pre>

Entramos a la carpeta
<pre><code>cd ~/Qball/src </code></pre>

Inicializamos el espacio de trabajo
<pre><code>catkin_init_workspace </code></pre>

Salimos de la carpeta src
<pre><code>cd .. </code></pre>

Compilamos el espacio de trabajo
<pre><code>catkin_make </code></pre>

Observamos el listado de carpetas que se han creado
<pre><code>ls </code></pre>

Siempre que abramos un terminal nuevo debemos ejecutar un fichero de inicialización
<pre><code>source ~/Qball/devel/setup.bash </code></pre>

Otra opción, es crear un fichero de inicialización que ejecute ese comando automáticamente,
```
echo "source ~/Qball/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ahora, procedemos a clonar este repositorio, dentro de la carpeta ```src```:
```
cd ~/Qball/src
git clone https://github.com/CarlosAlfredoMarin/
```

Compilamos, nuevamente, el espacio de trabajo
```
cd ~/Qball
catkin_make
```   
<br><br>

# Como Lanzar el Experimento:

Ejecutamos el archivo lanzador con el siguiente comando
~~~
roslaunch qball Lanzador_Qball.launch
~~~

