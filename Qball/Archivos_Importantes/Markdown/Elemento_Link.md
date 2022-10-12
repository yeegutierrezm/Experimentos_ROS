# Elementos Necearios:

1. [Elementos Link:](#elemento-link)
    
    1. [Visual](#construcción-visual-en-gazebo-elemento-link)
    2. [Colision](#construcción-colisiones-elemento-link)
    3. [Inercia](#construcción-inercias-elemento-link)

2. [Elemento Joint](#configuración-elemento-joint)

# Elemento Link:

# Construcción Visual en Gazebo (Elemento Link)

![Dron blender](/imagenes/visual.png)

Para esto se procede a utilizar la etiqueta Visual, parámetro de geometría en el cual se importan
las mallas (estructuras complejas) creadas en Blender. Se realizaron diferentes Pruebas con las
diferentes extensiones que genera es programa para los archivos 3D y se observo lo siguiente:


- **.dae:** Los archivos DAE pertenecen principalmente a Blender, Los activos que se guardan
en un archivo DAE pueden incluir datos físicos, sombreados, animación y geometría,
almacenados mediante el espacio de nombres y el esquema de base de datos basados en
XML de COLLADA (Collaborative Design Activity).

- **.stl:** Una limitación del formato de archivo STL es que no contiene información más allá
de la geometría del modelo 3D. Por lo tanto, aparte del tamaño y la forma del modelo,
una STL no contiene ninguna otra información. Si deseas conservar la información sobre la textura y el color de su modelo 3D, entonces convertirlo a STL no es la mejor opcion.

~~~
Nota: Se opto por trabajar con esta opcion, y configurar cada material de forma individualya que las configuraciones que venian por defecto de blender, se veian muy diferentes
en terminos visuales a las que se configuraban en Gazebo.
~~~

- **.obj:** Además de preservar la forma del modelo, un archivo OBJ también contiene información sobre su geometría, textura y la malla original con la que fue creado.

## Parámetros:

- **\<visual\>** (opcional): Es la representación gráfica del eslabón, se puede establecer como geometría o con una malla 3D externa.
    
    - **\<name\>**(opcional): Establece un nombre a esta parte del eslabón
    
    - **\<origin\>** (opcional, si no se especifica será el equivalente a la matriz identidad): Establece la referencia del elemento visual respecto a la referencia del eslabón.
        - **xyz** (opcional, por defecto (0,0,0)) Representa el desplazamiento en cada coordenada. 19
        - **rpy** (opcional, si no se especifica será la matriz identidad) Representa el ángulo
de giro sobre cada eje.
    
    - **\<geometry\>** (requerido): Define la geometría del elemento visual, puede una de las
siguientes:
    
    - **\<box\>** Define un prisma como geometría, se puede establecer la longitud de sus
costados mediante el atributo size.
    
    - **\<cylinder\>** Define cilindro como geometría, se puede establecer su radio y la
altura mediante los atributos radius y lenght respectivamente.
    
    - **\<sphere\>** Define una esfera como geometría, se puede establecer el radio mediante
el atributo radius. El origen está situado en el centro
    
    - **\<mesh\>** Se importa una figura en tres dimensiones, tiene un atributo scale que
permite variar el tamaño según la necesidad. Se pueden importar fichero en formato
.dae o .stl.
    
    - **\<material\>**(opcional) Especifica el color o textura que se va a enseñar.
    
        - **\<name\>** (opcional) Nombre de referencia al color o a la textura.
    
        - **\<color\>** (opcional) Se establece el color mediante una combinación rgba; rojo,
verde, azul, opacidad; comprendido cada uno entre 0 y 1
    
        - **\<texture\>** (opcional) Un fichero externo conteniendo la textura, debe ser formato
.jpg, .png o .bmp


# Construcción Colisiones (Elemento Link)

![Dron blender](/imagenes/Construccion.png)

- El elemento de colisión es un subelemento directo del objeto de enlace, al mismo nivel
que la etiqueta visual.

- El elemento de colisión define su forma de la misma manera que lo hace el elemento
visual, con una etiqueta de geometría. El formato de la etiqueta de geometría depende
de la figura, puede revisar el siguiente link:

- También puede especificar un origen de la misma manera que un subelemento de la
etiqueta de colisión (como con el objeto visual).


En muchos casos, deseará que la geometría y el origen de la colisión sean exactamente iguales a
la geometría y el origen visuales. Sin embargo, hay dos casos principales en los que no lo harías.

- Procesamiento más rápido: hacer la detección de colisiones para mallas(figura con estructuras y texturas diferentes a las tradicionales) es mucho más complejo computacional que
para geometrías simples. Por lo tanto, es posible que desee reemplazar las mallas con
geometrías más simples en el elemento de colisión.

- Zonas seguras: es posible que desee restringir el movimiento cerca de equipos sensibles.
Por ejemplo, si no queremos que nada choque con la cabeza de R2D2, podríamos definir
la geometría de colisión como un cilindro que encierra su cabeza para evitar que algo se
acerque demasiado a su cabeza.

## Parámetros
En bloque **\<colision\>** se definen las propiedades de colisión del eslabón. Presenta unos atributos similares al bloque **\<visual\>**, e igualmente se le puede añadir una malla con la etiqueta
**\<mesh\>**. Igualmente, se presentan a continuación los atributos del bloque:

- **\<origin\>**: Origen del sistema de referencia visual. En la etiqueta **\<xyz\>** se define la
posición y en la etiqueta **\<rpy\>** la orientación del sólido rígido.

- **\<geometry\>:**  Forma visual del cuerpo. Puede contener las siguientes formas de etiquetas: **\<box\>**, **\<cylinder\>**, **\<sphere\>** y **\<mesh\>**.

    - **\<mesh\>**: está última se trata de una forma tridimensional en formato STL.

**NOTA:** Varias peticiones de etiquetas **\<collision\>** pueden existir para el mismo eslabón. La
unión de la geometría que definen constituye la representación visual del eslabón.

# Construcción Inercias (Elemento Link)

![Dron blender](/imagenes/inercia1.png)

Para que el modelo se simule correctamente, se deben definir varias propiedades físicas del robot,
es decir, las propiedades que necesita Gazebo para dar interaccion con el ambiente.

- Cada elemento de enlace que se simula necesita una etiqueta inercial.

- Este elemento también es un subelemento del objeto de enlace.

- La masa se define en kilogramos.

- La matriz de inercia rotacional de 3x3 se especifica con el elemento de inercia. Como esto
es simétrico, puede representarse por solo 6 elementos, como tal.

## Consideraciones:

- El tensor de inercia depende tanto de la masa como de la distribución de masa del objeto.
Una buena primera aproximación es asumir una di stribución equitativa de la masa en el
volumen del objeto y calcular el tensor de inercia en función de la forma del objeto, como
se describe anteriormente.

- Si no está seguro de qué poner, una matriz con ixx/iyy/izz=1e-3 o menor suele ser un
valor predeterminado razonable para un enlace de tamaño medio (corresponde a una caja
de 0,1 m de longitud lateral con una masa de 0,6 kg). La matriz identidad es una elección
particularmente mala, ya que a menudo es demasiado alta (corresponde a una caja de 0,1
m de lado con una masa de 600 kg).

- También puede especificar una etiqueta de origen para especificar el centro de gravedad
y el marco de referencia inercial (en relación con el marco de referencia del vínculo).

- Cuando se utilizan controladores en tiempo real, los elementos de inercia de cero (o casi
cero) pueden hacer que el modelo de robot colapse sin previo aviso, y todos los enlaces
aparecerán con sus orígenes coincidiendo con el origen mundial.

## Calculo Inercia

Para calcular las inercias se recurrió al siguiente link:

 <https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors>

En este se encuentra la forma correcta de calcular la Inercia segun la figura, para obtener la inercia. 

Formula para calcular la matriz de inercia de un paralelepípedo sólido de ancho w, altura h,
profundidad d y masa m:

![Dron blender](/imagenes/formula.png)



## Parametros:
- **\<inertial\>**(opcional):
Es el elemento que contiene la información respecto a las propiedades inerciales del eslabón.
        
    - **\<origin\>**(opcional, si no se especifica será el equivalente a la matriz identidad):
Especifica la referencia del origen de los momentos de inercia, al igual que la parte visual
y la de colisión contiene los atributos xyz y rpy para definir su posición y orientación.
        
    - **\<mass\>** (opcional): Define la masa del eslabón en gramos
        
    - **\<inertia\>** (opcional): Matriz 3x3 del rotacional de inercia, al ser simétrico solo se pueden
definir los atributos ixx, ixy, ixz, iyy, iyz e izz.

# Configuración Elemento Joint

El elemento joint (articulación) define las propiedades cinemáticas y dinámicas, además de los
límites de seguridad (safety limits) de la articulación:

## Parámetros:

Para este elemento es necesario declarar un nombre y definir el tipo de articulación, los cuales pueden ser:

- **Revolute:** articulación con un grado de libertad de revolución en torno a un eje con un
rango limitado de giro, que vienen delimitados por un rango superior e inferior.
144 6 Construcción del Dron

- **Fixed:** articulación fija en la que no existe ningún grado de libertad. Este tipo de articulación no requiere especificar las etiquetas **\<axis\>**, **\<calibration\>**, <dynamics>, **\<limits\>** ni safety_controller.
Continuos: una articulación similar a la articulación revolute pero sin límites de rango
de giro.

- **Prismatic:** una articulación de un grado de libertad de desplazamiento linear sobre un
eje dentro de un rango limitado.

- **Planar:** articulación con 2 grados de libertad que permite el movimiento en un plano
perpendicular al eje especificado.

- **Floating:** movimiento libre en los 6 grados de libertad.

El elemento articulación está definida por las siguientes etiquetas:

- **\<origin\>:** indica la transformada desde el eslabón del padre con el eslabón del hijo,
cuya ubicación del joint se encuentra en el origen del eslabón hijo. Se puede apreciar en
la figura X. La etiqueta **\<xyz\>** representa el desplazamiento en los ejes cartesianos, y la
etiqueta **\<rpy\>** la rotación entorno a los ángulos de Euler.

- **\<parent\>:** posee el nombre del eslabón padre al que está unido el joint.

- **\<child\>:** posee el nombre del eslabón hijo al que está unido el joint.

- **\<axis\>:** representa el eje de giro o traslación de la articulación.

- **\<calibration\>**(opcional) La posición de referencia de la articulación, usada para calibrar
la posición absoluta de la articulación.
    - **rising** (opcional) Cuando la articulación se mueve en una dirección positiva, esta
posición de referencia dará lugar a un flanco ascendente.

    - **falling** (opcional) Cuando la articulación se mueve en una dirección positiva, esta
posición de referencia dará lugar a un flanco descendente.

- **\<dynamic\>:** propiedades físicas de la articulación, que permite especificar los valores
de fricción y de amortiguación. Por defecto esto valores son nulos.
    - **damping** El valor de amortiguación física de la articulación ( N∗s
m para articulaciones prismáticas, N∗m∗s
rad para articulaciones de revolución).
   
    - **friction** El valor de fricción estática física de la articulación (N para articulaciones
prismáticas, N ∗ s para articulaciones de revolución).

- **\<limit\>:** indica los límites superior e inferior del rango de movimiento, velocidad máxima
y esfuerzo máximo aplicado a la articulación.
    - **lower** (opcional, por defecto es 0) Un atributo que especifica el límite inferior de
unión (radianes para juntas de revolución, metros en las articulaciones prismáticas).
En articulaciones contínuas no se especifica este campo.

    - **upper** (opcional, por defecto es 0) Un atributo que especifica el límite superior conjunta (radianes para juntas de revolución, metros en las articulaciones prismáticas).
En articulaciones contínuas no se especifica este campo.
    - **effort** (requerido) Un atributo para el máximo esfuerzo de la articulación (|esfuerzo
aplicado | <|esfuerzo|).
    - **velocity** (requerido) Define la velocidad máxima de la articulación.

- **\<mimic\>** (opcional) Esta etiqueta se utiliza para especificar esta articulación imita a
otra articulación existent. El valor de este conjunto puede calcularse como:

    $valor = multiplicador ∗ other\_joint\_value + offset$

    Va seguido de los siguientes atributos:
    
    - **joint** (requerido) Especifica el nombre de la articulación a imitar.

    - **multiplier** (opcional) Especifica el factor multiplicativo en la fórmula anterior. El valor por defecto es 1.
    
    - **offset** (opcional) Especifica el desplazamiento para agregar en la fórmula anterior.
Por defecto es 0.

- **\<safety_controller\>**: permite especificar una serie de atributos relacionados con los
límites de la articulación con respecto al controlador

    - **soft_lower_limit** (opcional, por defecto es 0) Un atributo que especifica el límite
inferior de la articulación donde el controlador de seguridad comienza a limitar la
posición de la articulación. Este límite debe ser mayor que el límite inferior de la
articulación.
    
    - **soft_upper_limit** (opcional, por defecto es 0) Un atributo que especifica el límite
superior de la articulación donde el controlador de seguridad comienza a limitar la
posición de la articulación. Este límite debe ser menor que el límite superior de la
articulación.
    
    - **k_position** (opcional, por defecto es 0) Un atributo que especifica la relación entre
los límites de posición y velocidad.
    - **k_velocity** (requerido) Un atributo que especifica la relación entre el esfuerzo y
los límites de velocidad.

