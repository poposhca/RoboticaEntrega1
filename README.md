# Proyecto Temas Selectos de Robotica

## Segunda entrega

En esta práctica se tiene como objetivo mostrar el funcionamiento de un control de poses para un robót móvil.

### Compilacion

En este repositorio se encuntra la carpeta *control* que es el workspace del proyecto del contról cinemático, para compilar se debe se ejecutar *catkin_make* dentró de esta carpeta (puede estar en ubucación deseada del equipo con el que se compile). Adicionalmente se debe contar con el modelo del carro autonomo y Gazebo instalados en la máquina. En este vínculo https://github.com/EagleKnights/SDI-11911/wiki se encuntra el módelo requerido y las instrucciones para instalar Gazebo.

### Estructura del paquete

La estructura del paquete que se encuntra en este repositorio es la siguiente:
* control (workspace del paquete ROS)
    * src (Ubicación de los archivos del paquete)
        * control (Ubicación del código fuente, el archivo Cmake y el manifiesto)

### Ejecución del paquete

Después de compilar y agregar las variebles de ambiente se obtendrá un paquete de ROS llamado **control** con un ejecutable:
*   *pose_controller* Es el programa que contiene el control de carro. No recibe parámetros de entrada, por el momento las metas del control están definidas dentro del código y se pueden modificar en la función main. 

    rosrun control pose_controller

___

## Primera entrega: Generación de trayectorias en ROS

En esta práctica tiene como objetivo mostrar los conocimientos básicos aprendidos de ROS (Robot Operativi System). Para realizarló se utiliza un sistema **Ubuntu 16.04** LTS junto con la versión **Kinetic Kame** de ROS.

### Compilacion

En este repositorio se encuntra la carpeta *test* que es el workspace del proyecto, para compilar se debe se ejecutar *catkin_make* dentró de esta carpeta (puede estar en ubucación deseada del equipo con el que se compile) para que CMake genere las direcciones locales adecuadas para la ejecución del paquete.

### Estructura del paquete

La estructura del paquete que se encuntra en este repositorio es la siguiente:
* test (workspace del paquete ROS)
    * src (Ubicación de los archivos del paquete)
        * test (Ubicación del código fuente, el archivo Cmake y el manifiesto)

### Ejecución del paquete

Después de compilar y agregar las variebles de ambiente se obtendrá un paquete de ROS llamado **test** con dos ejecutables:
*   *Hello* Es el programa encargado de mover linealmente a la tortuga, recibe dos argumentos, la distancia *d* y la velocidad máxima que alcanza la toruga *vmax*

    rosrun test hello *distancia* *vmax*

*   *subs*  Es el programa que se subscribe al tópico que posee la posición de la tortuga, este programa no recibe argumentos y se debe detener manuelmente.
    
    rosrun test subs