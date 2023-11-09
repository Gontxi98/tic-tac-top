# ROS Workspace MUCSI
Este contenedor contiene un workspace de ROS 1 con los recursos mínimos para trabajar los proyectos de la asignatura de Robótica y Automatización Inteligente.

**ESTE CONTENEDOR NO ESTÁ PENSADO PARA USO EN DISTRIBUCIÓN SOLO TIENE FINES DIDÁCTICOS**
## Instalación
Antes de lanzar el contenedor, asegurarse de que el host usa un sistema de ventanas X y la Xauthority está mapeada en la variable de entorno XAUTHORITY.

Si la Xauthority no está mapeada en esta variable, modificar el compose para mapear la localización correcta.

También se presupone que el host usa una trajeta gráfica NVIDIA.

Una vez comprobado todo es suficiente con lanzar:
 ```
 docker compose up
 ```
El contenedor mapea el directorio workspaces completo, todo lo que se almacene en este directorio será persistente incluso cuando se acaba el contenedor.

No se recomienda borrar el contenedor cuando se acabe de utilizar, basta con acabarlo y volverlo a lanzar más adelante.

## Uso
Una vez instalado, es necesario instalar las dependencias y construir el espacio de trabajo. Esto no se hace automáticamente para que los estudiantes entiendan el *workflow* de ROS.

Para instalar las dependencias de todos los paquetes:
```
apt update && rosdep update
```
```
rosdep install --from-paths src --ignore-src -r -y
```
Para construir el espacio de trabajo:
```
catkin build
```
Cada vez que se quieran utilizar los paquetes del espacio de trabajo es necesario *sourcearlo* (esto se debe hacer en cada consola nueva):
```
source /home/ubuntu/workspaces/catkin_ws/devel/setup.bash
```
**NO SE RECOMIENDA INCLUIR EL COMANDO EN .BASHRC**

Una vez *sourceado*, ya es posible lanzar cualquier launchfile o nodo del paquete.

## Lanzamiento de los drivers del robot
Los robots se pueden lanzar utilizando el siguiente comando, donde X es el robot especificado (3,4,5 o 6):
```
roslaunch launcher_robots_lab_robotica robot_20X.launch
```