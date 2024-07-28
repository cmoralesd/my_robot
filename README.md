# my_robot: Una plantilla para crear tu primer robot en ROS2.

Para una explicación paso a paso de este tutorial, revisa este video:
## 1. Agrega el repositorio a tu espacio de trabajo
En un terminal, abre la carpeta *src* de tu espacio de trabajo y clona este repositorio, utilizando la siguiente instrucción:   
`git clone http://github.com/cmoralesd/my_robot.git`  
   
Compila:   
`cd <ruta_a_tu_espacio_de_trabajo>`   
`colcon build --symlink-install`   
   
Ahora ejecuta el lanzador para verificar que todo está bien:   
`source install/setup.bash>`   
`ros2 launch my_robot view.launch.py`   
   
El lanzador abre una ventana de *rviz2* y un nodo que publica la descripción de un modelo base por medio de un tópico llamado *robot_description*. Sin embargo, aun falta configurar *rviz2* para que la información se despliegue correctamente.   
- Desde el panel *Displays*, agrega el plugin *RobotModel*. En las opciones de *RobotModel*, verifica que *Description Source* esté configurada en *Topic*. Luego, en *Description Topic* selecciona */robot_description*. Con esto, el modelo de una pequeña caja de color rojo debiera desplegarse en la ventana de rviz2.
- Si el color del modelo no se despliega correctamente, en *Global Options* edita la opción *Fixed Frame*, escribiendo *base_link*.
- Guarda esta configuración desde el menú *File*. Así no tendrás que repetirla cada vez que ejecutes el lanzador.   
Con esto se verifica que esta plantilla está funcionando en tu equipo. A continuación, aprenderás a convertir este modelo base en tu primer robot en ROS2.
## 2. Explorando la estructura de archivos
Abre una ventana de VSCode en la carpeta del repositorio.
`code src/my_robot`   
