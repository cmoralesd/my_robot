# my_robot: Una plantilla para crear tu primer robot en ROS2.

Para una explicación paso a paso de este tutorial, revisa este video:
## 1. Agrega el repositorio a tu espacio de trabajo
En un terminal, abre la carpeta *src* de tu espacio de trabajo y clona este repositorio, utilizando la siguiente instrucción:   
`git clone http://github.com/cmoralesd/my_robot.git`  
   
Compila:   
`cd <ruta_a_tu_espacio_de_trabajo>`   
`colcon build --symlink-install`   
   
Ahora ejecuta el lanzador para verificar que todo está bien:   
`source install/setup.bash`   
`ros2 launch my_robot view.launch.py`   
   
El lanzador abre una ventana de *rviz2* y un nodo que publica la descripción URDF de un modelo por medio de un tópico llamado *robot_description*. Sin embargo, aun falta configurar *rviz2* para que la información se despliegue correctamente.   
- Desde el panel *Displays*, agrega el plugin *RobotModel*. En las opciones de *RobotModel*, verifica que *Description Source* esté configurada en *Topic*. Luego, en *Description Topic* selecciona */robot_description*. Con esto, el modelo de una pequeña caja de color rojo debiera desplegarse en la ventana de rviz2.
- Si el color del modelo no se despliega correctamente, en *Global Options* edita la opción *Fixed Frame*, escribiendo *base_link*.
- Guarda esta configuración desde el menú *File*. Así no tendrás que repetirla cada vez que ejecutes el lanzador.   
Con esto se verifica que esta plantilla está funcionando en tu equipo. A continuación, aprenderás a convertir este modelo base en tu primer robot en ROS2.
## 2. Explora la estructura de archivos
Abre una ventana de VSCode en la carpeta del repositorio.
`code src/my_robot`   
La estructura de carpetas y archivos en el repositorio, es la siguiente:   
- Los archivos comunes y carpetas se encuentran en la carpeta raiz del repositorio, es decir, los archivos y carpetas que encontraremos en cualquier paquete de python para ros2: *package.xml*, *setup.cfg*, *setup.py* y las carpetas *resource*, *my_robot* y *test* (ver "Tu primer paquete con ament_python" en este enlace).
- Se ha agregado la carpeta *description*, para contener los archivos *urdf* y *xacro* con los cuales se describe el modelo del robot.
- Se ha agregado una carpeta *launch*, para contener los lanzadores. Mediante los lanzadores es posible ejecturar múltiples aplicaciones desde un único terminal, incluyendo configuraciones personalizadas y parámetros de ejecución.
Esta estructura básica será complementada más adelante, cuando agreguemos funcionalidades al robot y deseemos realizar simulaciones utilizando GazeboSim.
### 3. Modela la estructura básica de tu robot: robot_base.urdf
URDF (Universal Robot Description Format) es el formato que utiliza ros para modelar un robot.   
Consiste en una descripción en lenguaje XML de los eslabones (link) y articulaciones (joint) que conforman el robot.   
Por ahora, el archivo describe un robot compuesto por únicamente un link, el cual tiene la forma visual de una caja de 40x25x15 cm, de color rojo, ubicada justo al centro del sistema de coordenadas.
```
<?xml version="1.0"?>
<robot>
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="0.4 0.25 0.15" />
            </geometry>
            <material name="base_link_material">
                <color rgba="0.85 0.0 0.0 1.0" />
            </material>
        </visual>
    </link>
    
</robot>
```
Comenzaremos por editar la ubicación de la caja, levantándola del piso:   
`<origin xyz="0 0 0.125" rpy="0 0 0" />`    
Agregaremos una rueda a la izquierda. Para ello es necesario crear un *joint* de tipo *continuous* y agregar un nuevo *link*, dentro de la etiqueta *<robot>*. Daremos una una forma de cilindro al nuevo link, para representar la rueda.
```
    <joint name="left_wheel_joint" type="continuous">
        <origin xyz="-0.1 0.15 0.05" rpy="1.570796 0 0" />
        <axis xyz="0 0 1" />
        <parent link="base_link" />
        <child link="left_wheel" />
    </joint>
    <link name="left_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.05" />
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1.0" />
            </material>
        </visual>
    </link>
```
Al ejecutar nuevamente el lanzador. Se verá que la rueda no se muestra en la posición esperada y rviz2 señala un error en *RobotModel*. Esto se debe a que actualmente no hay información sobre la posición de *left_wheel_joint*.   
Para resolver esto, ejecutamos una aplicación que publique la posición de la rueda en el tópico *joint_status*. En un nuevo terminal, ejecutar:   
`ros2 run joint_state_publisher_gui joint_state_publisher_gui`  
Dado que habrá que hacer esta tarea cada vez que editemos el archivo URDF, es mejor agregar la instrucción al lanzador. Edita *view.launch.py*, agregando un lanzador para joint_state_publisher_gui.   
Dentro de *generate_launch_description()*, agrega:
```
joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
        
    )
```
Y agrega *joint_state_publisher_gui_node* a la lista de ejecutable en `return LaunchDescription([])`.   

Agrega ahora una nueva rueda, a la derecha: 
```
    <joint name="right_wheel_joint" type="continuous">
        <origin xyz="-0.1 -0.15 0.05" rpy="1.570796 0 0" />
        <axis xyz="0 0 1" />
        <parent link="base_link" />
        <child link="right_wheel" />
    </joint>
    <link name="right_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.05" />
            </geometry>
            <material name="gray">
                <color rgba="0.5 0.5 0.5 1.0" />
            </material>
        </visual>
    </link>
```
Y finalmente, agrega un link de tipo *fixed* y una pequeña esfera como rueda de soporte.
```
<joint name="connection_link" type="fixed">
        <origin xyz="0.15 0 0.05" rpy="0 0 0" />
        <axis xyz="0 0 1" />
        <parent link="base_link" />
        <child link="caster_wheel" />
    </joint>
    <link name="caster_wheel">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <sphere radius="0.05" />
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1.0" />
            </material>
        </visual>
    </link>
```
El modelo ya tiene representación visual. Sin embargo, para que el modelo sea funcional en una simulación, requiere contener, además parámetros inerciales y el contorno de colisión. Considera lo siguiente para complementar tu modelo:
- Las etiquetas de colisión pueden ser completados simplemente copiando la información de los elementos visuales. De esta forma, el modelo representará su contorno de colisión con el mismo contorno de su forma visual. Es preferible hacer modelos simples para el calculo de colisión, pues simplifica los calculos matemáticos posteriores.
- El cálculo de inercia es más complejo y requiere usualmente de simplificar las formas complejas a formas simples, como cajas, esferas o cilindros.
- En los modelos de robot construidos con software de diseño 3D, estos parámetros se calculan automáticamente al momento de exportar el modelo.

