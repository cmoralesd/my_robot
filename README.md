# my_robot - Una plantilla para crear tu primer robot en ROS2
Esta rama contiene el modelo completo del tutorial que se describe en este video: https://youtu.be/zIkWGv5pKLY      
Incluye, además, algunos aplicativos adicionales que permiten dar movimiento a tu robot.
## my_robot_controller
Un simple controlador en lazo abierto. Recibe comandos de velocidad desde el tópico */cmd_vel* y publica la posición del robot a través del tópico de odometría */odom*.    
Instrucciones detalladas aquí: https://youtu.be/3Q9U6D1uC2U   
   
Para enviar comandos de velocidad vía consola a nuestro robot, puede utilizarse la siguiente instrucción:    
`ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"`

## joy_velocity_publisher
Convierte movimientos de un joystick en comandos de velocidad que son publicados mediante el tópico */cmd_vel*.   
Instrucciones detalladas aquí: https://youtu.be/mP-rBK-ST-8   
