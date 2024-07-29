from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState, Joy

class JoyVelocityPublisher(Node):

    def __init__(self):
        # initializa
        super().__init__('joy_velocity_publisher')
        qos_profile = QoSProfile(depth=10)

        # define parámetros
        self._linear_scale = 0.2
        self._angular_scale = 0.2

        # inicia suscriptores
        self.joy_status = self.create_subscription(Joy, '/joy', self.joy_callback, QoSReliabilityPolicy.RELIABLE)

        # inicia publicadores
        self.robot_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info(f"{self.nodeName} started")

        now = self.get_clock().now()


    def joy_callback(self, msg):
        # IMPORTANTE: La asignación de botones se debe verificar
        # para cada controlador en particular...
        A_button = msg.buttons[0]
        B_button = msg.buttons[1]
        X_button = msg.buttons[3]
        Y_button = msg.buttons[4]
        LB_button = msg.buttons[6]
        RB_button = msg.buttons[7]
        LT_button = msg.buttons[8]
        RT_button = msg.buttons[9]
        LT_axis = msg.axes[5]
        RT_axis = msg.axes[4]
        SELECT_button = msg.buttons[10]
        START_button = msg.buttons[11]
        LSTICK_button = msg.buttons[13]
        LSTICK_x_axis = msg.axes[0]
        LSTICK_y_axis = msg.axes[1]
        RSTICK_button = msg.buttons[14]
        RSTICK_x_axis = msg.axes[2]
        RSTICK_y_axis = msg.axes[3]
        DIR_X = msg.axes[6]
        DIR_Y = msg.axes[7]

        if A_button:
            self.get_logger().info(f"Left axes x: {LSTICK_x_axis}, y: {LSTICK_y_axis}")
            # prepara y publica un mensaje para la velocidad
            cmd_vel = Twist()
            cmd_vel.linear.x = LSTICK_y_axis * self._linear_scale
            cmd_vel.angular.z = LSTICK_x_axis * self._angular_scale
            self.robot_vel.publish(cmd_vel)


def main(args=None):
    try:
        rclpy.init(args=args)
        joy_velocity_publisher = JoyVelocityPublisher()
        rclpy.spin(joy_velocity_publisher)
    except KeyboardInterrupt: 
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()