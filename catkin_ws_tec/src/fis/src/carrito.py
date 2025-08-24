#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int32

class ThrottleControlNode:
    def __init__(self):
        rospy.init_node('throttle_control_node', anonymous=True)
        
        # Suscripción a los tópicos necesarios
        rospy.Subscriber("/odom", PoseStamped, self.odom_callback)
        rospy.Subscriber("/velocity", Float64, self.velocity_callback)
        rospy.Subscriber("/light_status", Int32, self.light_status_callback)
        
        # Publicador para el comando de aceleración
        self.throttle_pub = rospy.Publisher("/pix/throttle_command", Float64, queue_size=10)
        
        # Inicialización de variables
        self.pose_x = 0.0
        self.xdot = 0.0
        self.light_status = 0
        
        # Parámetros de control
        self.L1_light_location_1 = 150
        self.L1_approachDist = 50
        self.speed_limit = 14

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x

    def velocity_callback(self, msg):
        self.xdot = msg.data

    def light_status_callback(self, msg):
        self.light_status = msg.data

    def compute_torque(self):
        pose_state = self.L1_light_location_1 - self.pose_x
        torque = 0.0
        
        if self.light_status == 1:
            if pose_state >= self.L1_approachDist:
                if self.xdot > 0.9 * self.speed_limit:
                    torque = -0.035
                elif self.xdot < 0:
                    torque = 0.01
                else:
                    torque = 2 / 3

            elif self.L1_approachDist > pose_state > 0:
                if self.xdot >= self.speed_limit:
                    torque = 0
                elif 0 <= self.xdot < self.speed_limit:
                    torque = 1
                else:
                    torque = 0.5

            else:
                torque = 0.67

        elif self.light_status == 2:
            if pose_state >= self.L1_approachDist:
                if self.xdot > 3:
                    torque = -0.63
                elif 0 < self.xdot <= 3:
                    torque = -0.035
                else:
                    torque = 0

            elif self.L1_approachDist > pose_state > 0:
                if self.xdot > 4:
                    torque = -0.83
                elif 0 < self.xdot <= 4:
                    torque = -0.03
                else:
                    torque = 0.167

            else:
                torque = 0.67

        elif self.light_status == 3:
            if pose_state >= self.L1_approachDist:
                if self.xdot > 3:
                    torque = -0.25
                elif self.xdot <= 0:
                    torque = 0.01
                else:
                    torque = 0.05

            elif self.L1_approachDist > pose_state > 0:
                if self.xdot > 2.5:
                    torque = -1
                elif 0.1 < self.xdot <= 2.5:
                    torque = -0.05
                else:
                    torque = 0.167

            else:
                torque = 0.67

        # Publica el torque calculado
        self.throttle_pub.publish(torque)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.compute_torque()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ThrottleControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass




















<launch>
    <!-- Lanzamiento del driver de PIX -->
    <include file="$(find pix_driver)/launch/pix_driver.launch"/>
    
    <!-- Lanzamiento del puente SocketCAN -->
    <include file="$(find socketcan_bridge)/launch/socketcan_bridge.launch"/>
    
    <!-- Lanzamiento de tu nodo personalizado -->
    <node name="throttle_control_node" pkg="fis" type="fis.py" output="screen"/>
</launch>

