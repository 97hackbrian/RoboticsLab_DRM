import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import sys

from sympy import Matrix, symbols, cos, sin, pi

#Cinematica directa pero con matrices no homogeneas
# Definicion de matrices de rotacion y vectores de longitud(posicion)
 
t = symbols('theta')
Rx = Matrix([[1,0,0],[0,cos(t),-sin(t)],[0,sin(t),cos(t)]])
Ry = Matrix([[cos(t),0,sin(t)],[0,1,0],[-sin(t),0,cos(t)]])
Rz = Matrix([[cos(t),-sin(t),0],[sin(t),cos(t),0],[0,0,1]])
L0 = Matrix([[0.4],[0],[0]])
L1 = Matrix([[0.3],[0],[0]])


joint_names = ['joint1','joint1_1','joint2']

class JointSubs(Node):

    def __init__(self):
        super().__init__('joint_subscriber')

        self.subscription = self.create_subscription(JointState, 'joint_states', self.sub_callback, 10)
        self.subscription  # prevent unused variable warning

    def sub_callback(self, msg):
        t1 = msg.position[0]
        t2 = msg.position[1]
        t3 = msg.position[2]
        p1 = Rz.subs(t, t1) * (Ry.subs(t, -t2) * L0)
        p2 = Rz.subs(t, t1) * Ry.subs(t, -t2) *  ((Ry.subs(t, -t3))*L1)  + p1
        print(p1,p2)


def main(args=None):
    rclpy.init(args=args)

    joints_sub = JointSubs()

    rclpy.spin(joints_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #rclpy.spin_once(joints_pub, timeout_sec=1.0)

    joints_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
