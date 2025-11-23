import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from sympy import Matrix, symbols, cos, sin, pi, diff 
import numpy as np 
from random import random 
from scipy.spatial.transform import Rotation as R
import time


t1=symbols('t1')
t2=symbols('t2')
t3=symbols('t3')
t4=symbols('t4')
t5=symbols('t5')
t6=symbols('t6')

header = Header()
joint_msg = JointState()
joint_msg.name = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
alpha=0.3 #learning rate
iterations = 100



px=0.08*(-(sin(t2)*cos(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*cos(t4) - sin(t1)*sin(t4))*sin(t5) - 0.08*(sin(t2)*sin(t3)*cos(t1) - cos(t1)*cos(t2)*cos(t3))*cos(t5) - 0.42*sin(t2)*sin(t3)*cos(t1) + 0.035*sin(t2)*cos(t1)*cos(t3) + 0.035*sin(t3)*cos(t1)*cos(t2) + 0.42*cos(t1)*cos(t2)*cos(t3) + 0.455*cos(t1)*cos(t2) + 0.025*cos(t1)
py=0.08*(-(-sin(t1)*sin(t2)*cos(t3) - sin(t1)*sin(t3)*cos(t2))*cos(t4) - sin(t4)*cos(t1))*sin(t5) - 0.08*(-sin(t1)*sin(t2)*sin(t3) + sin(t1)*cos(t2)*cos(t3))*cos(t5) + 0.42*sin(t1)*sin(t2)*sin(t3) - 0.035*sin(t1)*sin(t2)*cos(t3) - 0.035*sin(t1)*sin(t3)*cos(t2) - 0.42*sin(t1)*cos(t2)*cos(t3) - 0.455*sin(t1)*cos(t2) - 0.025*sin(t1)
pz=-0.08*(-sin(t2)*sin(t3) + cos(t2)*cos(t3))*sin(t5)*cos(t4) - 0.08*(sin(t2)*cos(t3) + sin(t3)*cos(t2))*cos(t5) - 0.035*sin(t2)*sin(t3) - 0.42*sin(t2)*cos(t3) - 0.455*sin(t2) - 0.42*sin(t3)*cos(t2) + 0.035*cos(t2)*cos(t3) + 0.4

J=Matrix([[diff(px,t1),diff(px,t2),diff(px,t3),diff(px,t4),diff(px,t5)],[diff(py,t1),diff(py,t2),diff(py,t3),diff(py,t4),diff(py,t5)],[diff(pz,t1),diff(pz,t2),diff(pz,t3),diff(pz,t4),diff(pz,t5)]])




class targetSubscriber(Node):
    
    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(Point, 'target', self.sub_callback,10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription
		
    def sub_callback(self,msg):
        target=Matrix([msg.x,msg.y,msg.z])
        ti=Matrix([random(),random(),random(),random(),random()])
        for i in range(iterations):
            cp= Matrix([px.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3]),(t5,ti[4])]),py.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3]),(t5,ti[4])]),pz.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3]),(t5,ti[4])])])
            e=target-cp	
            Jsubs=J.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3]),(t5,ti[4])])
            Jinv=Jsubs.H*(Jsubs*Jsubs.H)**-1
            dt=Jinv*e
            ti=ti+alpha*dt
   
            header.stamp = self.get_clock().now().to_msg()
            joint_angles=[ti[0],ti[1],ti[2],ti[3],ti[4],0]
            joint_msg.header = header
            joint_msg.position = joint_angles
            self.publisher_.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)

    target_sub = targetSubscriber()

    rclpy.spin(target_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #rclpy.spin_once(joints_pub, timeout_sec=1.0)
    
    target_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
