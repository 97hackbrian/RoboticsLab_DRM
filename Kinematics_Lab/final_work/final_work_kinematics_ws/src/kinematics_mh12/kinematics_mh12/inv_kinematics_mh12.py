import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from sympy import Matrix, symbols, cos, sin, pi, diff, lambdify, pprint
import numpy as np
import math
from random import random 
from kinematics_mh12.fw_kinematics_mh12 import T_sym

#-Old symbols definitions
#t1=symbols('t1')
#t2=symbols('t2')
#t3=symbols('t3')
#t4=symbols('t4')
#t5=symbols('t5')

# New symbols definitions
t1, t2, t3, t4, t5 = symbols('theta_1 theta_2 theta_3 theta_4 theta5', real=True)

# Learning parameters
alpha=0.06  #learning rate 0.07
iterations = 255  #350

def create_jacobian(T_sym, thetas):
    # Get position elements from the transformation matrix T_sym
    px = T_sym[0,3]
    py = T_sym[1,3]
    pz = T_sym[2,3]

    # Create a dinamic Jacobian matrix
    J_rows = []
    # For each position component
    for p in [px, py, pz]:
        # Parcial derivatives w.r.t. each theta
        row = [diff(p, theta) for theta in thetas]
        J_rows.append(row)
    return Matrix(J_rows)

# Get the list of joint variables
thetas = [t1, t2, t3, t4, t5] 

# Create jacobian symbolically
px = T_sym[0,3]
py = T_sym[1,3]
pz = T_sym[2,3]
J = create_jacobian(T_sym, thetas)

class invKinematicsMH12Node(Node):
    def __init__(self):
        super().__init__('inverse_mh12_node')
        self.subscription = self.create_subscription(Point, 'target', self.sub_callback,10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription
        self.get_logger().info('Node inverse started')
        
    def sub_callback(self,msg):
        driffz=0.0004
        driffxy=0.0003
        target=Matrix([msg.x,msg.y,msg.z])
        self.get_logger().info('Received target: x={:.3f}, y={:.3f}, z={:.3f}'.format(msg.x, msg.y, msg.z))
        ##target += np.sign(target) * driffxy
        
        for i in [0, 1]:
            target[i] += np.sign(target[i]) * driffxy
            '''
            if target[i]>0:
                target[i] += driffxy
            else:
                target[i] -= driffxy
        '''    
        target[2] += np.sign(target[2]) * driffz 

        ti=Matrix([random(),random(),random(),random(),random()])
        if hasattr(self, 'last_ti'):
            ti = self.last_ti
        else:
            ti = Matrix([0, 0, 0, 0, 0])

        for i in range(iterations):
            # Evaluate forward position and Jacobian numerically
            try:

                cp = Matrix([
                px.subs([(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])]),
                py.subs([(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])]),
                pz.subs([(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])])])
                Jsubs = J.subs([(t1, ti[0]), (t2, ti[1]), (t3, ti[2]), (t4, ti[3]), (t5, ti[4])])

                e = (target - cp)
                
                #e = e / max(1.0, e.norm())  
            
                Jinv=Jsubs.H*(Jsubs*Jsubs.H)**-1
                #Jinv = (Jsubs.T * Jsubs + 0.01 * Matrix.eye(5))**-1 * Jsubs.T

                dt=Jinv*e

                ti=ti+alpha*dt

            except Exception as exc:
                self.get_logger().error(f'Numeric evaluation failed at iter {i}: {exc}')
                break
            # Build and publish final JointState (6 joints expected by other parts;  joint_4_r fixed to 0)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            joint_msg = JointState()

            joint_msg.name = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

            joint_angles = [ti[0], ti[1], ti[2], 0.0, -ti[3], ti[4]]
            
            joint_msg.position = joint_angles
            joint_msg.header = header
            self.publisher_.publish(joint_msg)
            self.get_logger().info('Published joint angles: {}'.format(joint_angles))
            self.get_logger().info('Error: {:.6f}'.format(e.norm()))
        self.get_logger().info('Finished IK iterations for target.')
        self.last_ti = ti

def main(args=None):
    rclpy.init(args=args)
    node = invKinematicsMH12Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()