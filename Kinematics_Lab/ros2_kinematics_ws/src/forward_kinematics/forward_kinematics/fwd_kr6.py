import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from sympy import Matrix, symbols, cos, sin, pi 
import numpy as np 
from scipy.spatial.transform import Rotation as R

t=symbols('theta')
d=symbols('d')
a=symbols('a')
aph=symbols('alpha')

t1=symbols('theta_1')
t2=symbols('theta_2')
t3=symbols('theta_3')
t4=symbols('theta_4')
t5=symbols('theta_5')
t6=symbols('theta_6')


Rz=Matrix([[cos(t),-sin(t),0,0],[sin(t),cos(t),0,0],[0,0,1,0],[0,0,0,1]])
Tz=Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
Tx=Matrix([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
Rx=Matrix([[1,0,0,0],[0,cos(aph),-sin(aph),0],[0,sin(aph),cos(aph),0],[0,0,0,1]])

A=Rz*Tz*Tx*Rx
A01 =A.subs({t:pi      ,d:0.4   ,a:0     ,aph:pi    })
A12 =A.subs({t:t1+pi   ,d:0     ,a:0.025 ,aph:pi/2  })
A23 =A.subs({t:t2      ,d:0     ,a:0.455 ,aph:0  })
A34 =A.subs({t:t3-pi/2 ,d:0     ,a:0.035 ,aph:pi/2  })
A45 =A.subs({t:t4+pi   ,d:-0.42 ,a:0     ,aph:pi/2  })
A55p=A.subs({t:t5      ,d:0     ,a:0     ,aph:pi/2  })
A5p6=A.subs({t:pi      ,d:0.08  ,a:0     ,aph:pi    })
A67 =A.subs({t:t6    ,d:0     ,a:0     ,aph:0   })
At  =Matrix([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])



class JointSubscriber(Node):
    
    def __init__(self):
        super().__init__('joint_subscriber')
        self.subscription = self.create_subscription(JointState, 'joint_states', self.sub_callback,10)
        self.subscription
		
    def sub_callback(self,msg):
        j1=msg.position[0]
        j2=msg.position[1]
        j3=msg.position[2]
        j4=msg.position[3]
        j5=msg.position[4]
        j6=msg.position[5]
        T= A01*A12*A23*A34*A45*A55p*A5p6*A67*At
        #T=A.subs({t:0,d:0,a:0,aph:pi/2})*A01*A12
        res=T.subs({t1:j1 ,t2:j2 ,t3:j3 ,t4:j4 ,t5:j5 ,t6:j6})
        rotation = R.from_matrix(np.array(res)[0:3,0:3])
        quaternion = rotation.as_quat()
        print("x: ",res[0,3]," y: ",res[1,3]," z: ",res[2,3]," orientation: ",quaternion)
        #print(res)
        #print(quaternion)

def main(args=None):
    rclpy.init(args=args)

    joints_sub = JointSubscriber()

    rclpy.spin(joints_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #rclpy.spin_once(joints_pub, timeout_sec=1.0)
    
    joints_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
