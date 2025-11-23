import sympy as sp
from sympy import symbols, Matrix, eye, sin, cos, expand, pprint,pi

import numpy as np 
from scipy.spatial.transform import Rotation as R_quat

# Define symbols
theta = symbols('theta', real=True)
w0, w1, w2 = symbols('omega_0 omega_1 omega_2', real=True)

# Define vectors
w = Matrix([w0, w1, w2])   # rotation axis, unit vector


# Skew-symmetric matrix of w
skew_w = Matrix([
    [0, -w[2], w[1]],
    [w[2], 0, -w[0]],
    [-w[1], w[0], 0]
])

pprint(skew_w)
# Rodrigues rotation matrix
R = eye(3) + sin(theta)*skew_w + (1 - cos(theta))*(skew_w**2)

qx, qy, qz = symbols('q_x q_y q_z', real=True)
q = Matrix([[qx],[qy],[qz]])  


L1,L2,L3 = symbols('L_1 L_2 L_3', real=True)
t1,t2,t3 = symbols('theta_1 theta_2 theta_3', real=True)

v=-skew_w*q
pprint(v)
pprint(v.subs({w0:1,w1:0,w2:0,qx:0, qy:0, qz:-L1}))

Rv = (eye(3)*theta + (1 - cos(theta))*(skew_w) + (theta-sin(theta))*(skew_w**2))*v 

# Display results 

pprint(R.subs({theta:0.524,w0:0,w1:0.866,w2:0.5}))


T=Matrix([[R[0,0],R[0,1],R[0,2],Rv[0]],[R[1,0],R[1,1],R[1,2],Rv[1]],[R[2,0],R[2,1],R[2,2],Rv[2]],[0,0,0,1]])
'''
#Examples 
M=Matrix([[0,0,1,L1],[0,1,0,0],[-1,0,0,-L2],[0,0,0,1]])
T1=T.subs({theta:t1,w0:0,w1:0,w2:1,qx:0, qy:0, qz:0})
T2=T.subs({theta:t2,w0:0,w1:-1,w2:0,qx:L1, qy:0, qz:0})
T3=T.subs({theta:t3,w0:1,w1:0,w2:0,qx:0, qy:0, qz:-L2})
pprint(T1)
pprint(T2)
pprint(T3)
#pprint(T1*T2*T3*M)

T13=T1*T2*T3*M # Power of exponentials 
pprint(T13.subs({t1:0,t2:pi/2,t3:0}))
# exponential 
'''

M=Matrix([[1,0,0,0.7],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
T1=T.subs({theta:t1,w0:0,w1: 0,w2:1,qx:0, qy:0, qz:0})
T2=T.subs({theta:t2,w0:0,w1:-1,w2:0,qx:0, qy:0, qz:0})
T3=T.subs({theta:t3,w0:0,w1:-1,w2:0,qx:0.4, qy:0, qz:0})
T=T1*T2*T3*M
ts=[-2.318,0.930,1.147]
res=T.subs({t1:ts[0],t2:ts[1],t3:ts[2]})
pprint(res)
rotation = R_quat.from_matrix(np.array(res)[0:3,0:3])
quaternion = rotation.as_quat()
#angles come from joints GUI, only works with 2nd and 3rd joint. Watch recording
pprint(T[0,3].subs({t1:ts[0],t2:ts[1],t3:ts[2]}))
pprint(T[1,3].subs({t1:ts[0],t2:ts[1],t3:ts[2]}))
pprint(T[2,3].subs({t1:ts[0],t2:ts[1],t3:ts[2]}))
print(quaternion)

