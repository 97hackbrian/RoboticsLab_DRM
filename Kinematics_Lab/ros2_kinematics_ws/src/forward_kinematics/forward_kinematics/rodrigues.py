import sympy as sp
from sympy import symbols, Matrix, eye, sin, cos, expand, pprint

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

# Display results 

pprint(R.subs({theta:0.524,w0:0,w1:0.866,w2:0.5}))
