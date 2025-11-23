#ROS 2 PoE Node.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import sympy as sp
from sympy import symbols, Matrix, eye, sin, cos, pprint, pi
import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R_quat

# -- Symbols definition
theta = symbols('theta', real=True)
w0, w1, w2 = symbols('omega_0 omega_1 omega_2', real=True)
qx, qy, qz = symbols('q_x q_y q_z', real=True)
L1, L2, L3 = symbols('L_1 L_2 L_3', real=True)
t1, t2, t3, t4, t5 = symbols('theta_1 theta_2 theta_3 theta_4 theta5', real=True)

# Symbolic twist axis
w = Matrix([w0, w1, w2])

# Skew-symmetric matrix
skew_w = Matrix([
    [0, -w[2], w[1]],
    [w[2], 0, -w[0]],
    [-w[1], w[0], 0]
])

# Rodrigues / Exponential map pieces
R = eye(3) + sin(theta)*skew_w + (1 - cos(theta))*(skew_w**2)
v = -skew_w*Matrix([[qx], [qy], [qz]])
Rv = (eye(3)*theta + (1 - cos(theta))*(skew_w) + (theta - sin(theta))*(skew_w**2)) * v

# Homogeneous Transform
T_elem = Matrix([[R[0,0], R[0,1], R[0,2], Rv[0]],
                 [R[1,0], R[1,1], R[1,2], Rv[1]],
                 [R[2,0], R[2,1], R[2,2], Rv[2]],
                 [0, 0, 0, 1]])

# Symbolic quaternion components (tool)
qtx, qty, qtz, qtw = symbols('q_tool_x q_tool_y q_tool_z q_tool_w', real=True)

# Normalize quaternion (to be safe if the numeric inputs are not normalized)
norm_q = sp.sqrt(qtx**2 + qty**2 + qtz**2 + qtw**2)
qx_n = qtx / norm_q
qy_n = qty / norm_q
qz_n = qtz / norm_q
qw_n = qtw / norm_q

# Rotation matrix from normalized quaternion (qw is scalar part)
Rot_tool = Matrix([
    [1 - 2*(qy_n**2 + qz_n**2),     2*(qx_n*qy_n - qz_n*qw_n),     2*(qx_n*qz_n + qy_n*qw_n)],
    [2*(qx_n*qy_n + qz_n*qw_n),     1 - 2*(qx_n**2 + qz_n**2),     2*(qy_n*qz_n - qx_n*qw_n)],
    [2*(qx_n*qz_n - qy_n*qw_n),     2*(qy_n*qz_n + qx_n*qw_n),     1 - 2*(qx_n**2 + qy_n**2)]
])

# Definitions of the tool quaternion and position values
# Numeric defaults for the tool quaternion (qx,qy,qz,qw) - unit quaternion
# represents no rotation.
TOOL_QX = -1.89428046581952e-08
TOOL_QY = 0.7072578072547913
TOOL_QZ = 1.8950899516312347e-08
TOOL_QW = 0.7069556713104248

# Build the 4x4 M with the tool rotation and the previously used translation
M_sym = sp.eye(4)
for i in range(3):
    for j in range(3):
        M_sym[i, j] = Rot_tool[i, j]
# translation values from original M
M_sym[0, 3] = 0.8949621915817261
M_sym[1, 3] = 0.0
M_sym[2, 3] = 1.263683557510376

# Definition of M (pose end-effector at  home)
# Substitute numeric tool quaternion so FK depends only on joint angles
M = sp.N(M_sym.subs({qtx: TOOL_QX, qty: TOOL_QY, qtz: TOOL_QZ, qtw: TOOL_QW}))
#pprint(M)

# Transfrorms for each joint and screws (axis and point)
T1 = T_elem.subs({theta: t1, w0: 0, w1: 0, w2: 1, qx: 0, qy: 0, qz: 0.44999998807907104})
T2 = T_elem.subs({theta: t2, w0: 0, w1: 1, w2: 0, qx: 0.1550000011920929, qy: 0, qz: 0.44999998807907104})
T3 = T_elem.subs({theta: t3, w0: 0, w1: -1, w2: 0, qx: 0.15487676858901978, qy: 0, qz: 1.0640000104904175})
T4 = T_elem.subs({theta: t4, w0: 0, w1: 1, w2: 0, qx: 0.7949622273445129, qy: 0, qz: 1.2637263536453247})
T5 = T_elem.subs({theta: t5, w0: -1, w1: 0, w2: 0, qx: 0.7949622273445129, qy: 0, qz: 1.2637263536453247})

# Final Transform for the robot
T_sym = sp.expand(T1 * T2 * T3 * T4 * T5 * M)

# Save symbolic matrix to a YAML file
def save_symbolic_matrix(matrix, filename):
    matrix_data = {
        'shape': matrix.shape,
        'elements': [[str(matrix[i,j]) for j in range(matrix.shape[1])] 
                    for i in range(matrix.shape[0])]
    }
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'w') as f:
        yaml.dump(matrix_data, f, default_flow_style=False)

matrix_file = os.path.join('data', 'T_sym_matrix.yaml')
os.makedirs('data', exist_ok=True)

save_symbolic_matrix(T_sym, matrix_file)

pprint(T_sym)

# Lambdify for numeric evaluation
fk_func = sp.lambdify((t1, t2, t3, t4, t5), T_sym, modules=['numpy'])

class PoEfwMH12Node(Node):
    def __init__(self):
        super().__init__('poe_mh12_node')
        # Default joint values
        self.j1 = 0.0
        self.j2 = 0.0
        self.j3 = 0.0
        self.j4 = 0.0
        self.j5 = 0.0

        # Subscription `joint_states`
        self.sub = self.create_subscription(JointState, 'joint_states', self.cb_joint_states, 10)

        self.get_logger().info('Nodo PoE RRR iniciado. Suscrito a: /joint_states')

    def cb_joint_states(self, msg: JointState):
        #Callback for search JointState names.
        try:
            names = list(msg.name)
            positions = list(msg.position)
        except Exception:
            self.get_logger().warn('Mensaje JointState sin campos name/position válidos')
            return
        def get_by_name(n):
            if n in names:
                idx = names.index(n)
                if idx < len(positions):
                    return float(positions[idx])
            return None
        # correct joint name (URDF uses 'joint_1_s')
        v1s = get_by_name('joint_1_s')
        v2l = get_by_name('joint_2_l')
        v3u = get_by_name('joint_3_u')
        v5b = get_by_name('joint_5_b')
        v6t = get_by_name('joint_6_t')

        # Fallback:
        if v1s is None or v2l is None or v3u is None or v5b is None or v6t is None:
            if len(positions) >= 5:
                if v1s is None:
                    v1s = float(positions[0])
                if v2l is None:
                    v2l = float(positions[1])
                if v3u is None:
                    v3u = float(positions[2])
                if v5b is None:
                    v5b = float(positions[3])
                if v6t is None:
                    v6t = float(positions[4])
            else:
                self.get_logger().warn('JointState no contiene suficientes posiciones y no se encontraron nombres esperados')
                return
        self.j1 = v1s
        self.j2_= v2l
        self.j3 = v3u
        self.j4 = v5b
        self.j5 = v6t
        self._recompute_and_print()

    def _recompute_and_print(self):
        # Call to fk_func for the numeric evaluation
        
        j1 = getattr(self, 'j1', 0.0)
        j2 = getattr(self, 'j2_',0.0)
        j3 = getattr(self, 'j3', 0.0)
        j4 = getattr(self, 'j4', 0.0)
        j5 = getattr(self, 'j5', 0.0)
        try:
            T_num = fk_func(j1, j2, j3, j4, j5)
        except Exception as e:
            self.get_logger().error(f'Error al evaluar fk_func: {e}')
            return

        # numpy 4x4
        T_np = np.array(T_num, dtype=float)
        pos = T_np[0:3, 3]

        # Rotations 3x3 -> cuaternión (x, y, z, w)
        rot_mat = T_np[0:3, 0:3]
        try:
            rotation = R_quat.from_matrix(rot_mat)
            quat = rotation.as_quat()
        except Exception as e:
            self.get_logger().error(f'Error al convertir matriz a cuaternión: {e}')
            return

        #Position: [x, y, z]
        self.get_logger().info(f'Posición: x={pos[0]:.6f}, y={pos[1]:.6f}, z={pos[2]:.6f}')
        # Quaternion: [x, y, z, w]
        self.get_logger().info(f'Cuaternión: x={quat[0]:.6f}, y={quat[1]:.6f}, z={quat[2]:.6f}, w={quat[3]:.6f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoEfwMH12Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()