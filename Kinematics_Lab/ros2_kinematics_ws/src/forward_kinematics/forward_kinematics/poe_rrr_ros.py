"""Nodo ROS 2 para cinemática directa usando PoE (3-DOF RRR).

Este archivo reutiliza las expresiones simbólicas definidas originalmente
y crea un nodo `rclpy` que se suscribe a los tópicos de las articulaciones
`/join1`, `/joint1_1`, `/joint2` (tipo `std_msgs/Float64`). Cada vez que
se recibe un ángulo se recalcula la transformada homogénea final y se imprime
la posición (x,y,z) y la orientación en cuaternión (x,y,z,w).

Supuestos razonables:
- Los tópicos de las articulaciones publican `std_msgs/Float64` con el ángulo
  en radianes.
- Si aún no se han recibido los tres valores, se usan ceros por defecto.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import sympy as sp
from sympy import symbols, Matrix, eye, sin, cos
import numpy as np
from scipy.spatial.transform import Rotation as R_quat

# -- símbolos y construcción simbólica (reutiliza la lógica PoE del script)
theta = symbols('theta', real=True)
w0, w1, w2 = symbols('omega_0 omega_1 omega_2', real=True)
qx, qy, qz = symbols('q_x q_y q_z', real=True)
L1, L2, L3 = symbols('L_1 L_2 L_3', real=True)
t1, t2, t3 = symbols('theta_1 theta_2 theta_3', real=True)

# Vectores simbólicos
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

# Transform homogénea simbólica del movimiento elemental
T_elem = Matrix([[R[0,0], R[0,1], R[0,2], Rv[0]],
                 [R[1,0], R[1,1], R[1,2], Rv[1]],
                 [R[2,0], R[2,1], R[2,2], Rv[2]],
                 [0, 0, 0, 1]])

# Definición M (pose del end-effector en home)
M = Matrix([[1,0,0,0.7],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

# Sustituciones concretas para los tres ejes (coherentes con el script)
T1 = T_elem.subs({theta: t1, w0: 0, w1: 0, w2: 1, qx: 0, qy: 0, qz: 0})
T2 = T_elem.subs({theta: t2, w0: 0, w1: -1, w2: 0, qx: 0, qy: 0, qz: 0})
T3 = T_elem.subs({theta: t3, w0: 0, w1: -1, w2: 0, qx: 0.4, qy: 0, qz: 0})

# Transformada total simbólica
T_sym = sp.expand(T1 * T2 * T3 * M)

# Lambdify para obtener matriz numérica a partir de (t1,t2,t3)
fk_func = sp.lambdify((t1, t2, t3), T_sym, modules=['numpy'])


class PoERRRNode(Node):
    def __init__(self):
        super().__init__('poe_rrr_node')
        # Últimos valores de joints (inicialmente 0)
        self.j1 = 0.0
        self.j1_1 = 0.0
        self.j2 = 0.0

        # Suscripción al topic `joint_states` que contiene todas las articulaciones
        self.sub = self.create_subscription(JointState, 'joint_states', self.cb_joint_states, 10)

        self.get_logger().info('Nodo PoE RRR iniciado. Suscrito a: /joint_states')

    def cb_joint_states(self, msg: JointState):
        """Callback que extrae las tres articulaciones esperadas.

        Busca en msg.name los nombres 'join1', 'joint1_1', 'joint2'. Si no los
        encuentra, toma un fallback: las tres primeras posiciones si están
        disponibles.
        """
        # Preferimos buscar por nombre si están presentes
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

        v1 = get_by_name('join1')
        v1_1 = get_by_name('joint1_1')
        v2 = get_by_name('joint2')

        # Fallback: usar las tres primeras posiciones si las búsquedas por nombre fallan
        if v1 is None or v1_1 is None or v2 is None:
            if len(positions) >= 3:
                # asignar solo las que faltan
                if v1 is None:
                    v1 = float(positions[0])
                if v1_1 is None:
                    v1_1 = float(positions[1])
                if v2 is None:
                    v2 = float(positions[2])
            else:
                self.get_logger().warn('JointState no contiene suficientes posiciones y no se encontraron nombres esperados')
                return

        # Actualizar y recomputar
        self.j1 = v1
        self.j1_1 = v1_1
        self.j2 = v2
        self._recompute_and_print()

    def _recompute_and_print(self):
        # Llamada al fk_func para obtener la matriz transformada
        try:
            T_num = fk_func(self.j1, self.j1_1, self.j2)
        except Exception as e:
            self.get_logger().error(f'Error al evaluar fk_func: {e}')
            return

        # Asegurarnos de tener un array numpy 4x4
        T_np = np.array(T_num, dtype=float)
        pos = T_np[0:3, 3]

        # Rotación 3x3 -> cuaternión (x, y, z, w)
        rot_mat = T_np[0:3, 0:3]
        try:
            rotation = R_quat.from_matrix(rot_mat)
            quat = rotation.as_quat()
        except Exception as e:
            self.get_logger().error(f'Error al convertir matriz a cuaternión: {e}')
            return

        # Imprimir en consola
        self.get_logger().info(f'Posición: x={pos[0]:.6f}, y={pos[1]:.6f}, z={pos[2]:.6f}')
        # Quaternion: [x, y, z, w]
        self.get_logger().info(f'Cuaternión: x={quat[0]:.6f}, y={quat[1]:.6f}, z={quat[2]:.6f}, w={quat[3]:.6f}')


def main(args=None):
    rclpy.init(args=args)
    node = PoERRRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


