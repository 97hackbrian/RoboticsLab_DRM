#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
import ast
import math
import time

class DirectKinematics(Node):
    def __init__(self):
        super().__init__('direct_kinematics')

        # ---- Parámetros ----
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tool_frame', 'tool_link')
        self.declare_parameter(
            'sequence',
            [
                '[[0.0, 0.0, 0.0], 2.0]',
                '[[0.5, 0.0, 0.0], 2.0]',
                '[[0.5, 0.5, 0.0], 2.0]',
                '[[0.5, 0.5, 0.5], 2.0]',
                '[[0.0, 0.5, 0.5], 2.0]',
                '[[0.0, 0.0, 0.5], 2.0]',
                '[[0.0, 0.0, 0.0], 2.0]'
            ]
        )

        # ---- Leer parámetros ----
        self.joint_names = self.get_parameter('joint_names').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value

        sequence_raw = self.get_parameter('sequence').value
        self.sequence = []
        for s in sequence_raw:
            try:
                self.sequence.append(ast.literal_eval(s))
            except Exception as e:
                self.get_logger().warn(f"No se pudo interpretar la secuencia: {s} -> {e}")

        # ---- Publicador de joints ----
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # ---- TF Listener ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Secuencia ----
        self.current_step = 0
        self.step_start_time = self.get_clock().now().nanoseconds / 1e9  # tiempo en segundos
        self.get_logger().info(f"Nodo 'direct_kinematics' iniciado con joints: {self.joint_names}")
        self.get_logger().info(f"Secuencia cargada ({len(self.sequence)} pasos)")

        # ---- Timer a 20 Hz para publicar continuamente ----
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # ---- Obtener la posición actual de la secuencia ----
        if self.current_step < len(self.sequence):
            positions, duration = self.sequence[self.current_step]
            elapsed = now - self.step_start_time

            # ---- Publicar estado de las juntas continuamente ----
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = positions
            self.publisher.publish(msg)

            # ---- Intentar leer transform del efector ----
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tool_frame,
                    rclpy.time.Time()
                )
                t = transform.transform.translation
                r = transform.transform.rotation
                roll, pitch, yaw = self.quaternion_to_euler(r.x, r.y, r.z, r.w)
                self.get_logger().info(
                    f"📍 Posición: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f} | "
                    f"🔁 Rotación (rpy): roll={math.degrees(roll):.1f}°, "
                    f"pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°"
                )
            except Exception:
                pass  # TF puede no estar listo todavía

            # ---- Avanzar al siguiente paso si terminó el tiempo ----
            if elapsed >= duration:
                self.current_step += 1
                self.step_start_time = now
                self.get_logger().info(f"➡️ Avanzando al paso {self.current_step + 1}/{len(self.sequence)}")

        else:
            self.get_logger().info("✅ Secuencia completada")
            # Mantener publicando la última posición para TF
            positions, _ = self.sequence[-1]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = positions
            self.publisher.publish(msg)
            # No hacemos shutdown, el flujo es continuo como joint_state_publisher_gui

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


def main(args=None):
    rclpy.init(args=args)
    node = DirectKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

