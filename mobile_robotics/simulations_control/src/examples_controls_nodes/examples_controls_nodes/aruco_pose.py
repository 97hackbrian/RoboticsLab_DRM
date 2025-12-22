import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

import cv2
import numpy as np
import math


class ArucoTfPublisher(Node):

    def __init__(self):
        super().__init__("aruco_tf_publisher")

        self.declare_parameter('camera_frame', 'camera_link')
        self.camera_frame = self.get_parameter('camera_frame').value

        self.declare_parameter('use_optical_frame', True)
        self.use_optical_frame = self.get_parameter('use_optical_frame').value

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.marker_size = 0.10

        ms = self.marker_size / 2.0
        self.marker_points = np.array(
            [[-ms, ms, 0], [ms, ms, 0], [ms, -ms, 0], [-ms, -ms, 0]],
            dtype=np.float32)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()

        self.sub_info = self.create_subscription(CameraInfo,
                                                 '/camera/camera_info',
                                                 self.info_callback, 10)

        self.sub_img = self.create_subscription(Image, "/camera/image_raw",
                                                self.img_callback, 10)

        self.get_logger().info(
            f"Nodo ArUco TF Publisher iniciado. Camera frame: {self.camera_frame}, "
            f"Optical frame: {self.use_optical_frame}")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            try:
                self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(
                    (3, 3))
                d = list(msg.d) if msg.d else []
                while len(d) < 5:
                    d.append(0.0)
                self.dist_coeffs = np.array(d[:5], dtype=np.float64)
                self.get_logger().info("Info de camara configurada.")
            except Exception as e:
                self.get_logger().error(f"Error configurando camara: {e}")

    def img_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        except Exception as e:
            self.get_logger().error(f"Error en cv_bridge: {e}")
            return

        try:
            gray = np.ascontiguousarray(gray, dtype=np.uint8)

            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and len(ids) > 0:
                for i in range(len(ids)):
                    try:
                        c = corners[i][0].astype(np.float32)

                        success, rvec, tvec = cv2.solvePnP(
                            self.marker_points,
                            c,
                            self.camera_matrix,
                            self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE)

                        if success:
                            self.publish_tf(rvec, tvec, ids[i][0], msg.header)

                    except Exception as e:
                        self.get_logger().error(f"Error en solvePnP: {e}")

        except Exception as e:
            self.get_logger().error(f"Error en detección: {e}")

    def publish_tf(self, rvec, tvec, marker_id, header):
        try:
            t = TransformStamped()
            t.header.stamp = header.stamp
            t.header.frame_id = self.camera_frame  # Use configured camera frame
            t.child_frame_id = f"aruco_marker_{marker_id}"

            tvec = np.array(tvec).flatten()
            rvec_flat = np.array(rvec).flatten()
            R, _ = cv2.Rodrigues(rvec_flat)

            if self.use_optical_frame:
                R_flip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]],
                                  dtype=np.float64)
                R_corrected = R @ R_flip

                t.transform.translation.x = float(tvec[2])
                t.transform.translation.y = float(-tvec[0])
                t.transform.translation.z = float(-tvec[1])

                R_optical_to_robot = np.array(
                    [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
                R_corrected = R_optical_to_robot @ R_corrected
            else:
                R_corrected = R
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])

            q = self.rotation_matrix_to_quaternion(R_corrected)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error publicando TF: {e}")

    def rotation_matrix_to_quaternion(self, R):
        """Convierte una matriz de rotación a quaternion"""
        try:
            tr = np.trace(R)

            if tr > 0:
                S = math.sqrt(tr + 1.0) * 2
                qw = 0.25 * S
                qx = (R[2, 1] - R[1, 2]) / S
                qy = (R[0, 2] - R[2, 0]) / S
                qz = (R[1, 0] - R[0, 1]) / S
            elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                qw = (R[2, 1] - R[1, 2]) / S
                qx = 0.25 * S
                qy = (R[0, 1] + R[1, 0]) / S
                qz = (R[0, 2] + R[2, 0]) / S
            elif R[1, 1] > R[2, 2]:
                S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                qw = (R[0, 2] - R[2, 0]) / S
                qx = (R[0, 1] + R[1, 0]) / S
                qy = 0.25 * S
                qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                qw = (R[1, 0] - R[0, 1]) / S
                qx = (R[0, 2] + R[2, 0]) / S
                qy = (R[1, 2] + R[2, 1]) / S
                qz = 0.25 * S

            return [qx, qy, qz, qw]

        except Exception as e:
            self.get_logger().error(f"Error en conversión: {e}")
            return [0.0, 0.0, 0.0, 1.0]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
