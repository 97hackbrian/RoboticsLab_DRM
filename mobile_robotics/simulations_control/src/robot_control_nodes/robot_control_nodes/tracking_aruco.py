import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

import cv2
import numpy as np


class TrackingAruco(Node):
    """
    Node for tracking ArUco markers and publishing angular velocity commands.
    Uses ApproximateTimeSynchronizer for camera image synchronization.
    """

    def __init__(self):
        super().__init__("tracking_aruco")

        self.bridge = CvBridge()

        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Subscriber to camera images
        self.sub_img = self.create_subscription(
            Image, "/camera/image_raw", self.img_callback, 10
        )

        # Publisher for angular velocity commands (TwistStamped)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel_angular', 10)

        # Proportional control gain
        self.k = 2.0

        # Image center (calculated on first image)
        self.center_img = None
        self.img_width = None

        self.get_logger().info('Tracking ArUco node started. Waiting for camera images...')

    def img_callback(self, msg):
        """
        Callback for processing camera images and tracking ArUco markers.
        """
        # Convert ROS Image message to OpenCV format
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Calculate image center on first frame
        if self.center_img is None:
            height, width = cv_img.shape[:2]
            self.center_img = np.array([width / 2.0, height / 2.0])
            self.img_width = width
            self.get_logger().info(f'Image center initialized: {self.center_img}')

        # Detect ArUco markers
        corners, ids, rej = cv2.aruco.detectMarkers(
            cv_img, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
            markers_corners = corners[0][0]
            marker_center = np.mean(markers_corners, axis=0)

            error = (marker_center[0] - self.center_img[0]) / self.img_width

            p_out = self.k * error

            p_out = np.clip(p_out, -0.6, 0.6)

            print(p_out)

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.twist.angular.z = -p_out

            self.pub.publish(msg)

        cv2.imshow("robot_vis", cv_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingAruco()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
