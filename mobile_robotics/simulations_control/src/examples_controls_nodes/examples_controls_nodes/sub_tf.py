import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfSubscriber(Node):

    def __init__(self):
        super().__init__('tf_subscriber_node')

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.target_frame = 'base_link'
        self.source_frame = 'aruco_marker_1'

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(self.target_frame,
                                                self.source_frame,
                                                rclpy.time.Time())

            trans = t.transform.translation
            rot = t.transform.rotation

            self.get_logger().info(
                f'Marker Position: x={trans.x:.2f}, y={trans.y:.2f}, z={trans.z:.2f}'
            )

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}'
            )


def main():
    rclpy.init()
    node = TfSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
