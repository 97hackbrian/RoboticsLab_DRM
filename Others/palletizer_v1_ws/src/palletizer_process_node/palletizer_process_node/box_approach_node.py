#!/usr/bin/env python3
"""
Box Approach Node for Palletizer

Monitors the Sharp IR distance sensor and signals when
the box is within capture distance (< 5cm).

Author: hackbrian
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class BoxApproachNode(Node):
    """
    Box proximity detection using Sharp IR sensor.
    
    Subscribes to /sharp_distance/range and publishes
    whether the box is close enough for capture.
    """

    def __init__(self):
        super().__init__('box_approach_node')
        
        # Parameters
        self.declare_parameter('capture_distance', 0.1)  # 10cm
        self.declare_parameter('sensor_topic', '/sharp_distance/range')
        
        self.capture_distance = self.get_parameter('capture_distance').value
        sensor_topic = self.get_parameter('sensor_topic').value
        
        # Current distance
        self.current_distance = float('inf')
        self.box_close = False
        
        # --- Publishers ---
        self.box_close_pub = self.create_publisher(
            Bool, '/palletizer/box_close', 10
        )
        
        # --- Subscribers ---
        self.range_sub = self.create_subscription(
            Range, sensor_topic, self.range_callback, 10
        )
        
        # Status update timer (10 Hz)
        self.timer = self.create_timer(0.1, self.status_update)
        
        self.get_logger().info(
            f'Box Approach node initialized. Capture distance: {self.capture_distance*100:.1f}cm'
        )

    def range_callback(self, msg: Range):
        """Update current distance from Sharp sensor."""
        self.current_distance = msg.range
        
        # Check if box is close enough
        was_close = self.box_close
        self.box_close = self.current_distance < self.capture_distance
        
        if self.box_close and not was_close:
            self.get_logger().info(
                f'Box within capture distance! Distance: {self.current_distance*100:.1f}cm'
            )

    def status_update(self):
        """Publish box proximity status."""
        msg = Bool()
        msg.data = self.box_close
        self.box_close_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoxApproachNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
