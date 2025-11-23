import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import sys


joint_names=['joint1','joint1_1','joint2']

class JointPublisher(Node):
    
	def __init__(self):
		super().__init__('joint_publisher')
		self.declare_parameter('j1', 0.0)
		self.declare_parameter('j1_1', 0.0)
		self.declare_parameter('j2', 0.0)

		self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
		self.published_once = False 
		self.timer_ = self.create_timer(1.0, self.publish_message_once)
		self.joint_angles=[self.get_parameter('j1').get_parameter_value().double_value,
			self.get_parameter('j1_1').get_parameter_value().double_value, 
			self.get_parameter('j2').get_parameter_value().double_value]
		
	def publish_message_once(self):
		if not self.published_once:
			joint_msg=JointState()
			header_msg = Header()
			header_msg.stamp = self.get_clock().now().to_msg()
			joint_msg.header = header_msg
			
			print(self.joint_angles)
			joint_msg.name=joint_names
			joint_msg.position=self.joint_angles
			self.publisher_.publish(joint_msg)
			self.get_logger().info('joints published')
			self.published_once = True
			#self.destroy_timer(self.timer_) 
			self.destroy_node()
			rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    joints_pub = JointPublisher()

    rclpy.spin(joints_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #rclpy.spin_once(joints_pub, timeout_sec=1.0)
    
    #joints_pub.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
