import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion
import math

class OdomResetWrapper(Node):
    def __init__(self):
        super().__init__('odom_reset_wrapper')

        # --- Configuration ---
        self.input_odom_topic = '/odom'
        self.output_odom_topic = '/odom/sim'
        # Check your service list: usually /reset_positions is std_srvs/Empty
        self.stage_reset_service = '/reset_positions' 
        
        # --- Internal State ---
        # We store the raw odom pose at the moment of reset
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0
        self.reset_triggered = False

        # --- Subscribers & Publishers ---
        self.sub_odom = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            10
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            self.output_odom_topic,
            10
        )

        # --- Services ---
        # The service you call to reset everything
        self.srv_reset = self.create_service(
            Empty,
            'reset_sim',
            self.reset_sim_callback
        )

        # The client to talk to the simulator
        self.client_stage = self.create_client(Empty, self.stage_reset_service)

        self.get_logger().info("Odom Reset Wrapper Initialized. Waiting for /odom...")

    def reset_sim_callback(self, request, response):
        """
        1. Calls the simulator reset (teleports robot).
        2. Sets the flag to update the offset on the next odom msg.
        """
        self.get_logger().info("Reset requested: Teleporting robot and zeroing odom...")
        
        # 1. Call Stage /reset_positions
        if self.client_stage.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.client_stage.call_async(req)
        else:
            self.get_logger().error(f"Service {self.stage_reset_service} not available!")
            return response

        # 2. Trigger offset update (we do it in odom_callback to ensure sync)
        self.reset_triggered = True
        
        return response

    def odom_callback(self, msg: Odometry):
        # Extract current raw data
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        _, _, raw_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        # If a reset was just requested, capture the current values as the new "zero"
        if self.reset_triggered:
            self.offset_x = raw_x
            self.offset_y = raw_y
            self.offset_yaw = raw_yaw
            self.reset_triggered = False
            self.get_logger().info(f"Odom offset updated: x={raw_x:.2f}, y={raw_y:.2f}")

        # --- Transform ---
        # Compute corrected pose (Transformation Matrix concept: T_local = T_offset_inv * T_global)
        # Simplified 2D rotation/translation:
        
        dx = raw_x - self.offset_x
        dy = raw_y - self.offset_y
        
        # Rotate the delta vector by the negative offset yaw 
        # (This aligns the vector to the new 0,0 frame)
        # Formula: x' = x*cos(-theta) - y*sin(-theta)
        cos_inv = math.cos(-self.offset_yaw)
        sin_inv = math.sin(-self.offset_yaw)
        
        sim_x = dx * cos_inv - dy * sin_inv
        sim_y = dx * sin_inv + dy * cos_inv
        sim_yaw = raw_yaw - self.offset_yaw

        # Normalize yaw to [-pi, pi]
        sim_yaw = (sim_yaw + math.pi) % (2 * math.pi) - math.pi

        # --- Construct New Message ---
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id
        
        # Position
        new_msg.pose.pose.position.x = sim_x
        new_msg.pose.pose.position.y = sim_y
        new_msg.pose.pose.position.z = msg.pose.pose.position.z # Keep Z as is (usually 0)

        # Orientation
        new_msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, sim_yaw)

        # Twist (Velocity) - usually we pass this through directly, 
        # or we might need to rotate the linear vector if it's in the global frame.
        # Standard Odom twist is in Child Frame (Base Link), so it does NOT change!
        new_msg.twist = msg.twist

        self.pub_odom.publish(new_msg)

    # --- Math Helpers ---
    def euler_from_quaternion(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = OdomResetWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
