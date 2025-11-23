#!/usr/bin/env python3

# RRT Motion Planning

import numpy as np
import random
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_msgs.srv
import transforms3d.affines
import transforms3d.quaternions


def convert_to_message(T):
    t = Pose()
    position, Rot, _, _ = transforms3d.affines.decompose(T)
    orientation = transforms3d.quaternions.mat2quat(Rot)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]        
    return t


class RRTBranch:
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        self.ee_goal = None
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.q_current = []
        self.current_obstacle = "NONE"

        # -------------------------------------------------------------
        # Obtener URDF directamente desde /robot_description
        # -------------------------------------------------------------
        robot_description_str = self.get_parameter('/robot_description').get_parameter_value().string_value
        self.robot = URDF.from_xml_string(robot_description_str)


        
        self.base = self.robot.get_root()
        self.get_joint_info()

        # -------------------------------------------------------------
        # Servicios MoveIt
        # -------------------------------------------------------------
        self.service_cb_group1 = MutuallyExclusiveCallbackGroup()
        self.service_cb_group2 = MutuallyExclusiveCallbackGroup()

        self.ik_service = self.create_client(
            moveit_msgs.srv.GetPositionIK, '/compute_ik', callback_group=self.service_cb_group1)
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service ready')

        self.state_valid_service = self.create_client(
            moveit_msgs.srv.GetStateValidity, '/check_state_validity', callback_group=self.service_cb_group2)
        while not self.state_valid_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state validity service...')
        self.get_logger().info('State validity service ready')

        self.group_name = 'arm'

        # -------------------------------------------------------------
        # Suscripciones
        # -------------------------------------------------------------
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.get_joint_state, 10)
        self.goal_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_goal = self.create_subscription(Transform, '/motion_planning_goal', self.motion_planning_cb, 2,
                                                 callback_group=self.goal_cb_group)
        self.sub_obs = self.create_subscription(String, '/obstacle', self.get_obstacle, 10)

        # -------------------------------------------------------------
        # Publicador
        # -------------------------------------------------------------
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.motion_planning_timer, callback_group=self.timer_cb_group)

        self.get_logger().info("MoveArm node initialized successfully")

    # -------------------------------------------------------------
    # Función para leer /robot_description
    # -------------------------------------------------------------
    def get_robot_description(self, timeout=2.0):
        """Lee el URDF desde /robot_description como String."""
        robot_desc = None

        def callback(msg):
            nonlocal robot_desc
            robot_desc = msg.data

        sub = self.create_subscription(String, '/robot_description', callback, 10)

        import time
        t0 = time.time()
        while robot_desc is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)
        return robot_desc

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------
    def get_joint_state(self, msg):
        self.q_current = [msg.position[msg.name.index(name)] for name in self.joint_names]

    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    def motion_planning_cb(self, ee_goal):
        if self.ee_goal is not None:
            self.get_logger().info("Motion planner busy. Try again later.")
            return
        self.ee_goal = ee_goal
        self.get_logger().info("Motion planner goal received.")

    def motion_planning_timer(self):
        if self.ee_goal is not None:
            self.get_logger().info("Calling motion planner...")
            self.motion_planning(self.ee_goal)
            self.ee_goal = None
            self.get_logger().info("Motion planner done")

    # -------------------------------------------------------------
    # Movimiento
    # -------------------------------------------------------------
    def motion_planning(self, ee_goal: Transform):
        # RRT parameters
        num_iterations = 3000
        step_size = 0.1
        goal_tolerance = 0.05

        # Transform EE
        q_goal_mat = self.transform_to_matrix(ee_goal)
        q_goal = self.IK(q_goal_mat)
        if not self.is_state_valid(q_goal):
            self.get_logger().info("Goal is invalid or in collision")
            return

        initial_node = RRTBranch(None, self.q_current)
        tree = [initial_node]

        for _ in range(num_iterations):
            q_rand = np.array([random.uniform(-math.pi, math.pi) for _ in range(self.num_joints)])
            q_rand = np.array(self.interpolate_towards_goal(q_rand, q_goal, 0.2))
            closest_node = self.find_closest_point_in_tree(tree, q_rand)
            q_new = self.extend_tree(closest_node.q, q_goal, step_size)
            if self.is_state_valid(q_new):
                tree.append(RRTBranch(closest_node, q_new))
            if np.linalg.norm(np.array(q_new) - np.array(q_goal)) < goal_tolerance:
                break

        trajectory = self.trajectory_generation(tree, q_goal)
        self.execute_path(trajectory)

    # -------------------------------------------------------------
    # Funciones auxiliares
    # -------------------------------------------------------------
    def transform_to_matrix(self, transform):
        t = [transform.translation.x, transform.translation.y, transform.translation.z]
        r = transforms3d.quaternions.quat2mat(
            [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]
        )
        return transforms3d.affines.compose(t, r, np.ones(3))

    def interpolate_towards_goal(self, start_config, goal_config, bias_factor):
        direction = np.array(goal_config) - np.array(start_config)
        return (np.array(start_config) + bias_factor * direction).tolist()

    def extend_tree(self, from_q, to_q, step_size):
        delta_q = np.array(to_q) - np.array(from_q)
        dist = np.linalg.norm(delta_q)
        if dist <= step_size:
            return to_q
        return np.clip(np.array(from_q) + step_size * delta_q / dist, -math.pi, math.pi)

    def trajectory_generation(self, tree, q_goal):
        path = [q_goal]
        node = tree[-1]
        while node.parent:
            path.append(node.q)
            node = node.parent
        path.reverse()

        step_size = 0.1
        discretized = [path[0]]
        for to_q in path[1:]:
            from_q = discretized[-1]
            delta = np.array(to_q) - np.array(from_q)
            while np.linalg.norm(delta) > step_size:
                delta = np.array(to_q) - np.array(from_q)
                inter = np.array(from_q) + step_size * delta / np.linalg.norm(delta)
                discretized.append(inter.tolist())
                from_q = inter
            discretized.append(to_q)
        return discretized

    def execute_path(self, trajectory):
        if not trajectory:
            self.get_logger().info("Empty path, nothing to execute")
            return
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint(positions=q) for q in trajectory]
        self.pub.publish(msg)

    def find_closest_point_in_tree(self, tree, r):
        closest = min(tree, key=lambda n: np.linalg.norm(np.array(n.q) - np.array(r)))
        return closest

    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = list(np.zeros(self.num_joints))
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = 'base'
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rclpy.duration.Duration(seconds=5).to_msg()
        res = self.ik_service.call(req)

        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = list(res.solution.joint_state.position)
        for i in range(len(q)):
            while q[i] < -math.pi: q[i] += 2 * math.pi
            while q[i] > math.pi: q[i] -= 2 * math.pi
        return q

    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map:
                break
            joint_name, next_link = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]
            if joint.type != 'fixed':
                self.num_joints += 1
                self.joint_names.append(joint.name)
                self.joint_axes.append(joint.axis)
            link = next_link
        self.get_logger().info(f"Num joints: {self.num_joints}")

    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidity.Request()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = list(q)
        res = self.state_valid_service.call(req)
        return res.valid


def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

