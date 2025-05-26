import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import time

class WallFollowerFSM(Node):
    def __init__(self):
        super().__init__('controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose2D, 'set_point', self.setpoint_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Bool, 'goal_reached', 10)

        self.x = self.y = self.theta = 0.0
        self.x_goal = self.y_goal = self.theta_goal = 0.0
        self.lidar = None
        self.goal_set = False
        self.state = 'GO_TO_GOAL'

        self.m_line_start = (0.0, 0.0)
        self.m_line_end = (0.0, 0.0)
        self.m_line_angle = 0.0
        self.m_line_projection = 0.0
        self.avoidance_start_point = (0.0, 0.0)
        self.avoidance_start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        if self.goal_set:
            dx = self.x - self.m_line_start[0]
            dy = self.y - self.m_line_start[1]
            self.m_line_projection = dx * np.cos(self.m_line_angle) + dy * np.sin(self.m_line_angle)

    def setpoint_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta
        self.goal_set = True

        self.m_line_start = (self.x, self.y)
        self.m_line_end = (self.x_goal, self.y_goal)
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        self.m_line_angle = np.arctan2(dy, dx)
        self.m_line_projection = 0.0

    def lidar_callback(self, msg):
        self.lidar = msg
        self.lidar.ranges = [r if not np.isinf(r) else 100.0 for r in msg.ranges]

    def control_loop(self):
        if not self.goal_set or self.lidar is None:
            return

        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        rho = np.hypot(dx, dy)

        if rho < 0.1:
            self.cmd_pub.publish(Twist())
            self.goal_pub.publish(Bool(data=True))
            return
        else:
            self.goal_pub.publish(Bool(data=False))

        front_dist = self.get_front_distance()
        can_see_goal = self.check_goal_path_clear()

        if self.state == 'GO_TO_GOAL':
            if front_dist < 0.5:
                self.get_logger().info("Obstáculo detectado, cambio a FOLLOW_WALL")
                self.state = 'FOLLOW_WALL'
                self.avoidance_start_point = (self.x, self.y)
                self.avoidance_start_time = time.time()
                return
            self.move_to_goal(dx, dy)

        elif self.state == 'FOLLOW_WALL':
            if self.avoidance_start_time is None:
                self.avoidance_start_time = time.time()

            elapsed = time.time() - self.avoidance_start_time
            if elapsed > 3.0 and self.can_return_to_mline() and can_see_goal:
                self.get_logger().info("Camino al goal despejado, regreso a GO_TO_GOAL")
                self.state = 'GO_TO_GOAL'
                self.avoidance_start_time = None
                return
            self.follow_wall()

    def move_to_goal(self, dx, dy):
        angle_to_goal = np.arctan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.theta)

        v = min(0.5, 0.8 * np.hypot(dx, dy))
        w = 2.0 * angle_diff

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def follow_wall(self):
        ranges = self.lidar.ranges
        size = len(ranges)

        if size < 20:
            self.get_logger().warn("Muy pocos datos en el LiDAR.")
            return

        left = min(ranges[int(3*size/20):int(5*size/20)])
        front = self.get_front_distance()
        front_left = min(ranges[int(1*size/20):int(3*size/20)])
        back_left = min(ranges[int(5*size/20):int(7*size/20)])

        twist = Twist()
        error = left - 0.35

        if front < 0.45:
            self.get_logger().info("Esquina detectada: girando suavemente")
            twist.linear.x = 0.05
            twist.angular.z = -1.0
        elif back_left < 0.3 and left > 0.7 and front_left > 0.7 and front > 0.7:
            self.get_logger().info("Esquina exterior: girando hacia la pared")
            twist.linear.x = 0.1
            twist.angular.z = 1.0
        else:
            twist.linear.x = 0.25
            twist.angular.z = -1.5 * error

        self.cmd_pub.publish(twist)

    def can_return_to_mline(self):
        dx = self.x - self.m_line_start[0]
        dy = self.y - self.m_line_start[1]
        perp_distance = abs(dx * np.sin(self.m_line_angle) - dy * np.cos(self.m_line_angle))

        dx_start = self.avoidance_start_point[0] - self.m_line_start[0]
        dy_start = self.avoidance_start_point[1] - self.m_line_start[1]
        start_projection = dx_start * np.cos(self.m_line_angle) + dy_start * np.sin(self.m_line_angle)

        return perp_distance < 0.2 and self.m_line_projection > start_projection

    def get_front_distance(self):
        ranges = self.lidar.ranges
        size = len(ranges)
        if size < 20:
            return 100.0
        front = ranges[0:int(size/20)] + ranges[int(19*size/20):]
        return min(front)

    def check_goal_path_clear(self):
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        angle_to_goal = np.arctan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.theta)

        ranges = self.lidar.ranges
        size = len(ranges)
        if size < 20:
            return False

        angle_resolution = 2 * np.pi / size
        goal_index = int((angle_diff + np.pi) / angle_resolution)

        window = 5
        indices = [(goal_index + i) % size for i in range(-window, window + 1)]
        distances = [ranges[i] for i in indices]

        return all(d > 0.8 for d in distances)

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
