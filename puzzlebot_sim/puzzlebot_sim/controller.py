import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

class WallFollowerFSM(Node):
    def __init__(self):
        super().__init__('controller')

        # Subscripciones
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose2D, 'set_point', self.setpoint_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Bool, 'goal_reached', 10)

        # Variables
        self.x = self.y = self.theta = 0.0
        self.x_goal = self.y_goal = self.theta_goal = 0.0
        self.lidar = None
        self.goal_set = False
        self.state = 'GO_TO_GOAL'

        # Timer principal
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def setpoint_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta
        self.goal_set = True

    def lidar_callback(self, msg):
        self.lidar = msg
        # Reemplaza inf por un valor alto
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
                self.get_logger().info(" Obstáculo detectado, cambio a FOLLOW_WALL")
                self.state = 'FOLLOW_WALL'
                return
            self.move_to_goal(dx, dy)

        elif self.state == 'FOLLOW_WALL':
            if can_see_goal:
                self.get_logger().info(" Camino al goal despejado, regreso a GO_TO_GOAL")
                self.state = 'GO_TO_GOAL'
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
            self.get_logger().warn(" Muy pocos datos en el LiDAR.")
            return

        left = min(ranges[int(3*size/20):int(5*size/20)])
        front = self.get_front_distance()
        front_left = min(ranges[int(1*size/20):int(3*size/20)])
        back_left = min(ranges[int(5*size/20):int(7*size/20)])

        twist = Twist()
        error = left - 0.35

        #  Esquina interna (pared al frente)
        if front < 0.45:
            self.get_logger().info(" Esquina detectada: girando suavemente")
            twist.linear.x = 0.05
            twist.angular.z = -1.0

        # ↪ Esquina exterior (pared se abrió)
        elif back_left < 0.3 and left > 0.7 and front_left > 0.7 and front > 0.7:
            self.get_logger().info("↩ Esquina exterior: girando hacia la pared")
            twist.linear.x = 0.1
            twist.angular.z = 1.0

        #  Seguimiento normal
        else:
            twist.linear.x = 0.25
            twist.angular.z = -1.5 * error

        self.cmd_pub.publish(twist)

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
        front = self.get_front_distance()
        return abs(angle_diff) < 0.3 and front > 1.0

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
