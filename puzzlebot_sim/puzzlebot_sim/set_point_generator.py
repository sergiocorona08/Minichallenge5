import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

class SetPointGenerator(Node):
    def __init__(self):
        super().__init__('set_point_generator')

        self.publisher = self.create_publisher(Pose2D, 'set_point', 10)
        self.create_subscription(Bool, 'goal_reached', self.goal_callback, 10)

        self.goal_sent = False
        self.goal_reached = False

        # Coordenadas 
        self.goal_x = -3.0
        self.goal_y = 3.0
        self.goal_theta = 0.0

        self.timer = self.create_timer(0.5, self.timer_callback)

    def goal_callback(self, msg):
        self.goal_reached = msg.data
        if self.goal_reached:
            self.get_logger().info("¡Goal alcanzado!")

    def timer_callback(self):
        if not self.goal_sent and not self.goal_reached:
            point = Pose2D()
            point.x = self.goal_x
            point.y = self.goal_y
            point.theta = self.goal_theta
            self.publisher.publish(point)
            self.get_logger().info(f"Enviando goal: ({point.x:.2f}, {point.y:.2f})")
            self.goal_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = SetPointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
