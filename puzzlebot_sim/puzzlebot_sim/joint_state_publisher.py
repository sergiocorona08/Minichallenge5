import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.02, self.timer_cb)

        self.msgJoints = JointState()
        self.msgJoints.name = ['wheel_l_joint', 'wheel_r_joint']
        self.msgJoints.position = [0.0, 0.0]
        self.msgJoints.velocity = [0.0, 0.0]
        self.msgJoints.effort = [0.0, 0.0]

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.r = 0.05
        self.L = 0.19

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

        self.define_TF()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]

        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def timer_cb(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        wr = (2 * self.v + self.w * self.L) / (2 * self.r)
        wl = (2 * self.v - self.w * self.L) / (2 * self.r)

        self.msgJoints.header.stamp = self.get_clock().now().to_msg()
        self.msgJoints.position[0] += wl * dt
        self.msgJoints.position[1] += wr * dt

        self.publisher.publish(self.msgJoints)

        self.tf_broadcaster.sendTransform(self.create_transform(
            parent_frame="odom",
            child_frame="base_footprint",
            x=self.x, y=self.y, z=0.0,
            roll=0.0, pitch=0.0, yaw=self.yaw
        ))

    def define_TF(self):
        static_transforms = [
            self.create_transform(
                parent_frame="map",
                child_frame="odom",
                x=0.0, y=0.0, z=0.0,
                roll=0.0, pitch=0.0, yaw=0.0
            )]
        self.tf_static_broadcaster.sendTransform(static_transforms)

    def create_transform(self, parent_frame, child_frame, x, y, z, roll, pitch, yaw):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z

        q = transforms3d.euler.euler2quat(roll, pitch, yaw)
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        tf.transform.rotation.w = q[0]
        return tf

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
