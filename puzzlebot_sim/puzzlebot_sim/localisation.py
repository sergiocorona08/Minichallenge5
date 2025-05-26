import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy import qos
import numpy as np
import transforms3d

class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')

        # Subscripciones
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, qos.qos_profile_sensor_data)

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.wr_pub = self.create_publisher(Float32, 'wr_loc', qos.qos_profile_sensor_data)
        self.wl_pub = self.create_publisher(Float32, 'wl_loc', qos.qos_profile_sensor_data)

        # Constantes del robot
        self.r = 0.05  # Radio de rueda
        self.L = 0.19  # Separación entre ruedas

        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wr = 0.0
        self.wl = 0.0

        # Covarianza inicial (aumentada para mejor visualización)
        self.P = np.diag([0.1, 0.1, 0.5])  # x, y, theta

        # Parámetros del modelo de ruido (aumentados)
        self.alpha1 = 0.5  # ruido en v (aumentado de 0.2)
        self.alpha2 = 0.3  # ruido en w (aumentado de 0.1)

        # Tiempo
        self.prev_time = self.get_clock().now().nanoseconds
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds
        dt = (now - self.prev_time) * 1e-9
        self.prev_time = now

        # Solo actualizar si hay movimiento significativo
        if abs(self.wr) > 0.01 or abs(self.wl) > 0.01:
            v = self.r * (self.wr + self.wl) / 2.0
            w = self.r * (self.wr - self.wl) / self.L

            self.update_pose(v, w, dt)
            self.update_covariance(v, w, dt)
            self.publish_odometry(v, w)
            self.publish_tf()
        
        self.publish_wheels()

    def update_pose(self, v, w, dt):
        delta_x = v * np.cos(self.theta) * dt
        delta_y = v * np.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

    def update_covariance(self, v, w, dt):
        # Jacobiano del modelo de movimiento respecto al estado
        J_h = np.array([
            [1, 0, -v * dt * np.sin(self.theta)],
            [0, 1,  v * dt * np.cos(self.theta)],
            [0, 0,  1]
        ])

        # Jacobiano del modelo de movimiento respecto al ruido de control
        # (modificado para mejor representación del ruido en las ruedas)
        J_u = np.array([
            [0.5 * self.r * dt * np.cos(self.theta), 0.5 * self.r * dt * np.cos(self.theta)],
            [0.5 * self.r * dt * np.sin(self.theta), 0.5 * self.r * dt * np.sin(self.theta)],
            [self.r * dt / self.L, -self.r * dt / self.L]
        ])

        # Covarianza del ruido de control (aumentada)
        # Usamos valores absolutos para evitar valores negativos con velocidades inversas
        Q_control = np.diag([
            self.alpha1 * abs(self.wr) + self.alpha2 * abs(self.wr) + 1e-3,
            self.alpha1 * abs(self.wl) + self.alpha2 * abs(self.wl) + 1e-3
        ])

        # Actualización de la covarianza
        self.P = J_h @ self.P @ J_h.T + J_u @ Q_control @ J_u.T
        
        # Asegurar que P permanezca positiva definida y simétrica
        self.P = (self.P + self.P.T) / 2  # Hacer simétrica
        
        # Añadir un pequeño valor a la diagonal para asegurar definición positiva
        min_eig = np.min(np.real(np.linalg.eigvals(self.P)))
        if min_eig < 1e-6:
            self.P += np.eye(3) * (1e-6 - min_eig)
        
        # Limitar la covarianza máxima para evitar valores numéricamente inestables
        max_cov = 10.0  # Valor máximo para cualquier elemento de la diagonal
        np.fill_diagonal(self.P, np.minimum(np.diag(self.P), max_cov))
        
        self.get_logger().info(f"Covarianza diagonal: {self.P.diagonal()}", throttle_duration_sec=1)

    def publish_odometry(self, v, w):
        odom = Odometry()
        now = self.get_clock().now().to_msg()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]

        # Pose covariance 6x6 (solo x, y, theta)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.P[0, 0]   # x-x
        odom.pose.covariance[1] = self.P[0, 1]   # x-y
        odom.pose.covariance[5] = self.P[0, 2]   # x-theta
        odom.pose.covariance[6] = self.P[1, 0]   # y-x
        odom.pose.covariance[7] = self.P[1, 1]   # y-y
        odom.pose.covariance[11] = self.P[1, 2]  # y-theta
        odom.pose.covariance[30] = self.P[2, 0]  # theta-x
        odom.pose.covariance[31] = self.P[2, 1]  # theta-y
        odom.pose.covariance[35] = self.P[2, 2]  # theta-theta

        # Velocidades
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        # Covarianza del twist (con valores basados en el ruido)
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = self.alpha1 * abs(v) + 1e-3
        odom.twist.covariance[35] = self.alpha2 * abs(w) + 1e-3

        self.odom_pub.publish(odom)

    def publish_tf(self):
        t = TransformStamped()
        now = self.get_clock().now().to_msg()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = transforms3d.euler.euler2quat(0, 0, self.theta)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        self.tf_broadcaster.sendTransform(t)

    def publish_wheels(self):
        self.wr_pub.publish(Float32(data=self.wr))
        self.wl_pub.publish(Float32(data=self.wl))

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()