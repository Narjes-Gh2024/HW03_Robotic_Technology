import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class EkfDiffImu(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        self.wheel_odom_topic_ = self.declare_parameter('wheel_odom_topic', '/wheel_encoder/odom').value
        self.odom_topic_ = self.declare_parameter('odom_topic', '/ekf/odom').value
        self.imu_topic_ = self.declare_parameter('imu_topic', '/zed/zed_node/imu/data_raw').value
        
        self.sigma_v_ = self.declare_parameter('sigma_v', 0.10).value
        self.sigma_omega_ = self.declare_parameter('sigma_omega', 1e-8).value
        self.sigma_omega_ = math.sqrt(self.sigma_omega_)
        
        self.x_ = np.zeros(3)
        self.P_ = np.eye(3) * 0.1
        
        self.last_predict_time_ = None
        self.last_v_ = 0.0
        self.last_omega_ = 0.0
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.wheel_odom_sub_ = self.create_subscription(
            Odometry,
            self.wheel_odom_topic_,
            self.wheel_odom_cb,
            sensor_qos
        )
        
        self.imu_sub_ = self.create_subscription(
            Imu,
            self.imu_topic_,
            self.imu_cb,
            sensor_qos
        )
        
        self.odom_pub_ = self.create_publisher(Odometry, self.odom_topic_, 10)
        
        self.tf_broadcaster_ = TransformBroadcaster(self)
    
    @staticmethod
    def wrap_to_pi(a):
        while a <= -math.pi:
            a += 2.0 * math.pi
        while a > math.pi:
            a -= 2.0 * math.pi
        return a
    
    def predict(self, v, omega, dt):
        th = self.x_[2]
        c = math.cos(th)
        s = math.sin(th)
        
        self.x_[0] += v * c * dt
        self.x_[1] += v * s * dt
        self.x_[2] = self.wrap_to_pi(th + omega * dt)
        
        F = np.eye(3)
        F[0, 2] = -v * s * dt
        F[1, 2] = v * c * dt
        
        G = np.array([
            [c * dt, 0.0],
            [s * dt, 0.0],
            [0.0, dt]
        ])
        
        Qu = np.array([
            [self.sigma_v_ ** 2, 0.0],
            [0.0, self.sigma_omega_ ** 2]
        ])
        
        Q = G @ Qu @ G.T
        self.P_ = F @ self.P_ @ F.T + Q
    
    def update_yaw(self, yaw_meas, var_yaw):
        H = np.array([[0.0, 0.0, 1.0]])
        
        z_pred = self.x_[2]
        y = self.wrap_to_pi(yaw_meas - z_pred)
        
        S = (H @ self.P_ @ H.T)[0, 0] + var_yaw
        K = (self.P_ @ H.T) / S
        
        self.x_ = self.x_ + (K @ np.array([y])).flatten()
        self.x_[2] = self.wrap_to_pi(self.x_[2])
        
        I = np.eye(3)
        self.P_ = (I - K @ H) @ self.P_
    
    def publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x_[0]
        odom.pose.pose.position.y = self.x_[1]
        odom.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        yaw = self.x_[2]
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.P_[0, 0]
        odom.pose.covariance[7] = self.P_[1, 1]
        odom.pose.covariance[35] = self.P_[2, 2]
        
        odom.twist.twist.linear.x = self.last_v_
        odom.twist.twist.angular.z = self.last_omega_
        
        self.odom_pub_.publish(odom)
        
        # TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x_[0]
        tf_msg.transform.translation.y = self.x_[1]
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster_.sendTransform(tf_msg)
    
    def wheel_odom_cb(self, msg):
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        t = msg.header.stamp
        
        if self.last_predict_time_ is None:
            self.last_predict_time_ = t
        
        dt = (t.sec - self.last_predict_time_.sec) + \
             (t.nanosec - self.last_predict_time_.nanosec) * 1e-9
        
        if dt <= 1e-4:
            dt = 1e-4
        
        self.predict(v, omega, dt)
        self.last_v_ = v
        self.last_omega_ = omega
        
        self.publish_odom(t)
        self.last_predict_time_ = t
    
    def imu_cb(self, msg):
        q = msg.orientation
        
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        var_yaw = 1e-8
        if msg.orientation_covariance[8] >= 0.0:
            var_yaw = max(msg.orientation_covariance[8], 1e-8)
        
        self.update_yaw(yaw, var_yaw)
        self.publish_odom(msg.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = EkfDiffImu()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
