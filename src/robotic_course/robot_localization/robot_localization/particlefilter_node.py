import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from scipy.spatial import cKDTree

class Particle:
    def __init__(self, x, y, theta, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particlefilter_node')
        self.declare_parameter('num_particles', 500)
        self.declare_parameter('alpha1', 0.05)
        self.declare_parameter('alpha2', 0.05)
        self.declare_parameter('alpha3', 0.02)
        self.declare_parameter('alpha4', 0.02)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('laser_subsample', 3)
        self.declare_parameter('resample_threshold', 0.2)
        self.declare_parameter('likelihood_sigma_m', 0.15)
        self.declare_parameter('z_hit', 0.95)
        self.declare_parameter('z_rand', 0.05)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')

        self.num_particles = int(self.get_parameter('num_particles').value)
        self.alpha1 = float(self.get_parameter('alpha1').value)
        self.alpha2 = float(self.get_parameter('alpha2').value)
        self.alpha3 = float(self.get_parameter('alpha3').value)
        self.alpha4 = float(self.get_parameter('alpha4').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.laser_subsample = int(self.get_parameter('laser_subsample').value)
        self.resample_threshold = float(self.get_parameter('resample_threshold').value)
        self.likelihood_sigma_m = float(self.get_parameter('likelihood_sigma_m').value)
        self.z_hit = float(self.get_parameter('z_hit').value)
        self.z_rand = float(self.get_parameter('z_rand').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)

        self.particles = []
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.obstacle_kdtree = None

        self.last_odom = None
        self.initialized = False
        self.free_cells_cache = []

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/ekf/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.initpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.particle_cloud_pub = self.create_publisher(PoseArray, '/particlecloud', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quat_from_yaw(self, yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        return mx, my

    def is_valid_position(self, x, y):
        mx, my = self.world_to_map(x, y)
        if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
            return False
        return self.map_data[my, mx] == 0

    def build_free_cells_cache(self):
        self.free_cells_cache = []
        for my in range(self.map_height):
            for mx in range(self.map_width):
                if self.map_data[my, mx] == 0:
                    wx = mx * self.map_resolution + self.map_origin[0] + self.map_resolution / 2.0
                    wy = my * self.map_resolution + self.map_origin[1] + self.map_resolution / 2.0
                    self.free_cells_cache.append((wx, wy))

    def create_obstacle_kdtree(self):
        obs = np.argwhere((self.map_data >= 50) | (self.map_data < 0))
        if obs.size == 0:
            self.obstacle_kdtree = None
            return
        cells = np.stack([obs[:, 1], obs[:, 0]], axis=1).astype(np.float32)
        self.obstacle_kdtree = cKDTree(cells)

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.map_resolution = float(msg.info.resolution)
        self.map_origin = (float(msg.info.origin.position.x), float(msg.info.origin.position.y))
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)

        self.create_obstacle_kdtree()
        self.build_free_cells_cache()

        if not self.initialized and self.free_cells_cache:
            self.initialize_particles_uniform()
            self.initialized = True

        self.publish_identity_map_tf()

    def publish_identity_map_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def initialize_particles_uniform(self):
        self.particles = []
        for _ in range(self.num_particles):
            idx = np.random.randint(0, len(self.free_cells_cache))
            x, y = self.free_cells_cache[idx]
            th = np.random.uniform(-math.pi, math.pi)
            self.particles.append(Particle(x, y, th, 1.0 / self.num_particles))
        self.publish_particle_cloud()

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        if self.map_data is None or not self.free_cells_cache:
            return
        x0 = float(msg.pose.pose.position.x)
        y0 = float(msg.pose.pose.position.y)
        th0 = float(self.yaw_from_quat(msg.pose.pose.orientation))
        cov = msg.pose.covariance
        std_x = math.sqrt(max(cov[0], 1e-6))
        std_y = math.sqrt(max(cov[7], 1e-6))
        std_th = math.sqrt(max(cov[35], 1e-6))
        std_xy = float(np.clip(max(std_x, std_y), 0.05, 1.5))
        std_th = float(np.clip(std_th, 0.1, 2.0))
        self.initialize_particles_gaussian(x0, y0, th0, std_xy, std_th)
        self.initialized = True
        self.publish_particle_cloud()
        self.publish_pose()

    def initialize_particles_gaussian(self, x0, y0, th0, std_xy, std_th):
        new_particles = []
        attempts = 0
        max_attempts = self.num_particles * 60
        while len(new_particles) < self.num_particles and attempts < max_attempts:
            attempts += 1
            x = float(np.random.normal(x0, std_xy))
            y = float(np.random.normal(y0, std_xy))
            th = float(self.normalize_angle(np.random.normal(th0, std_th)))
            if self.is_valid_position(x, y):
                new_particles.append(Particle(x, y, th, 1.0 / self.num_particles))
        while len(new_particles) < self.num_particles and self.free_cells_cache:
            idx = np.random.randint(0, len(self.free_cells_cache))
            x, y = self.free_cells_cache[idx]
            th = np.random.uniform(-math.pi, math.pi)
            new_particles.append(Particle(x, y, th, 1.0 / self.num_particles))
        if new_particles:
            self.particles = new_particles

    def extract_odom(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        theta = float(self.yaw_from_quat(q))
        return {'x': x, 'y': y, 'theta': theta}

    def compute_odom_delta(self, old_odom, new_odom):
        dx = new_odom['x'] - old_odom['x']
        dy = new_odom['y'] - old_odom['y']
        dtheta = self.normalize_angle(new_odom['theta'] - old_odom['theta'])
        trans = math.hypot(dx, dy)
        rot1 = 0.0
        if trans > 1e-9:
            rot1 = self.normalize_angle(math.atan2(dy, dx) - old_odom['theta'])
        rot2 = self.normalize_angle(dtheta - rot1)
        return {'trans': trans, 'rot1': rot1, 'rot2': rot2}

    def motion_update(self, delta):
        trans = float(delta['trans'])
        rot1 = float(delta['rot1'])
        rot2 = float(delta['rot2'])
        for p in self.particles:
            trans_noise = self.alpha3 * abs(trans) + self.alpha4 * (abs(rot1) + abs(rot2))
            rot1_noise = self.alpha1 * abs(rot1) + self.alpha2 * abs(trans)
            rot2_noise = self.alpha1 * abs(rot2) + self.alpha2 * abs(trans)
            trans_noisy = trans + np.random.normal(0.0, max(trans_noise, 1e-4))
            rot1_noisy = rot1 + np.random.normal(0.0, max(rot1_noise, 1e-4))
            rot2_noisy = rot2 + np.random.normal(0.0, max(rot2_noise, 1e-4))
            p.x += trans_noisy * math.cos(p.theta + rot1_noisy)
            p.y += trans_noisy * math.sin(p.theta + rot1_noisy)
            p.theta = self.normalize_angle(p.theta + rot1_noisy + rot2_noisy)

    def odom_callback(self, msg):
        if not self.initialized:
            return
        curr = self.extract_odom(msg)
        if self.last_odom is None:
            self.last_odom = curr
            return
        delta = self.compute_odom_delta(self.last_odom, curr)
        if abs(delta['trans']) > 1e-4 or abs(delta['rot1']) > 1e-4 or abs(delta['rot2']) > 1e-4:
            self.motion_update(delta)
        self.last_odom = curr
        self.publish_pose()
        self.publish_particle_cloud()

    def measurement_update(self, scan_msg):
        if self.obstacle_kdtree is None:
            for p in self.particles:
                p.weight *= 0.001
            return

        angle_min = float(scan_msg.angle_min)
        angle_inc = float(scan_msg.angle_increment)
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)

        sigma_cells = max(self.likelihood_sigma_m / self.map_resolution, 1e-3)
        inv_2sigma2 = 1.0 / (2.0 * sigma_cells * sigma_cells)

        for p in self.particles:
            log_w = 0.0
            cnt = 0
            for i in range(0, ranges.shape[0], self.laser_subsample):
                z = float(ranges[i])
                if (not math.isfinite(z)) or z < self.min_range or z > self.max_range:
                    continue
                cnt += 1
                a = p.theta + (angle_min + i * angle_inc)
                hx = p.x + z * math.cos(a)
                hy = p.y + z * math.sin(a)
                mx, my = self.world_to_map(hx, hy)
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    dist_cells, _ = self.obstacle_kdtree.query([mx, my], k=1)
                    p_hit = math.exp(-float(dist_cells * dist_cells) * inv_2sigma2)
                    prob = self.z_hit * p_hit + (self.z_rand / self.max_range)
                else:
                    prob = 1e-6
                log_w += math.log(max(prob, 1e-12))
            if cnt > 0:
                p.weight *= math.exp(log_w / cnt)
            else:
                p.weight *= 1e-6

    def normalize_weights(self):
        s = sum(p.weight for p in self.particles)
        if s <= 0.0:
            w = 1.0 / float(self.num_particles)
            for p in self.particles:
                p.weight = w
            return
        inv = 1.0 / s
        for p in self.particles:
            p.weight *= inv

    def effective_sample_size(self):
        ss = sum(p.weight * p.weight for p in self.particles)
        if ss <= 0.0:
            return 0.0
        return 1.0 / ss

    def resample(self):
        weights = np.array([p.weight for p in self.particles], dtype=np.float64)
        cdf = np.cumsum(weights)
        if cdf[-1] <= 0:
            return
        step = 1.0 / self.num_particles
        r = np.random.uniform(0.0, step)
        i = 0
        new_particles = []
        for m in range(self.num_particles):
            u = r + m * step
            while i < self.num_particles - 1 and u > cdf[i]:
                i += 1
            old = self.particles[i]
            x = old.x + np.random.normal(0.0, 0.01)
            y = old.y + np.random.normal(0.0, 0.01)
            th = self.normalize_angle(old.theta + np.random.normal(0.0, 0.005))
            new_particles.append(Particle(x, y, th, 1.0 / self.num_particles))
        self.particles = new_particles

    def scan_callback(self, msg):
        if (not self.initialized) or (self.map_data is None):
            return
        self.measurement_update(msg)
        self.normalize_weights()
        neff = self.effective_sample_size()
        if neff < self.resample_threshold * self.num_particles:
            self.resample()
        self.publish_pose()
        self.publish_particle_cloud()

    def estimate_pose(self):
        if not self.particles:
            return None
        wsum = sum(p.weight for p in self.particles)
        if wsum <= 0:
            return None
        x = sum(p.x * p.weight for p in self.particles) / wsum
        y = sum(p.y * p.weight for p in self.particles) / wsum
        s = sum(math.sin(p.theta) * p.weight for p in self.particles) / wsum
        c = sum(math.cos(p.theta) * p.weight for p in self.particles) / wsum
        th = math.atan2(s, c)
        return {'x': x, 'y': y, 'theta': th}

    def publish_map_to_odom_tf(self, pose_map, odom_pose):
        mx, my, mth = pose_map['x'], pose_map['y'], pose_map['theta']
        ox, oy, oth = odom_pose['x'], odom_pose['y'], odom_pose['theta']

        cos_o = math.cos(oth)
        sin_o = math.sin(oth)

        inv_ox = -ox * cos_o - oy * sin_o
        inv_oy = ox * sin_o - oy * cos_o
        inv_oth = -oth

        th = self.normalize_angle(mth + inv_oth)

        tx = mx + (inv_ox * math.cos(mth) - inv_oy * math.sin(mth))
        ty = my + (inv_ox * math.sin(mth) + inv_oy * math.cos(mth))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = self.quat_from_yaw(th)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self):
        est = self.estimate_pose()
        if est is None:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(est['x'])
        msg.pose.pose.position.y = float(est['y'])
        qx, qy, qz, qw = self.quat_from_yaw(est['theta'])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1
        self.pose_pub.publish(msg)

        if self.last_odom is not None:
            self.publish_map_to_odom_tf(est, self.last_odom)
        else:
            self.publish_identity_map_tf()

    def publish_particle_cloud(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.poses = []
        for p in self.particles:
            pose = Pose()
            pose.position.x = float(p.x)
            pose.position.y = float(p.y)
            qx, qy, qz, qw = self.quat_from_yaw(p.theta)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            msg.poses.append(pose)
        self.particle_cloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
