"""
Automatic extrinsic calibration of 2D LiDAR (RPLidar S2) and camera (RealSense D455).

Algorithm:
    1. Collects synchronized LaserScan + Depth Image pairs
    2. Converts LaserScan to 3D points in lidar_link frame (z=0 plane)
    3. Transforms lidar points to camera_depth_optical_frame using candidate extrinsic
    4. Projects lidar points onto depth image using camera intrinsics
    5. Compares projected depth vs measured depth from the camera
    6. Optimizes extrinsic (tx, ty, tz, yaw) to minimize depth disagreement

Since the lidar is 2D (single horizontal plane), we optimize 4 DOF:
    - tx, ty, tz: translation from base (lidar) to camera_link
    - yaw (rz): rotation around Z axis

The remaining 2 DOF (roll, pitch) are assumed zero for a rigid platform.
"""

import math
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class LidarCameraCalibNode(Node):
    """ROS2 node for automatic 2D LiDAR to camera extrinsic calibration."""

    def __init__(self):
        super().__init__("lidar_camera_calib_node")

        # --- Parameters ---
        self.declare_parameter("lidar_scan_topic", "/lidar/scan")
        self.declare_parameter("depth_image_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")

        # Initial guess for extrinsic (lidar_link -> camera_link)
        # From model.yaml: camera_link is at (0.25, 0.05, -0.04) relative to base/lidar
        self.declare_parameter("init_tx", 0.25)
        self.declare_parameter("init_ty", 0.05)
        self.declare_parameter("init_tz", -0.04)
        self.declare_parameter("init_yaw", 0.0)  # radians

        # Calibration parameters
        self.declare_parameter("num_samples", 30)
        self.declare_parameter("depth_range_min", 0.3)  # meters, ignore closer
        self.declare_parameter("depth_range_max", 6.0)  # meters, ignore farther
        self.declare_parameter("depth_scale", 0.001)  # RealSense depth scale (mm -> m)
        self.declare_parameter("max_depth_error", 0.3)  # meters, outlier rejection

        # Read parameters
        self.lidar_topic = self.get_parameter("lidar_scan_topic").value
        self.depth_topic = self.get_parameter("depth_image_topic").value
        self.info_topic = self.get_parameter("camera_info_topic").value
        self.init_tx = self.get_parameter("init_tx").value
        self.init_ty = self.get_parameter("init_ty").value
        self.init_tz = self.get_parameter("init_tz").value
        self.init_yaw = self.get_parameter("init_yaw").value
        self.num_samples = self.get_parameter("num_samples").value
        self.depth_range_min = self.get_parameter("depth_range_min").value
        self.depth_range_max = self.get_parameter("depth_range_max").value
        self.depth_scale = self.get_parameter("depth_scale").value
        self.max_depth_error = self.get_parameter("max_depth_error").value

        # --- State ---
        self.bridge = CvBridge()
        self.camera_intrinsics: Optional[np.ndarray] = None  # 3x3
        self.img_width = 0
        self.img_height = 0
        self.collected_pairs: list = []
        self.lock = threading.Lock()
        self.calibration_done = False

        # Latest data for approximate synchronization
        self.latest_scan: Optional[LaserScan] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_scan_stamp = None
        self.latest_depth_stamp = None

        # --- TF chain: lidar_link -> base -> camera_link -> camera_depth_frame
        #     -> camera_depth_optical_frame
        # lidar_link = base (translation 0,0,0, identity rotation)
        # camera_link relative to base: (tx, ty, tz) + yaw rotation
        # camera_depth_frame = camera_link (identity)
        # camera_depth_optical_frame relative to camera_depth_frame:
        #   rotation quaternion (-0.5, 0.5, -0.5, 0.5) from model.yaml
        # This is a 90° rotation: x_optical = -y_cam, y_optical = -z_cam, z_optical = x_cam
        self.R_depth_to_optical = Rotation.from_quat([-0.5, 0.5, -0.5, 0.5]).as_matrix()

        # --- QoS for sensor data ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- Subscribers ---
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.camera_info_cb, sensor_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.lidar_topic, self.scan_cb, sensor_qos
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_cb, sensor_qos
        )

        # Timer for approximate sync + collection
        self.sync_timer = self.create_timer(0.2, self.sync_and_collect)

        self.get_logger().info(
            f"Lidar-Camera Calibration Node started.\n"
            f"  Lidar topic:  {self.lidar_topic}\n"
            f"  Depth topic:  {self.depth_topic}\n"
            f"  Info topic:   {self.info_topic}\n"
            f"  Initial guess: tx={self.init_tx}, ty={self.init_ty}, "
            f"tz={self.init_tz}, yaw={self.init_yaw}\n"
            f"  Collecting {self.num_samples} samples..."
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def camera_info_cb(self, msg: CameraInfo):
        """Extract camera intrinsic matrix from CameraInfo."""
        if self.camera_intrinsics is not None:
            return
        K = np.array(msg.k).reshape(3, 3)
        if K[0, 0] == 0.0:
            return
        self.camera_intrinsics = K
        self.img_width = msg.width
        self.img_height = msg.height
        self.get_logger().info(
            f"Camera intrinsics received: {self.img_width}x{self.img_height}\n"
            f"  fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}"
        )

    def scan_cb(self, msg: LaserScan):
        """Store latest LaserScan."""
        with self.lock:
            self.latest_scan = msg
            self.latest_scan_stamp = msg.header.stamp

    def depth_cb(self, msg: Image):
        """Store latest depth image."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.lock:
                self.latest_depth = depth_img
                self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    # -------------------------------------------------------------------------
    # Approximate sync + data collection
    # -------------------------------------------------------------------------

    def sync_and_collect(self):
        """Approximately synchronize scan and depth, collect pairs."""
        if self.calibration_done:
            return
        if self.camera_intrinsics is None:
            return

        with self.lock:
            scan = self.latest_scan
            depth = self.latest_depth
            scan_stamp = self.latest_scan_stamp
            depth_stamp = self.latest_depth_stamp

        if scan is None or depth is None:
            return
        if scan_stamp is None or depth_stamp is None:
            return

        # Check approximate time sync (within 200ms)
        dt = abs(
            (scan_stamp.sec + scan_stamp.nanosec * 1e-9)
            - (depth_stamp.sec + depth_stamp.nanosec * 1e-9)
        )
        if dt > 0.2:
            return

        # Convert LaserScan to 3D points in lidar_link frame
        lidar_points = self.scan_to_points(scan)
        if lidar_points.shape[0] < 50:
            return

        self.collected_pairs.append((lidar_points, depth.copy()))

        n = len(self.collected_pairs)
        self.get_logger().info(f"Collected sample {n}/{self.num_samples}")

        # Clear latest to avoid re-using
        with self.lock:
            self.latest_scan = None
            self.latest_depth = None

        if n >= self.num_samples:
            self.calibration_done = True
            self.get_logger().info("All samples collected. Starting calibration...")
            self.run_calibration()

    # -------------------------------------------------------------------------
    # LaserScan -> 3D points
    # -------------------------------------------------------------------------

    def scan_to_points(self, scan: LaserScan) -> np.ndarray:
        """
        Convert LaserScan to Nx3 array of 3D points in lidar_link frame.
        Points lie in the z=0 plane (2D lidar).
        """
        angles = np.arange(
            scan.angle_min,
            scan.angle_min + len(scan.ranges) * scan.angle_increment,
            scan.angle_increment,
        )
        if len(angles) > len(scan.ranges):
            angles = angles[: len(scan.ranges)]

        ranges = np.array(scan.ranges, dtype=np.float64)

        # Filter invalid ranges
        valid = np.isfinite(ranges) & (ranges >= self.depth_range_min) & (ranges <= self.depth_range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert polar to cartesian (lidar_link frame, z=0)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        return np.column_stack([x, y, z])

    # -------------------------------------------------------------------------
    # Transform and project
    # -------------------------------------------------------------------------

    def build_extrinsic(self, tx, ty, tz, yaw) -> np.ndarray:
        """
        Build 4x4 transform from lidar_link to camera_depth_optical_frame.

        Chain: lidar_link -> base (identity) -> camera_link (tx, ty, tz, yaw)
               -> camera_depth_frame (identity) -> camera_depth_optical_frame (fixed rotation)
        """
        # Rotation: yaw around Z-axis (lidar_link -> camera_link)
        R_yaw = Rotation.from_euler("z", yaw).as_matrix()

        # lidar_link -> camera_link
        T_lidar_to_cam = np.eye(4)
        T_lidar_to_cam[:3, :3] = R_yaw
        T_lidar_to_cam[:3, 3] = [tx, ty, tz]

        # camera_link -> camera_depth_optical_frame
        T_cam_to_optical = np.eye(4)
        T_cam_to_optical[:3, :3] = self.R_depth_to_optical

        # Full chain: lidar_link -> camera_depth_optical_frame
        # Points in lidar are first transformed to camera_link frame,
        # then to optical frame
        # P_optical = T_cam_to_optical * inv(T_lidar_to_cam) * P_lidar
        # Because T_lidar_to_cam transforms FROM lidar TO camera,
        # but we need: where is the lidar point in camera frame?
        # P_cam = R_yaw_inv * (P_lidar - t)  -- NO
        # Actually: if camera_link is at (tx,ty,tz,yaw) relative to base/lidar,
        # then a point P in lidar frame transforms to camera_link frame as:
        # P_cam = R_yaw^T * (P_lidar - [tx, ty, tz])
        # Then to optical frame: P_optical = R_depth_to_optical * P_cam

        T_lidar_to_cam_inv = np.eye(4)
        T_lidar_to_cam_inv[:3, :3] = R_yaw.T
        T_lidar_to_cam_inv[:3, 3] = -R_yaw.T @ np.array([tx, ty, tz])

        return T_cam_to_optical @ T_lidar_to_cam_inv

    def project_points(
        self, points_lidar: np.ndarray, extrinsic_4x4: np.ndarray
    ) -> tuple:
        """
        Transform lidar points to optical frame, then project onto image plane.

        Returns:
            pixels: Nx2 array of (u, v) pixel coordinates
            depths: N array of depth values in optical frame (z-component)
            valid_mask: N boolean array (True if point projects within image)
        """
        K = self.camera_intrinsics
        N = points_lidar.shape[0]

        # Transform to optical frame
        pts_h = np.hstack([points_lidar, np.ones((N, 1))])  # Nx4
        pts_optical = (extrinsic_4x4 @ pts_h.T).T[:, :3]  # Nx3

        # Depth in optical frame is z-component
        depths = pts_optical[:, 2]

        # Project: pixel = K * [x/z, y/z, 1]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = K[0, 0] * pts_optical[:, 0] / depths + K[0, 2]
            v = K[1, 1] * pts_optical[:, 1] / depths + K[1, 2]

        pixels = np.column_stack([u, v])

        # Valid: in front of camera and within image bounds
        valid = (
            (depths > 0.1)
            & np.isfinite(u)
            & np.isfinite(v)
            & (u >= 0)
            & (u < self.img_width)
            & (v >= 0)
            & (v < self.img_height)
        )

        return pixels, depths, valid

    # -------------------------------------------------------------------------
    # Cost function
    # -------------------------------------------------------------------------

    def compute_cost(self, params: np.ndarray) -> float:
        """
        Compute mean absolute depth error across all collected samples.

        params: [tx, ty, tz, yaw]
        """
        tx, ty, tz, yaw = params
        T = self.build_extrinsic(tx, ty, tz, yaw)

        total_error = 0.0
        total_count = 0

        for lidar_points, depth_img in self.collected_pairs:
            pixels, lidar_depths, valid = self.project_points(lidar_points, T)

            if np.sum(valid) == 0:
                continue

            u = pixels[valid, 0].astype(int)
            v = pixels[valid, 1].astype(int)
            z_lidar = lidar_depths[valid]

            # Get camera depth at projected pixel locations
            z_camera = depth_img[v, u].astype(np.float64) * self.depth_scale

            # Filter: both depths must be valid
            depth_valid = (z_camera > self.depth_range_min) & (z_camera < self.depth_range_max)
            if np.sum(depth_valid) == 0:
                continue

            z_lidar_f = z_lidar[depth_valid]
            z_camera_f = z_camera[depth_valid]

            # Compute error with outlier rejection
            errors = np.abs(z_lidar_f - z_camera_f)
            inlier_mask = errors < self.max_depth_error
            if np.sum(inlier_mask) == 0:
                continue

            # Use Huber-like loss: L1 for robustness
            total_error += np.sum(errors[inlier_mask])
            total_count += np.sum(inlier_mask)

        if total_count == 0:
            return 1e6

        return total_error / total_count

    # -------------------------------------------------------------------------
    # Optimization
    # -------------------------------------------------------------------------

    def run_calibration(self):
        """Run the optimization to find best extrinsic parameters."""
        x0 = np.array([self.init_tx, self.init_ty, self.init_tz, self.init_yaw])

        initial_cost = self.compute_cost(x0)
        self.get_logger().info(
            f"Initial cost (mean depth error): {initial_cost:.4f} m\n"
            f"Initial params: tx={x0[0]:.4f}, ty={x0[1]:.4f}, "
            f"tz={x0[2]:.4f}, yaw={x0[3]:.4f}"
        )

        # Bounds: reasonable range around initial guess
        bounds = [
            (self.init_tx - 0.15, self.init_tx + 0.15),  # tx ± 15cm
            (self.init_ty - 0.15, self.init_ty + 0.15),  # ty ± 15cm
            (self.init_tz - 0.10, self.init_tz + 0.10),  # tz ± 10cm
            (self.init_yaw - 0.35, self.init_yaw + 0.35),  # yaw ± 20°
        ]

        self.get_logger().info("Running optimization (Nelder-Mead)...")

        result = minimize(
            self.compute_cost,
            x0,
            method="Nelder-Mead",
            options={
                "maxiter": 2000,
                "xatol": 1e-4,
                "fatol": 1e-5,
                "adaptive": True,
            },
        )

        # Clip to bounds manually (Nelder-Mead doesn't enforce bounds)
        opt = np.clip(result.x, [b[0] for b in bounds], [b[1] for b in bounds])

        # Also try L-BFGS-B with bounds for comparison
        self.get_logger().info("Running optimization (L-BFGS-B)...")
        result_bfgs = minimize(
            self.compute_cost,
            x0,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 500, "ftol": 1e-7},
        )

        # Pick best result
        cost_nm = self.compute_cost(opt)
        cost_bfgs = self.compute_cost(result_bfgs.x)

        if cost_bfgs < cost_nm:
            best = result_bfgs.x
            best_cost = cost_bfgs
            best_method = "L-BFGS-B"
        else:
            best = opt
            best_cost = cost_nm
            best_method = "Nelder-Mead"

        # Convert yaw to quaternion for model.yaml
        quat = Rotation.from_euler("z", best[3]).as_quat()  # [x, y, z, w]

        # Print results
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  CALIBRATION RESULT (best: {best_method})\n"
            f"{'='*60}\n"
            f"  Initial cost:  {initial_cost:.4f} m\n"
            f"  Final cost:    {best_cost:.4f} m\n"
            f"  Improvement:   {initial_cost - best_cost:.4f} m\n"
            f"\n"
            f"  Translation:\n"
            f"    tx = {best[0]:.5f} m\n"
            f"    ty = {best[1]:.5f} m\n"
            f"    tz = {best[2]:.5f} m\n"
            f"\n"
            f"  Rotation:\n"
            f"    yaw = {best[3]:.5f} rad ({math.degrees(best[3]):.2f} deg)\n"
            f"\n"
            f"  Quaternion (x, y, z, w):\n"
            f"    x = {quat[0]:.6f}\n"
            f"    y = {quat[1]:.6f}\n"
            f"    z = {quat[2]:.6f}\n"
            f"    w = {quat[3]:.6f}\n"
            f"\n"
            f"{'='*60}\n"
            f"  Copy this to model.yaml (tf_static, base -> camera_link):\n"
            f"{'='*60}\n"
            f"\n"
            f'  - frame_id: "base"\n'
            f'    child_frame_id: "camera_link"\n'
            f"    translation: {{ x: {best[0]:.5f}, y: {best[1]:.5f}, z: {best[2]:.5f} }}\n"
            f"    rotation: {{ x: {quat[0]:.6f}, y: {quat[1]:.6f}, "
            f"z: {quat[2]:.6f}, w: {quat[3]:.6f} }}\n"
            f"\n{'='*60}"
        )

        self.get_logger().info("Calibration complete. You can now Ctrl+C.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
