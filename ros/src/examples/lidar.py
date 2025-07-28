#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import rospy
import numpy as np
import math
from fs_msgs.msg import ControlCommand, Cone
from sensor_msgs.msg import PointCloud2, NavSatFix, Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Empty
from sklearn.cluster import DBSCAN, KMeans
from tf.transformations import euler_from_quaternion

# ==============================================================================
# ======================== 기본 파라미터 (Basic Parameters) ======================
# ==============================================================================
CONTROL_RATE = 50.0
TARGET_SPEED = 5.0
KP_SPEED = 0.5
MAX_STEER = 0.5
MAX_THROTTLE = 0.4
VEHICLE_WHEELBASE = 1.55
LOOKAHEAD_DISTANCE = 3.5
LIDAR_RANGE_CUTOFF = 15.0
# ==============================================================================

class LidarSubplotMapper:
    def __init__(self):
        rospy.init_node('lidar_subplot_mapper_node')
        # --- 상태 변수 ---
        self.current_speed = 0.0; self.detected_cones_local = []; self.last_valid_steering = 0.0
        self.prev_gps_pos = None; self.last_gps_time = None

        # --- 글로벌 맵핑용 변수 ---
        self.global_pose = np.zeros(3); self.gps_origin = None
        self.global_blue_cones = []; self.global_yellow_cones = []
        
        # --- ROS 통신 설정 ---
        self.control_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        rospy.Subscriber('/fsds/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/fsds/lidar/Lidar1', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/fsds/imu', Imu, self.imu_callback)
        rospy.loginfo("Lidar Subplot Mapper 노드 시작됨.")
        
        self.start_driving()
        
        # --- 시각화 설정: 1x2 서브플롯 생성 ---
        self.fig, (self.ax_local, self.ax_global) = plt.subplots(1, 2, figsize=(15, 8))
        self.setup_local_plot()
        self.setup_global_plot()

    def gps_callback(self, msg: NavSatFix):
        current_time = rospy.Time.now(); raw_speed = 0.0
        if self.prev_gps_pos:
            dt = (current_time - self.last_gps_time).to_sec()
            if dt > 0.01:
                dx = (msg.latitude - self.prev_gps_pos.latitude) * 111132.0
                dy = (msg.longitude - self.prev_gps_pos.longitude) * 111132.0 * math.cos(math.radians(msg.latitude))
                raw_speed = math.sqrt(dx**2 + dy**2) / dt
        self.current_speed = raw_speed
        self.prev_gps_pos, self.last_gps_time = msg, current_time

        if self.gps_origin is None: self.gps_origin = msg
        self.global_pose[0] = (msg.longitude - self.gps_origin.longitude) * 111132.0 * math.cos(math.radians(self.gps_origin.latitude))
        self.global_pose[1] = (msg.latitude - self.gps_origin.latitude) * 111132.0

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.global_pose[2] = yaw

    def lidar_callback(self, msg: PointCloud2):
        raw_points = [p[:2] for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True) if np.hypot(p[0],p[1]) < LIDAR_RANGE_CUTOFF]
        if not raw_points: self.detected_cones_local = []; return
        db = DBSCAN(eps=0.5, min_samples=3).fit(raw_points)
        self.detected_cones_local = [np.mean(np.array(raw_points)[db.labels_ == l], axis=0) for l in set(db.labels_) if l != -1]

    def transform_local_to_global(self, local_cones):
        gx, gy, gyaw = self.global_pose; cos_yaw, sin_yaw = math.cos(gyaw), math.sin(gyaw)
        return [[gx + lx*cos_yaw - ly*sin_yaw, gy + lx*sin_yaw + ly*cos_yaw] for lx, ly in local_cones]

    def control_logic(self):
        path_cones = self.detected_cones_local
        blue_path_cones, yellow_path_cones, path_waypoints = [], [], []
        path_found = False

        if len(path_cones) >= 4:
            kmeans = KMeans(n_clusters=2, n_init='auto', random_state=0).fit(path_cones)
            left_label = np.argmax([center[1] for center in kmeans.cluster_centers_])
            blue_path_cones = np.array([c for i, c in enumerate(path_cones) if kmeans.labels_[i] == left_label])
            yellow_path_cones = np.array([c for i, c in enumerate(path_cones) if kmeans.labels_[i] != left_label])
            self.global_blue_cones.extend(self.transform_local_to_global(blue_path_cones))
            self.global_yellow_cones.extend(self.transform_local_to_global(yellow_path_cones))

        if len(blue_path_cones) >= 2 and len(yellow_path_cones) >= 2:
            blue_coeffs = np.polyfit(blue_path_cones[:, 0], blue_path_cones[:, 1], 1)
            yellow_coeffs = np.polyfit(yellow_path_cones[:, 0], yellow_path_cones[:, 1], 1)
            center_coeffs = (blue_coeffs + yellow_coeffs) / 2.0
            x_points = np.arange(0.5, LIDAR_RANGE_CUTOFF, 0.5)
            path_waypoints = np.vstack([x_points, np.polyval(center_coeffs, x_points)]).T
            path_found = True
        
        steering_command = self.last_valid_steering; goal_point = None
        if path_found:
            distances = np.linalg.norm(path_waypoints, axis=1)
            candidates = path_waypoints[distances > LOOKAHEAD_DISTANCE]
            if len(candidates) > 0:
                goal_point = candidates[0]
                alpha = math.atan2(goal_point[1], goal_point[0])
                steering_command = -1 * math.atan(2 * VEHICLE_WHEELBASE * math.sin(alpha) / LOOKAHEAD_DISTANCE)

        speed_error = TARGET_SPEED - self.current_speed
        throttle = np.clip(KP_SPEED * speed_error, 0, MAX_THROTTLE)
        if not path_found: throttle = 0.0
        
        self.last_valid_steering = steering_command
        self.publish_control(throttle, steering_command)
        
        self.update_local_plot_data(blue_path_cones, yellow_path_cones, path_waypoints, goal_point)
        self.update_global_plot_data()

    def run(self):
        rate = rospy.Rate(CONTROL_RATE)
        plt.show(block=False)
        while not rospy.is_shutdown():
            self.control_logic()
            try:
                # --- 하나의 figure만 업데이트 ---
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
                plt.pause(0.001)
            except Exception: pass
            rate.sleep()

    # --- Plotting & Boilerplate Functions ---
    def setup_local_plot(self):
        self.ax_local.set_title("Local View"); self.ax_local.set_xlabel("X (local, m)"); self.ax_local.set_ylabel("Y (local, m)")
        self.blue_cones_plot, = self.ax_local.plot([], [], 'bo', ms=6); self.yellow_cones_plot, = self.ax_local.plot([], [], 'o', color='gold', ms=6)
        self.center_path_plot, = self.ax_local.plot([], [], 'g--', lw=2)
        self.ax_local.plot([0], [0], 'r>', ms=15)
        self.goal_point_plot, = self.ax_local.plot([], [], 'rx', ms=12, mew=3)
        self.ax_local.grid(True); self.ax_local.set_aspect('equal', 'box'); self.ax_local.set_xlim(-5, LIDAR_RANGE_CUTOFF); self.ax_local.set_ylim(-10, 10)

    def setup_global_plot(self):
        self.ax_global.set_title("Global Map"); self.ax_global.set_xlabel("X (global, m)"); self.ax_global.set_ylabel("Y (global, m)")
        self.global_blue_cones_plot, = self.ax_global.plot([], [], 'bo', ms=4, linestyle='None')
        self.global_yellow_cones_plot, = self.ax_global.plot([], [], 'o', color='gold', ms=4, linestyle='None')
        self.car_plot_global, = self.ax_global.plot([], [], 'r>', ms=10)
        self.ax_global.grid(True); self.ax_global.set_aspect('equal', 'box')

    def update_local_plot_data(self, blue, yellow, path, goal):
        if len(blue) > 0: self.blue_cones_plot.set_data(blue[:,0], blue[:,1])
        else: self.blue_cones_plot.set_data([], [])
        if len(yellow) > 0: self.yellow_cones_plot.set_data(yellow[:,0], yellow[:,1])
        else: self.yellow_cones_plot.set_data([], [])
        if len(path) > 0: self.center_path_plot.set_data(path[:, 0], path[:, 1])
        else: self.center_path_plot.set_data([], [])
        if goal is not None: self.goal_point_plot.set_data([goal[0]], [goal[1]])
        else: self.goal_point_plot.set_data([], [])

    def update_global_plot_data(self):
        if self.global_blue_cones:
            blue_cones_arr = np.array(self.global_blue_cones)
            self.global_blue_cones_plot.set_data(blue_cones_arr[:, 0], blue_cones_arr[:, 1])
        if self.global_yellow_cones:
            yellow_cones_arr = np.array(self.global_yellow_cones)
            self.global_yellow_cones_plot.set_data(yellow_cones_arr[:, 0], yellow_cones_arr[:, 1])
        gx, gy, gyaw = self.global_pose
        self.car_plot_global.set_data([gx], [gy])
        self.car_plot_global.set_marker((3, 0, -gyaw * 180/np.pi + 90))
        self.ax_global.relim(); self.ax_global.autoscale_view()

    def publish_control(self, t, s):
        cmd=ControlCommand(); cmd.throttle=np.clip(t,0,MAX_THROTTLE); cmd.steering=np.clip(s,-MAX_STEER,MAX_STEER); self.control_pub.publish(cmd)
    def start_driving(self):
        rospy.sleep(1.0) # 노드가 완전히 활성화될 때까지 잠시 대기
        go_pub = rospy.Publisher('/fsds/signal/go', Empty, queue_size=1)
        go_pub.publish(Empty())
        rospy.loginfo("주행 시작 'Go' 신호를 발행했습니다!")

if __name__ == '__main__':
    try:
        controller = LidarSubplotMapper()
        controller.run()
    except rospy.ROSInterruptException:
        pass