#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 필요한 라이브러리들을 가져옵니다.
import matplotlib
matplotlib.use('TkAgg')  # ROS 환경에서 Matplotlib 그래프가 오류 없이 잘 뜨게 하기 위한 설정
import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion # 쿼터니언(quaternion) 타입의 각도를 오일러 각도(yaw)로 변환하기 위해 사용
import matplotlib.pyplot as plt

# FSDS 시뮬레이터와 통신하기 위한 메시지 타입들
from fs_msgs.msg import ControlCommand, Track, Cone
from nav_msgs.msg import Odometry

# 경로를 부드럽게 만들기 위한 스플라인(spline) 보간 함수
from scipy.interpolate import splprep, splev

# ==============================================================================
# ======================== 기본 파라미터 (Basic Parameters) ======================
# ==============================================================================
# 이 값들을 조절하여 차량의 주행 특성을 변경할 수 있습니다. (튜닝 포인트)

TARGET_SPEED = 5.0              # 목표 주행 속도 (m/s)
KP_SPEED = 0.5                  # 속도 제어를 위한 비례(Proportional) 제어 상수. 클수록 목표 속도에 더 빠르게 반응합니다.

LOOKAHEAD_DISTANCE = 3.0        # Pure Pursuit 알고리즘의 전방 주시 거리 (m). 클수록 경로는 부드러워지지만 코너를 크게 돌고, 작을수록 경로를 잘 따라가지만 주행이 불안정해질 수 있습니다.
VEHICLE_LENGTH = 1.55           # 차량의 축거(앞바퀴와 뒷바퀴 사이의 거리) (m). 이 값은 조향각 계산에 사용됩니다.
MAX_STEER_ANGLE = 0.5           # 차량의 최대 조향 각도 (rad). 시뮬레이터의 차량 모델에 맞춰 설정합니다.
INTERPOLATION_FACTOR = 10       # 경로 보간 배수. 콘의 중점에서 얻은 경로점들을 얼마나 더 촘촘하게 만들지를 결정합니다. 클수록 경로가 부드러워집니다.
# ==============================================================================


class PurePursuitFinalV3:
    """
    Pure Pursuit 경로 추종 알고리즘을 구현한 컨트롤러 클래스.
    - `track_callback`: 트랙 정보를 받아 주행 경로를 생성.
    - `odom_callback`: 차량의 현재 상태를 받아 제어 로직을 실행.
    - `control_loop`: Pure Pursuit 계산을 통해 조향각과 속도를 결정하고 차량에 명령.
    """
    def __init__(self):
        # ROS 노드를 'pure_pursuit_controller'라는 이름으로 초기화합니다.
        rospy.init_node("pure_pursuit_controller")
        
        # 주행 경로, 차량의 현재 상태(위치, 속도) 등을 저장할 변수들을 초기화합니다.
        self.path = None
        self.current_pose = np.zeros(3)  # [x, y, yaw]
        self.current_speed = 0.0
        self.is_path_generated = False  # 경로가 한 번만 생성되도록 하는 플래그
        
        # 주행 궤적과 콘의 위치를 시각화하기 위해 좌표들을 저장할 리스트
        self.history_x, self.history_y = [], []
        self.blue_cones_x, self.blue_cones_y = [], []
        self.yellow_cones_x, self.yellow_cones_y = [], []
        
        # Matplotlib을 사용한 실시간 시각화 그래프 설정
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.setup_plot()
        
        # ROS 퍼블리셔(Publisher)와 서브스크라이버(Subscriber) 설정
        # '/fsds/control_command' 토픽으로 차량 제어 명령(조향, 스로틀, 브레이크)을 보냅니다.
        self.control_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
        # '/fsds/testing_only/track' 토픽에서 트랙 정보(콘 위치)를 받습니다.
        rospy.Subscriber('/fsds/testing_only/track', Track, self.track_callback, queue_size=1)
        # '/fsds/testing_only/odom' 토픽에서 차량의 위치/자세/속도 정보를 받습니다.
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, self.odom_callback, queue_size=1)
        
        rospy.loginfo("Pure Pursuit 컨트롤러 노드가 성공적으로 시작되었습니다.")

    def setup_plot(self):
        """시각화 그래프의 제목, 라벨, 범례 등 초기 모습을 설정합니다."""
        self.ax.set_title("Pure Pursuit Controller")
        self.ax.set_xlabel("X (m)"), self.ax.set_ylabel("Y (m)")
        self.path_plot, = self.ax.plot([], [], 'g--', label="Path")
        self.blue_cone_plot, = self.ax.plot([], [], 'bo', markersize=4, label="Blue Cones")
        self.yellow_cone_plot, = self.ax.plot([], [], 'o', color='gold', markersize=4, label="Yellow Cones")
        self.history_plot, = self.ax.plot([], [], 'r-', label="Car Trajectory")
        self.car_plot, = self.ax.plot([], [], 'ko', markersize=6, label="Car")
        self.lookahead_plot, = self.ax.plot([], [], 'mx', markersize=8, mew=2, label="Lookahead Point")
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_aspect('equal', adjustable='box') # x, y 축의 스케일을 동일하게 설정하여 실제 비율을 유지

    def track_callback(self, track_msg: Track):
        """
        트랙 정보를 받아 주행 경로를 생성하는 콜백 함수 (시뮬레이션 시작 시 한 번만 실행됨).
        """
        if self.is_path_generated: return
        rospy.loginfo("트랙 정보를 수신하여 주행 경로 생성을 시작합니다...")
        
        # 1. 트랙 메시지로부터 파란색 콘과 노란색 콘의 위치를 각각 분리합니다.
        blue_cones = [(c.location.x, c.location.y) for c in track_msg.track if c.color == Cone.BLUE]
        yellow_cones = [(c.location.x, c.location.y) for c in track_msg.track if c.color == Cone.YELLOW]
        if not blue_cones or not yellow_cones: return
        
        # 시각화를 위해 콘의 좌표들을 클래스 변수에 저장합니다.
        self.blue_cones_x = [c[0] for c in blue_cones]
        self.blue_cones_y = [c[1] for c in blue_cones]
        self.yellow_cones_x = [c[0] for c in yellow_cones]
        self.yellow_cones_y = [c[1] for c in yellow_cones]
        
        # 2. 콘 쌍을 '순차적으로' 찾아 중앙 경로(waypoints)를 생성합니다.
        raw_path = []
        # 시작점은 원점(0,0)에서 가장 가까운 파란색 콘으로 정합니다.
        current_blue = min(blue_cones, key=lambda p: np.hypot(p[0], p[1]))
        
        while blue_cones:
            # 현재 처리 중인 파란색 콘을 리스트에서 제거합니다 (중복 처리 방지).
            blue_cones.remove(current_blue)
            # 현재 파란색 콘과 가장 가까운 노란색 콘을 찾아 짝을 짓습니다.
            matching_yellow = min(yellow_cones, key=lambda p: np.hypot(p[0]-current_blue[0], p[1]-current_blue[1]))
            # 두 콘의 중앙점을 계산하여 경로점(waypoint)으로 추가합니다.
            raw_path.append(((current_blue[0] + matching_yellow[0]) / 2.0, (current_blue[1] + matching_yellow[1]) / 2.0))
            
            if not blue_cones: break
            # 다음 파란색 콘은 '방금 처리한' 파란색 콘과 가장 가까운 콘으로 선택합니다. 이것이 경로를 순서대로 만드는 핵심입니다.
            current_blue = min(blue_cones, key=lambda p: np.hypot(p[0]-current_blue[0], p[1]-current_blue[1]))
        
        raw_path = np.array(raw_path)

        # 3. '신발끈 공식(Shoelace Formula)'을 사용하여 경로의 방향을 항상 반시계 방향(CCW)으로 통일합니다.
        # 이렇게 하면 경로가 시계 방향으로 생성되어도 조향각 계산이 반대로 되는 것을 막을 수 있습니다.
        signed_area = 0.0
        for i in range(len(raw_path)):
            x1, y1 = raw_path[i]
            x2, y2 = raw_path[(i + 1) % len(raw_path)]
            signed_area += (x1 * y2 - x2 * y1)
        if signed_area < 0: # 면적이 음수이면 시계 방향이므로 경로를 뒤집어 반시계 방향으로 만듭니다.
            raw_path = raw_path[::-1]

        # 4. 생성된 경로점들을 '스플라인 보간(spline interpolation)'을 통해 부드러운 곡선 경로로 만듭니다.
        if len(raw_path) > 3:
            tck, u = splprep([raw_path[:, 0], raw_path[:, 1]], s=1.0, k=3, per=True)
            u_new = np.linspace(u.min(), u.max(), len(raw_path) * INTERPOLATION_FACTOR)
            x_new, y_new = splev(u_new, tck, der=0)
            self.path = np.c_[x_new, y_new]
            self.is_path_generated = True # 경로 생성이 완료되었음을 표시
            
            # 그래프의 x, y축 범위를 전체 트랙에 맞게 고정합니다.
            margin = 10
            self.ax.set_xlim(self.path[:, 0].min() - margin, self.path[:, 0].max() + margin)
            self.ax.set_ylim(self.path[:, 1].min() - margin, self.path[:, 1].max() + margin)
            rospy.loginfo("주행 경로 생성 완료!")

    def odom_callback(self, odom_msg: Odometry):
        """차량의 Odometry 정보를 받아 현재 속도, 위치, 자세를 업데이트합니다."""
        if not self.is_path_generated: return # 경로가 생성되기 전에는 아무것도 하지 않습니다.
        
        # 차량의 실제 속도(speed)를 계산합니다. 전진(x) 및 측면(y) 속도를 모두 고려합니다.
        linear_vel = odom_msg.twist.twist.linear
        self.current_speed = math.sqrt(linear_vel.x**2 + linear_vel.y**2)
        
        # 차량의 위치(x, y)와 현재 주행 방향(yaw)을 업데이트합니다.
        q = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw])
        
        # 주행한 궤적을 시각화를 위해 저장합니다.
        self.history_x.append(self.current_pose[0]); self.history_y.append(self.current_pose[1])
        
        # 모든 상태 업데이트가 끝나면, 메인 제어 로직을 실행합니다.
        self.control_loop()

    def control_loop(self):
        """Pure Pursuit 알고리즘을 기반으로 차량의 조향과 속도를 제어합니다."""
        
        # --- 1. 경로 추종 (조향 제어) ---
        # 차량에서 가장 가까운 경로점을 찾습니다.
        distances = np.linalg.norm(self.path - self.current_pose[:2], axis=1)
        closest_idx = np.argmin(distances)

        # 가장 가까운 경로점으로부터 'LOOKAHEAD_DISTANCE'만큼 앞에 있는 목표점(target_point)을 찾습니다.
        target_idx = closest_idx
        for i in range(len(self.path)):
            check_idx = (closest_idx + i) % len(self.path) # 경로의 끝에 도달하면 다시 처음부터 찾도록 함 (원형 큐)
            if distances[check_idx] >= LOOKAHEAD_DISTANCE:
                target_idx = check_idx
                break
        target_point = self.path[target_idx]
        
        # 목표점과 차량 사이의 각도(alpha)를 계산합니다.
        alpha = math.atan2(target_point[1] - self.current_pose[1], target_point[0] - self.current_pose[0]) - self.current_pose[2]
        
        # Pure Pursuit 공식을 사용하여 목표점으로 가기 위한 조향각(delta)을 계산합니다.
        delta = math.atan2(2.0 * VEHICLE_LENGTH * math.sin(alpha), LOOKAHEAD_DISTANCE)
        
        # 계산된 조향각을 최대 허용치 내로 제한합니다.
        steering_angle = np.clip(delta, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
        
        # --- 2. 속도 제어 ---
        # 목표 속도와 현재 속도의 차이(오차)를 계산합니다.
        speed_error = TARGET_SPEED - self.current_speed
        # 비례 제어(P-control)를 사용하여 스로틀 값을 계산합니다. (오차가 클수록 스로틀을 더 많이 밟음)
        throttle = np.clip(KP_SPEED * speed_error, 0.0, 0.4) # 스로틀 값을 0.0 ~ 0.4 사이로 제한하여 안정성 확보
        
        # --- 3. 최종 명령 발행 ---
        # FSDS 시뮬레이터의 좌표계에 맞추기 위해 조향각에 음수(-)를 취합니다.
        cmd_msg = ControlCommand(throttle=throttle, steering=-steering_angle, brake=0.0)
        self.control_pub.publish(cmd_msg)
        
        # 시각화 그래프를 업데이트합니다.
        self.update_plot_data(target_point)

    def update_plot_data(self, target_point):
        """Matplotlib 그래프의 각 요소들을 최신 데이터로 업데이트합니다."""
        self.path_plot.set_data(self.path[:, 0], self.path[:, 1])
        self.blue_cone_plot.set_data(self.blue_cones_x, self.blue_cones_y)
        self.yellow_cone_plot.set_data(self.yellow_cones_x, self.yellow_cones_y)
        self.history_plot.set_data(self.history_x, self.history_y)
        self.car_plot.set_data([self.current_pose[0]], [self.current_pose[1]])
        self.lookahead_plot.set_data([target_point[0]], [target_point[1]])

# 이 스크립트가 직접 실행될 때만 아래 코드가 실행됩니다.
if __name__ == '__main__':
    try:
        controller = PurePursuitFinalV3()
        plt.show(block=False) # 그래프 창이 다른 작업을 막지 않도록 설정
        # rospy.is_shutdown()이 True가 될 때까지 (Ctrl+C 등) 루프를 돌며 그래프를 계속 업데이트합니다.
        while not rospy.is_shutdown():
            plt.pause(0.01)
    except rospy.ROSInterruptException:
        rospy.loginfo("노드가 종료되었습니다.")