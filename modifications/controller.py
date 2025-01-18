import numpy as np
from scipy.interpolate import splprep, splev

class Controller:
    def __init__(self, k=1.0, k_soft=0.5, max_speed=30.0, resolution=0.3):
        """
        Stanley Controller 기반 차량 제어
        Args:
            k (float): 조향 강도 게인
            k_soft (float): 소프트닝 게인
            max_speed (float): 전체 속도 제한
            resolution (float): 경로 보간 해상도
        """
        self.k = k
        self.k_soft = k_soft
        self.max_speed = max_speed  # 전체 속도 제한
        self.resolution = resolution
        self.integral_error = 0.0
        self.previous_cross_track_error = 0.0
        self.previous_steering_angle = 0.0

    def control(self, robot, path, obstacle_map, dt):
        """
        Stanley Controller를 사용하여 차량 제어
        Args:
            robot: 로봇 객체
            path: 계획된 경로
            obstacle_map: 장애물 맵
            dt: 시간 간격
        Returns:
            acceleration, steering_angle: 가속도와 조향각
        """
        current_pos = np.array([robot.x, robot.y])
        closest_idx, closest_point = self.find_closest_point(current_pos, path)

        # 헤딩 오류와 크로스 트랙 오류 계산
        heading_error = self.calculate_heading_error(robot, path, look_ahead_base=5)
        cross_track_error = self.calculate_cross_track_error(robot, closest_point, path, closest_idx)

        # I-Term과 D-Term (적분 및 미분 항)
        self.integral_error += cross_track_error * dt
        cross_track_error_derivative = (cross_track_error - self.previous_cross_track_error) / dt
        self.previous_cross_track_error = cross_track_error

        # Stanley 조향각 계산 및 속도 기반 보정
        raw_steering_angle = heading_error + np.arctan2(
            self.k * cross_track_error + 0.1 * self.integral_error + 0.1 * cross_track_error_derivative,
            (robot.velocity + self.k_soft)
        )

        # 속도 감속 (조향각에 비례하여 속도를 줄임)
        steering_ratio = abs(raw_steering_angle) / np.radians(45)  # 조향각이 45도 기준
        speed = max(self.max_speed * (1 - 2.0 * steering_ratio), 5.0)  # 속도 비율 조절
        acceleration = (speed - robot.velocity) / dt

        # 전체 가속도와 조향각 제한
        acceleration = np.clip(acceleration, -robot.max_acceleration / 2, robot.max_acceleration / 2)
        steering_angle = np.clip(raw_steering_angle, -robot.max_steering_angle, robot.max_steering_angle)

        # 이전 조향각 업데이트
        self.previous_steering_angle = steering_angle

        return acceleration, steering_angle


    def find_closest_point(self, current_pos, path):
        """
        경로에서 가장 가까운 점을 찾음
        """
        distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]
        closest_idx = int(np.argmin(distances))
        return closest_idx, path[closest_idx]

    def calculate_cross_track_error(self, robot, closest_point, path, closest_idx):
        """
        크로스 트랙 오류 계산
        """
        next_point = path[min(closest_idx + 1, len(path) - 1)]
        path_vector = np.array(next_point) - np.array(closest_point)
        robot_vector = np.array([robot.x, robot.y]) - np.array(closest_point)
        return np.cross(path_vector, robot_vector) / np.linalg.norm(path_vector)

    def calculate_heading_error(self, robot, path, look_ahead_base=7):
        """
        동적 Look-Ahead Distance를 사용해 헤딩 오류를 계산
        """
        look_ahead = look_ahead_base + int(robot.velocity / 10.0)  # 속도에 비례해 증가
        look_ahead = min(look_ahead, 15)  # 최대 제한

        distances = np.linalg.norm(path - np.array([robot.x, robot.y]), axis=1)
        closest_idx = np.argmin(distances)
        future_idx = min(closest_idx + look_ahead, len(path) - 1)
        future_point = path[future_idx]

        target_heading = np.arctan2(future_point[1] - robot.y, future_point[0] - robot.x)
        heading_error = target_heading - robot.yaw
        return np.arctan2(np.sin(heading_error), np.cos(heading_error))
