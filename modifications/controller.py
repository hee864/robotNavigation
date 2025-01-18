import numpy as np

class Controller:
    def __init__(self, k=1.0, k_soft=0.5, max_speed=30.0, resolution=0.3):
        """
        Stanley Controller 개선 버전
        Args:
            k (float): 크로스 트랙 오류 게인
            k_soft (float): 속도 안정화 게인
            max_speed (float): 최대 속도
            resolution (float): 경로 보간 해상도
        """
        self.k = k
        self.k_soft = k_soft
        self.max_speed = max_speed
        self.resolution = resolution
        self.previous_steering_angle = 0.0
        self.steering_rate_limit = np.radians(10)  # 최대 조향 변화율 (10도/s)

    def control(self, robot, path,obstacle_map, dt):
        """
        개선된 Stanley Controller 제어 로직
        Args:
            robot: 로봇 객체
            path: 계획된 경로
            dt: 시간 간격
        Returns:
            acceleration, steering_angle: 가속도와 조향각
        """
        current_pos = np.array([robot.x, robot.y])
        closest_idx, closest_point = self.find_closest_point(current_pos, path)
        
        # 동적 Look-Ahead Distance 조정
        look_ahead = self.dynamic_look_ahead(robot.velocity)
        future_idx = min(closest_idx + look_ahead, len(path) - 1)
        future_point = path[future_idx]

        # 헤딩 오류 및 크로스 트랙 오류 계산
        heading_error = self.calculate_heading_error(robot, future_point)
        cross_track_error = self.calculate_cross_track_error(robot, closest_point, path, closest_idx)

        # 곡률 기반 속도 감속
        curvature = self.estimate_curvature(path, closest_idx, look_ahead)
        target_speed = self.max_speed * (1 - np.clip(curvature * 2, 0, 0.8))

        # Stanley 조향각 계산
        raw_steering_angle = heading_error + np.arctan2(self.k * cross_track_error, (robot.velocity + self.k_soft))

        # 조향각 변화율 제한
        steering_angle = np.clip(
            raw_steering_angle,
            self.previous_steering_angle - self.steering_rate_limit * dt,
            self.previous_steering_angle + self.steering_rate_limit * dt
        )
        self.previous_steering_angle = steering_angle

        # 속도 제어
        acceleration = (target_speed - robot.velocity) / dt
        return np.clip(acceleration, -robot.max_acceleration, robot.max_acceleration), steering_angle

    def dynamic_look_ahead(self, velocity):
        """
        속도에 따라 Look-Ahead Distance 조정
        """
        base_look_ahead = 5
        return base_look_ahead + int(velocity / 5)

    def estimate_curvature(self, path, idx, look_ahead):
        """
        경로의 곡률 추정
        """
        if idx + look_ahead >= len(path):
            return 0.0
        p1 = np.array(path[idx])
        p2 = np.array(path[idx + look_ahead // 2])
        p3 = np.array(path[idx + look_ahead])

        # 세 점을 이용한 곡률 계산
        vec1 = p2 - p1
        vec2 = p3 - p2
        angle = np.arccos(
            np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2) + 1e-6)
        )
        distance = np.linalg.norm(p3 - p1)
        return angle / (distance + 1e-6)

    def find_closest_point(self, current_pos, path):
        distances = [np.linalg.norm(current_pos - np.array(p)) for p in path]
        closest_idx = int(np.argmin(distances))
        return closest_idx, path[closest_idx]

    def calculate_cross_track_error(self, robot, closest_point, path, closest_idx):
        next_point = path[min(closest_idx + 1, len(path) - 1)]
        path_vector = np.array(next_point) - np.array(closest_point)
        robot_vector = np.array([robot.x, robot.y]) - np.array(closest_point)
        return np.cross(path_vector, robot_vector) / (np.linalg.norm(path_vector) + 1e-6)

    def calculate_heading_error(self, robot, target_point):
        target_heading = np.arctan2(target_point[1] - robot.y, target_point[0] - robot.x)
        heading_error = target_heading - robot.yaw
        return np.arctan2(np.sin(heading_error), np.cos(heading_error))
