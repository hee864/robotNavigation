import numpy as np
import json

class RobotModel:
    
    def __init__(self, config_path):
        """
        로봇 모델 초기화
        
        Args:
            config_path (str): 설정 파일 경로
        """
        # 설정 파일 로드
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        vehicle_config = config['vehicle']
        start_point = config['start_point']
        
        # 차량의 위치 및 방향 정의
        self.x = float(start_point['x'])  # [m] and [pixel]
        self.y = float(start_point['y'])  # [m] and [pixel]
        self.yaw = float(vehicle_config['yaw'])  # [rad]
        
        # 차량의 크기 정의
        self.length = float(vehicle_config['length'])  # [m] and [pixel]
        self.width = float(vehicle_config['width'])  # [m] and [pixel]
        
        # 기본 제어 파라미터 설정
        self.velocity = 0.0  # [m/s] and [pixel/s]
        self.max_velocity = 80.0  # [m/s] and [pixel/s]
        self.min_velocity = -20.0  # [m/s] and [pixel/s]
        self.max_acceleration = 30.0  # [m/s^2] and [pixel/s^2]
        self.max_steering_angle = np.pi/4  # [rad]
    
    def update(self, acceleration, steering_angle, dt):
        """
        로봇의 상태를 가속도와 조향 각도를 바탕으로 업데이트.

        매개변수:
            acceleration (float): 차량의 가속도 [m/s^2]
            steering_angle (float): 앞바퀴의 조향 각도 [rad]
            dt (float): 상태를 업데이트할 시간 간격 [s]
        """
        # 가속도와 조향각 제한
        acceleration = np.clip(acceleration, -self.max_acceleration, self.max_acceleration)
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        # 최대 및 최소 속도 도달 시간 계산
        if acceleration > 0:  # 가속 중
            t_max = (self.max_velocity - self.velocity) / acceleration
        elif acceleration < 0:  # 감속 중
            t_max = (self.min_velocity - self.velocity) / acceleration
        else:  # 가속도 0
            t_max = np.inf
        
        # 두 가지 경우를 나눠 계산
        if dt <= t_max:  # 최대/최저 속도에 도달하지 않음
            v_end = self.velocity + acceleration * dt
            
            accel_distance = self.velocity * dt + 0.5 * acceleration * dt**2
            delta_yaw = accel_distance / self.length * np.tan(steering_angle)
            avg_yaw = self.yaw + delta_yaw / 2
            
            delta_x = accel_distance * np.cos(avg_yaw)
            delta_y = accel_distance * np.sin(avg_yaw)
            
        else:  # 최대/최저 속도에 도달한 이후 일정 속도로 이동
            if acceleration > 0:
                v_end = self.max_velocity
            else:
                v_end = self.min_velocity

            # 나머지 시간 및 거리 계산
            remaining_time = dt - t_max
            constant_distance = v_end * remaining_time
            
            # t_max 동안 가속/감속, 나머지 시간 일정 속도 유지
            accel_distance = self.velocity * t_max + 0.5 * acceleration * t_max**2
            delta_yaw = (accel_distance + constant_distance) / self.length * np.tan(steering_angle)
            avg_yaw = self.yaw + delta_yaw / 2

            delta_x = accel_distance * np.cos(avg_yaw) + constant_distance * np.cos(self.yaw + delta_yaw)
            delta_y = accel_distance * np.sin(avg_yaw) + constant_distance * np.sin(self.yaw + delta_yaw)

        # 상태 업데이트
        self.velocity = v_end
        self.yaw += delta_yaw
        self.x += delta_x
        self.y += delta_y
    
    def get_state(self):
        """
        현재 상태 반환
        
        Returns:
            tuple: (x, y, yaw, velocity) - 현재 위치, 방향, 속도
        """
        return self.x, self.y, self.yaw, self.velocity
