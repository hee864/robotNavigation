import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Polygon

class Visualizer:
    def __init__(self, figsize=(10, 10)):
        """
        시각화 클래스 초기화
        
        Args:
            figsize (tuple): matplotlib figure 크기
        """
        self.fig = plt.figure(figsize=figsize)
        self.visualization_counter = 0
        self.car_positions_x = []
        self.car_positions_y = []
        self.collision_point = None  # 충돌 지점 저장용
        
    def update_car_position(self, x, y):
        """차량 위치 기록 업데이트"""
        self.car_positions_x.append(x)
        self.car_positions_y.append(y)
        
    def set_collision_point(self, x, y):
        """충돌 지점 설정"""
        self.collision_point = (x, y)
        
    def draw_car(self, car):
        """직사각형 모양의 자동차 그리기"""
        # 차량의 네 모서리 좌표 계산
        cos_yaw = np.cos(car.yaw)
        sin_yaw = np.sin(car.yaw)
        
        corners = [
            # 앞쪽 왼쪽
            (car.x + cos_yaw * car.length/2 - sin_yaw * car.width/2,
             car.y + sin_yaw * car.length/2 + cos_yaw * car.width/2),
            # 앞쪽 오른쪽
            (car.x + cos_yaw * car.length/2 + sin_yaw * car.width/2,
             car.y + sin_yaw * car.length/2 - cos_yaw * car.width/2),
            # 뒤쪽 오른쪽
            (car.x - cos_yaw * car.length/2 + sin_yaw * car.width/2,
             car.y - sin_yaw * car.length/2 - cos_yaw * car.width/2),
            # 뒤쪽 왼쪽
            (car.x - cos_yaw * car.length/2 - sin_yaw * car.width/2,
             car.y - sin_yaw * car.length/2 + cos_yaw * car.width/2)
        ]
        
        # 차량 본체 그리기 (직사각형)
        car_polygon = Polygon(corners, color='magenta', alpha=0.5, label='Car')
        plt.gca().add_patch(car_polygon)
        
        # 차량 진행 방향 표시 (화살표)
        front_center = (
            car.x + cos_yaw * car.length/2,
            car.y + sin_yaw * car.length/2
        )
        direction_len = car.length/2
        plt.arrow(car.x, car.y,
                 direction_len * cos_yaw,
                 direction_len * sin_yaw,
                 head_width=car.width/4, color='red')
        
    def find_closest_path_point(self, car_pos, path):
        """
        현재 차량 위치에서 가장 가까운 경로 점의 인덱스를 찾습니다.
        
        Args:
            car_pos (tuple): 차량의 현재 위치 (x, y)
            path (numpy.ndarray): 경로 점들의 배열
        
        Returns:
            int: 가장 가까운 경로 점의 인덱스
        """
        distances = np.sqrt(np.sum((path - np.array(car_pos))**2, axis=1))
        return np.argmin(distances)

    def visualize(self, car, path, obstacle_map, start, goal, distance_to_goal, simulation_time, update_interval=5, force_progress=None, goal_radius=None):
        """
        시뮬레이션 상태 시각화
        
        Args:
            force_progress (float, optional): 강제로 설정할 진행률. None이면 계산된 값 사용
            goal_radius (float, optional): 목표 도달 판정 거리
        """
        if self.visualization_counter % update_interval == 0:
            plt.clf()
            
            # 트랙 표시 (이미지 그대로 표시, origin='upper'로 설정)
            plt.imshow(obstacle_map, cmap='gray', origin='upper')
            
            # 경로 표시 (y좌표 변환 없이 그대로 사용)
            plt.plot(path[:, 0], path[:, 1], 'b-', label='Planned Path')
            
            # 차량 궤적 표시 (y좌표 변환 없이 그대로 사용)
            plt.plot(self.car_positions_x, self.car_positions_y, 'm--', 
                    label='Car Trajectory', alpha=0.5)
            
            # 시작점과 목표점 표시 (y좌표 변환 없이 그대로 사용)
            plt.plot(start[0], start[1], 'go', label='Start')
            plt.plot(goal[0], goal[1], 'ro', label='Goal')
            
            # 차량 그리기 (y좌표 변환 없이 그대로 사용)
            self.draw_car(car)
            
            # 현재 경로 진행률 계산
            if force_progress is not None:
                progress_percentage = force_progress
            else:
                closest_idx = self.find_closest_path_point((car.x, car.y), path)
                progress_percentage = (closest_idx / (len(path) - 1)) * 100
            
            # 충돌 지점 표시 (y좌표 변환 없이 그대로 사용)
            if self.collision_point is not None:
                collision_x, collision_y = self.collision_point
                
                plt.plot(collision_x, collision_y, 'rx', markersize=20, 
                        markeredgewidth=3, label='Collision Point')
                circle = plt.Circle((collision_x, collision_y), radius=5, 
                                  color='red', fill=False, linewidth=2)
                plt.gca().add_patch(circle)
                plt.text(collision_x + 10, collision_y + 10, 'COLLISION!', 
                        color='red', fontsize=15, fontweight='bold',
                        bbox=dict(facecolor='white', edgecolor='red', alpha=0.7))
            
            # 현재 상태 정보 표시 (시뮬레이션 시간, 진행률, 속도 추가)
            status = 'COLLISION!' if self.collision_point is not None else f'Distance to Goal: {distance_to_goal:.1f}m (<{goal_radius}m)'
            plt.title(f'Time: {simulation_time:.1f}s | Progress: {progress_percentage:.1f}% | Velocity: {car.velocity:.1f}m/s\n{status}\n'
                     f'Car Position: ({car.x:.1f}m, {car.y:.1f}m)\n'
                     f'Yaw: {np.degrees(car.yaw):.1f}°',
                     color='red' if self.collision_point is not None else 'black')
            
            plt.legend()
            plt.pause(0.001)
            
        self.visualization_counter += 1
    
    def show(self):
        """최종 결과 표시"""
        plt.show() 