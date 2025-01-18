# main.py

import numpy as np
import matplotlib.pyplot as plt
from no_modifications.robot_model import RobotModel
from no_modifications.track import Track
from modifications.path_planner import PathPlanner
from modifications.controller import Controller
from math import sqrt
from visualizer import Visualizer
import json
import os
from datetime import datetime

# 설정 파일 경로
config_path = 'configs/config_track1.json'

def check_collision(car, obstacle_map, track):
    """
    차량과 장애물 간의 충돌을 확인합니다.
    차량의 네 모서리와 변을 검사합니다.
    
    Returns:
        tuple: (충돌 여부, 충돌 지점) - 충돌이 없으면 (False, None)
    """
    # 차량이 차지하는 영역 계산
    car_length = car.length
    car_width = car.width
    
    # 차량의 네 모서리 좌표 계산
    cos_yaw = np.cos(car.yaw)
    sin_yaw = np.sin(car.yaw)
    
    corners = [
        # 앞쪽 왼쪽
        (car.x + cos_yaw * car_length/2 - sin_yaw * car_width/2,
         car.y + sin_yaw * car_length/2 + cos_yaw * car_width/2),
        # 앞쪽 오른쪽
        (car.x + cos_yaw * car_length/2 + sin_yaw * car_width/2,
         car.y + sin_yaw * car_length/2 - cos_yaw * car_width/2),
        # 뒤쪽 왼른쪽
        (car.x - cos_yaw * car_length/2 + sin_yaw * car_width/2,
         car.y - sin_yaw * car_length/2 - cos_yaw * car_width/2),
        # 뒤쪽 왼쪽
        (car.x - cos_yaw * car_length/2 - sin_yaw * car_width/2,
         car.y - sin_yaw * car_length/2 + cos_yaw * car_width/2)
    ]
    
    def check_line_segment(start, end, num_points=10):
        """두 점을 잇는 선분 상의 점들을 검사"""
        x0, y0 = start
        x1, y1 = end
        
        for i in range(num_points + 1):
            t = i / num_points
            x = int(x0 + t * (x1 - x0))
            y = int(y0 + t * (y1 - y0))
            
            if (0 <= x < obstacle_map.shape[1] and 
                0 <= y < obstacle_map.shape[0]):
                if obstacle_map[y, x] == 1:
                    return True, (x, y)
            else:
                return True, (x, y)
        return False, None
    
    # 각 모서리 점에서 충돌 검사
    for corner in corners:
        x, y = corner
        check_x = int(x)
        check_y = int(y)
        
        if (0 <= check_x < obstacle_map.shape[1] and 
            0 <= check_y < obstacle_map.shape[0]):
            if obstacle_map[check_y, check_x] == 1:
                return True, corner
        else:
            return True, corner
    
    # 각 변에서 충돌 검사
    for i in range(4):
        start = corners[i]
        end = corners[(i + 1) % 4]  # 마지막 점은 첫 점과 연결
        
        collision, collision_point = check_line_segment(start, end)
        if collision:
            return True, collision_point
    
    return False, None

def save_simulation_results(config_path, collision, reached_goal, 
                          simulation_time, progress, real_time, collision_point=None):
    """
    시뮬레이션 결과를 파일로 저장
    
    Args:
        config_path (str): 사용된 설정 파일 경로
        collision (bool): 충돌 여부
        reached_goal (bool): 목표 도달 여부
        simulation_time (float): 시뮬레이션 소요 시간
        progress (float): 경로 진행률
        real_time (float): 실제 시뮬레이션 실행 시간
        collision_point (tuple, optional): 충돌 지점 좌표
    """
    # 결과 저장 디렉토리 생성
    results_dir = 'results'
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    # 현재 시간을 파일명에 포함
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    config_name = os.path.splitext(os.path.basename(config_path))[0]
    result_filename = f"{config_name}_result_{timestamp}.json"
    
    # 설정 파일 로드
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # 결과 데이터 구성
    result_data = {
        "configuration": config,
        "simulation_result": {
            "collision_occurred": collision,
            "reached_goal": reached_goal,
            "simulation_time": simulation_time,
            "real_execution_time": real_time,
            "progress_percentage": progress
        }
    }
    
    if collision_point is not None:
        result_data["simulation_result"]["collision_point"] = {
            "x": float(collision_point[0]),
            "y": float(collision_point[1])
        }
    
    # 결과 저장
    result_path = os.path.join(results_dir, result_filename)
    with open(result_path, 'w') as f:
        json.dump(result_data, f, indent=4)
    
    print(f"시뮬레이션 결과가 저장되었습니다: {result_path}")

def main():
    
    # 설정 파일 로드
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # Track
    track = Track(config_path)
    start = track.get_start_point(coord_type='pixel')
    goal = track.get_goal_point(coord_type='pixel')
    
    # Path Planning
    planner = PathPlanner(track.get_obstacle_map())
    path = planner.find_path(tuple(start), tuple(goal))
    
    if not path:
        print("경로를 찾을 수 없습니다.")
        return
    path = np.array(path)
    
    
    # Robot Model
    robot = RobotModel(config_path)
    
    # Controller
    controller = Controller()
    
    # Visualizer
    visualizer = Visualizer()
    
    # 시뮬레이션 파라미터
    dt = 0.1
    goal_radius = float(config['goal_radius'])
    simulation_time = 0.0  # 시뮬레이션 시간 추적
    
    # 목표 도달 여부
    reached_goal = False
    
    # 시뮬레이션 시작 시간 기록
    start_time = datetime.now()
    
    while not reached_goal:
        visualizer.update_car_position(robot.x, robot.y)
        # 진행 상태 출력
        print(f"Simulation Time: {simulation_time:.2f}s | Robot Position: ({robot.x:.2f}, {robot.y:.2f}) | Velocity: {robot.velocity:.2f} m/s")
        # 목표점 도달 확인 (goal_radius 사용)
        distance_to_goal = sqrt((robot.x - goal[0])**2 + (robot.y - goal[1])**2)
        
        # 충돌 감지 및 시각화
        collision, collision_point = check_collision(robot, track.get_obstacle_map(), track)
        if collision:
            print("장애물과 충돌했습니다!")
            print(f"충돌 지점: ({collision_point[0]}, {collision_point[1]})")
            
            visualizer.set_collision_point(collision_point[0], collision_point[1])
            visualizer.visualize(robot, path, track.get_obstacle_map(), start, goal, 
                               distance_to_goal, simulation_time, goal_radius=goal_radius)
            
            # 실제 실행 시간 계산
            real_time = (datetime.now() - start_time).total_seconds()
            
            # 결과 저장
            closest_idx = visualizer.find_closest_path_point((robot.x, robot.y), path)
            progress = (closest_idx / (len(path) - 1)) * 100
            save_simulation_results(
                config_path, True, False, simulation_time, 
                progress, real_time, collision_point
            )
            
            plt.pause(2)
            break
            
        if distance_to_goal < goal_radius:
            print("목표점에 도착했습니다!")
            reached_goal = True
            
            # 실제 실행 시간 계산
            real_time = (datetime.now() - start_time).total_seconds()
            
            # 마지막 상태 시각화
            visualizer.visualize(robot, path, track.get_obstacle_map(), start, goal, 
                               distance_to_goal, simulation_time, force_progress=100.0, 
                               goal_radius=goal_radius)
            
            # 결과 저장  
            save_simulation_results(
                config_path, False, True, simulation_time, 
                100.0, real_time
            )
            break
        
        # Controller 제어 및 속도 제어
        acceleration, steering_angle = controller.control(robot, path, track.get_obstacle_map(), dt)
        
        # 차량 상태 업데이트
        robot.update(acceleration=acceleration, steering_angle=steering_angle, dt=dt)
        
        # 시각화 업데이트
        visualizer.visualize(robot, path, track.get_obstacle_map(), start, goal, 
                           distance_to_goal, simulation_time, goal_radius=goal_radius)
        
        # 시뮬레이션 시간 업데이트
        simulation_time += dt
    
    # 시뮬레이션 종료 후 최종 상태 표시
    if collision:
        plt.title(f"Simulation ended: Collision detected! (Time: {simulation_time:.1f}s)", 
                 color='red', fontsize=15)
    elif reached_goal:
        plt.title(f"Simulation ended: Goal reached! (Time: {simulation_time:.1f}s)", 
                 color='green', fontsize=15)
    
    visualizer.show()
    
if __name__ == "__main__":
    main()

