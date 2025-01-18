import cv2
import numpy as np
import json
import os

class Track:
    def __init__(self, config_path):
        """
        트랙 클래스 초기화
        
        Args:p
            config_path (str): 설정 파일 경로
        """
        # 설정 파일 로드
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        # 설정값 저장
        self.track_path = os.path.join('maps', config['map_name'])
        self.threshold = config['threshold']
        self.obstacle_above_threshold = config['obstacle_above_threshold']
        self.resolution = 1.0  # 1 픽셀 = 1 미터
        
        # 맵 관련 변수들
        self.obstacle_map = None
        self.width_px = None
        self.height_px = None
        self.width_m = None
        self.height_m = None
        
        # 시작점과 목표점 설정
        start = (config['start_point']['x'], config['start_point']['y'])
        goal = (config['goal_point']['x'], config['goal_point']['y'])
        
        # 초기 맵 로드 및 변환
        self._load_map()
        
        # 시작점과 도착점 설정
        self.set_start_point(start)
        self.set_goal_point(goal)
    
    def _load_map(self):
        """맵 이미지 로드 및 초기화"""
        # OpenCV를 사용하여 이미지 읽기
        img = cv2.imread(self.track_path, cv2.IMREAD_GRAYSCALE)
        
        # 이진화 (obstacle_above_threshold에 따라 장애물 설정)
        if self.obstacle_above_threshold:
            self.obstacle_map = (img > self.threshold).astype(np.int8)
        else:
            self.obstacle_map = (img <= self.threshold).astype(np.int8)
        
        # 맵의 크기 저장
        self.height_px, self.width_px = self.obstacle_map.shape
        self.width_m = self.width_px * self.resolution
        self.height_m = self.height_px * self.resolution
    
    def image_to_xy_pixel(self, image_pixel):
        """이미지 픽셀 좌표를 xy 픽셀 좌표로 변환"""
        image_pixel = np.array(image_pixel)
        return image_pixel
    
    def xy_pixel_to_image(self, xy_pixel):
        """xy 픽셀 좌표를 이미지 픽셀 좌표로 변환"""
        xy_pixel = np.array(xy_pixel)
        return xy_pixel
    
    def pixel_to_xy(self, pixel_coord):
        """픽셀 좌표를 xy 좌표계(미터)로 변환"""
        pixel_coord = np.array(pixel_coord)
        xy_pixel = self.image_to_xy_pixel(pixel_coord)
        return xy_pixel * self.resolution
    
    def xy_to_pixel(self, xy_coord):
        """xy 좌표(미터)를 픽셀 좌표로 변환"""
        xy_coord = np.array(xy_coord)
        px = (xy_coord / self.resolution).astype(int)
        return self.xy_pixel_to_image(px)
    
    def set_start_point(self, pixel_coord):
        """시작점 설정"""
        pixel_coord = np.array(pixel_coord)
        xy_pixel = self.image_to_xy_pixel(pixel_coord)
        if self.obstacle_map[xy_pixel[1], xy_pixel[0]] == 1:
            raise ValueError(f"시작점 위치 ({pixel_coord[0]}, {pixel_coord[1]})는 장애물 영역입니다.")
            
        self.start = pixel_coord
        self.start_xy = self.pixel_to_xy(pixel_coord)
    
    def set_goal_point(self, pixel_coord):
        """도착점 설정"""
        pixel_coord = np.array(pixel_coord)
        xy_pixel = self.image_to_xy_pixel(pixel_coord)
        if self.obstacle_map[xy_pixel[1], xy_pixel[0]] == 1:
            raise ValueError(f"도착점 위치 ({pixel_coord[0]}, {pixel_coord[1]})는 장애물 영역입니다.")
            
        self.goal = pixel_coord
        self.goal_xy = self.pixel_to_xy(pixel_coord)
    
    def get_map_extent(self):
        """맵의 실제 크기 범위를 반환 (미터 단위)"""
        return [0, self.width_m, 0, self.height_m]
    
    def get_obstacle_map(self):
        """
        장애물 맵을 반환
        
        Returns:
            numpy.ndarray: 장애물 맵 (0: 주행가능, 1: 장애물)
            shape: (height_px, width_px)
        """
        if self.obstacle_map is None:
            raise ValueError("장애물 맵이 초기화되지 않았습니다.")
        return self.obstacle_map.copy()
    
    def get_start_point(self, coord_type='pixel'):
        """
        시작점 좌표를 반환
        
        Args:
            coord_type (str): 반환할 좌표 타입 ('pixel' 또는 'xy')
        
        Returns:
            numpy.ndarray: 시작점 좌표
                - coord_type='pixel': 이미지 픽셀 좌표 (shape: (2,))
                - coord_type='xy': 실제 좌표 [m] (shape: (2,))
        """
        if coord_type == 'pixel':
            return self.start
        elif coord_type == 'xy':
            return self.start_xy
        else:
            raise ValueError("coord_type은 'pixel' 또는 'xy'여야 합니다.")
    
    def get_goal_point(self, coord_type='pixel'):
        """
        목표점 좌표를 반환
        
        Args:
            coord_type (str): 반환할 좌표 타입 ('pixel' 또는 'xy')
        
        Returns:
            numpy.ndarray: 목표점 좌표
                - coord_type='pixel': 이미지 픽셀 좌표 (shape: (2,))
                - coord_type='xy': 실제 좌표 [m] (shape: (2,))
        """
        if coord_type == 'pixel':
            return self.goal
        elif coord_type == 'xy':
            return self.goal_xy
        else:
            raise ValueError("coord_type은 'pixel' 또는 'xy'여야 합니다.")
    
    def get_start_goal_points(self, coord_type='pixel'):
        """
        시작점과 목표점의 좌표를 함께 반환
        
        Args:
            coord_type (str): 반환할 좌표 타입 ('pixel' 또는 'xy')
        
        Returns:
            tuple: (시작점 좌표, 목표점 좌표)
                - coord_type='pixel': 이미지 픽셀 좌표 (각각 shape: (2,))
                - coord_type='xy': 실제 좌표 [m] (각각 shape: (2,))
        """
        if coord_type == 'pixel':
            return self.start, self.goal
        elif coord_type == 'xy':
            return self.start_xy, self.goal_xy
        else:
            raise ValueError("coord_type은 'pixel' 또는 'xy'여야 합니다.")
