import numpy as np
import heapq
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import splprep, splev

class PathPlanner:
    def __init__(self, obstacle_map, robot_size=5.0, safety_margin=6.0):
        self.obstacle_map = obstacle_map
        self.robot_size = robot_size
        self.safety_margin = safety_margin
        self.distance_map = self._generate_distance_map()
        self.safe_mask = self._generate_safe_mask()

    def _generate_distance_map(self):
        return distance_transform_edt(self.obstacle_map == 0)

    def _generate_safe_mask(self):
        return self.distance_map > (self.robot_size + self.safety_margin)

    def find_path(self, start, goal):
        rows, cols = self.obstacle_map.shape
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                path = self.reconstruct_path(came_from, start, goal)
                path = self.simplify_path(path)  # 직선 구간 보정
                path = self.interpolate_path(path, resolution=0.5)
                path = self.smooth_path(path, smoothing_factor=0.5)
                return path

            for neighbor in self.get_neighbors(current, rows, cols):
                if not self.is_valid_point(neighbor):
                    continue

                curvature_penalty = self.curvature_cost(current, neighbor, came_from)
                tentative_g_score = g_score[current] + 1 + curvature_penalty

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def curvature_cost(self, current, neighbor, came_from):
        if current in came_from:
            prev = came_from[current]
            vec1 = np.array(current) - np.array(prev)
            vec2 = np.array(neighbor) - np.array(current)
            angle = np.arccos(
                np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2) + 1e-6)
            )
            return abs(angle) * 5  # 곡률 비용 가중치
        return 0

    def get_neighbors(self, point, rows, cols):
        x, y = point
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
        return [(x + dx, y + dy) for dx, dy in directions if 0 <= x + dx < cols and 0 <= y + dy < rows]

    def is_valid_point(self, point):
        x, y = int(point[0]), int(point[1])
        return self.safe_mask[y, x]

    def reconstruct_path(self, came_from, start, goal):
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    def simplify_path(self, path):
        """
        직선 경로를 찾아서 불필요한 중간점을 제거
        """
        simplified_path = [path[0]]
        for i in range(1, len(path) - 1):
            prev, curr, next_ = np.array(path[i - 1]), np.array(path[i]), np.array(path[i + 1])
            vec1, vec2 = curr - prev, next_ - curr
            if not np.allclose(vec1 / np.linalg.norm(vec1), vec2 / np.linalg.norm(vec2), atol=0.1):
                simplified_path.append(tuple(curr))
        simplified_path.append(path[-1])
        return simplified_path

    def interpolate_path(self, path, resolution=0.5):
        interpolated_path = [path[0]]
        for i in range(1, len(path)):
            p1, p2 = np.array(path[i - 1]), np.array(path[i])
            distance = np.linalg.norm(p2 - p1)
            num_points = max(1, int(distance / resolution))
            for j in range(1, num_points + 1):
                new_point = p1 + (p2 - p1) * (j / num_points)
                interpolated_path.append(tuple(new_point))
        return interpolated_path

    def smooth_path(self, path, smoothing_factor=0.5):
        if len(path) < 3:
            return path
        path = np.array(path)
        x, y = path[:, 0], path[:, 1]
        tck, u = splprep([x, y], s=smoothing_factor)
        u_new = np.linspace(0, 1, len(path) * 5)
        x_new, y_new = splev(u_new, tck)
        return list(zip(x_new, y_new))
