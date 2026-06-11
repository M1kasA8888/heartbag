import numpy as np
import math
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
from typing import List, Tuple, Dict, Optional

class FlightPlanner:
    """航线规划器 - 负责路径规划和避障算法"""
    
    def __init__(self, obstacles: List[List[List[float]]], safe_radius: float):
        """
        初始化航线规划器
        
        Args:
            obstacles: 障碍区列表，每个障碍区是经纬度点列表 [[lat, lon], ...]
            safe_radius: 安全半径（米）
        """
        self.obstacles = obstacles
        self.safe_radius = safe_radius
        self.obstacle_polygons = []
        self.safety_buffers = []
        
        # 创建障碍区多边形和安全缓冲区
        for obs in obstacles:
            if len(obs) >= 3:
                # 创建多边形（注意：Shapely使用 (lon, lat) 顺序）
                polygon = Polygon([(p[1], p[0]) for p in obs])
                self.obstacle_polygons.append(polygon)
                
                # 创建安全缓冲区（将米转换为度）
                buffer_degrees = self.meters_to_degrees(safe_radius)
                safety_buffer = polygon.buffer(buffer_degrees)
                self.safety_buffers.append(safety_buffer)
    
    def meters_to_degrees(self, meters: float, latitude: float = 0) -> float:
        """
        将米转换为度数（近似）
        
        Args:
            meters: 距离（米）
            latitude: 纬度（用于更精确的转换）
        
        Returns:
            对应的度数
        """
        # 1度纬度约等于111公里
        # 1度经度约等于111公里 * cos(latitude)
        return meters / 111000.0
    
    def degrees_to_meters(self, degrees: float, latitude: float = 0) -> float:
        """
        将度数转换为米（近似）
        
        Args:
            degrees: 度数
            latitude: 纬度
        
        Returns:
            对应的米数
        """
        return degrees * 111000.0
    
    def is_point_safe(self, point: List[float]) -> bool:
        """
        检查点是否安全（不在任何障碍区的安全缓冲区内）
        
        Args:
            point: [lat, lon]
        
        Returns:
            是否安全
        """
        shapely_point = Point(point[1], point[0])
        
        for buffer in self.safety_buffers:
            if buffer.contains(shapely_point):
                return False
        
        # 额外检查是否在障碍区内
        for polygon in self.obstacle_polygons:
            if polygon.contains(shapely_point):
                return False
        
        return True
    
    def is_line_safe(self, start: List[float], end: List[float]) -> bool:
        """
        检查线段是否安全（不与任何安全缓冲区相交）
        
        Args:
            start: 起点 [lat, lon]
            end: 终点 [lat, lon]
        
        Returns:
            是否安全
        """
        line = LineString([(start[1], start[0]), (end[1], end[0])])
        
        for buffer in self.safety_buffers:
            if line.intersects(buffer):
                return False
        
        return True
    
    def calculate_distance(self, point1: List[float], point2: List[float]) -> float:
        """
        计算两点间的球面距离（使用Haversine公式）
        
        Args:
            point1: 第一个点 [lat, lon]
            point2: 第二个点 [lat, lon]
        
        Returns:
            距离（米）
        """
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        
        R = 6371000  # 地球平均半径（米）
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def find_waypoints_on_line(self, start: List[float], end: List[float], 
                               interval_meters: float = 100) -> List[List[float]]:
        """
        在线段上生成中间航点
        
        Args:
            start: 起点
            end: 终点
            interval_meters: 航点间隔（米）
        
        Returns:
            航点列表（包含起点和终点）
        """
        distance = self.calculate_distance(start, end)
        num_points = max(2, int(distance / interval_meters) + 1)
        
        waypoints = []
        for i in range(num_points):
            ratio = i / (num_points - 1)
            lat = start[0] + (end[0] - start[0]) * ratio
            lon = start[1] + (end[1] - start[1]) * ratio
            waypoints.append([lat, lon])
        
        return waypoints
    
    def find_alternative_path(self, start: List[float], end: List[float]) -> Optional[List[List[float]]]:
        """
        寻找替代路径（当直线不可行时）
        使用多方向探索策略
        
        Args:
            start: 起点
            end: 终点
        
        Returns:
            绕行路径的航点列表，如果找不到则返回None
        """
        # 计算中点
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        
        # 计算方向向量
        dx = end[1] - start[1]
        dy = end[0] - start[0]
        length = math.sqrt(dx**2 + dy**2)
        
        if length == 0:
            return None
        
        # 单位方向向量
        dx /= length
        dy /= length
        
        # 垂直向量
        perp_x = -dy
        perp_y = dx
        
        # 尝试不同偏移距离和方向
        offsets = [0.002, 0.005, 0.01, 0.015, 0.02]  # 偏移距离（度）
        
        for offset in offsets:
            for direction in [1, -1]:  # 左右两个方向
                # 计算偏移点
                offset_lat = mid_lat + direction * perp_y * offset
                offset_lon = mid_lon + direction * perp_x * offset
                mid_point = [offset_lat, offset_lon]
                
                # 检查中间点是否安全
                if not self.is_point_safe(mid_point):
                    continue
                
                # 检查两段路径是否都安全
                if self.is_line_safe(start, mid_point) and self.is_line_safe(mid_point, end):
                    # 在路径上生成细化航点
                    waypoints = self.find_waypoints_on_line(start, mid_point)
                    waypoints.extend(self.find_waypoints_on_line(mid_point, end)[1:])
                    return waypoints
        
        # 如果单中点不行，尝试多个绕行点
        return self.find_multi_point_path(start, end)
    
    def find_multi_point_path(self, start: List[float], end: List[float]) -> Optional[List[List[float]]]:
        """
        寻找多中点绕行路径（更复杂的避障）
        
        Args:
            start: 起点
            end: 终点
        
        Returns:
            多段绕行路径
        """
        # 计算障碍区的包围盒
        if not self.safety_buffers:
            return None
        
        # 合并所有安全缓冲区
        combined_buffer = unary_union(self.safety_buffers)
        bounds = combined_buffer.bounds  # (minx, miny, maxx, maxy)
        
        # 尝试绕过障碍区的四个角
        corners = [
            [bounds[1], bounds[0]],  # 左下
            [bounds[1], bounds[2]],  # 右下
            [bounds[3], bounds[0]],  # 左上
            [bounds[3], bounds[2]]   # 右上
        ]
        
        best_path = None
        best_distance = float('inf')
        
        for corner in corners:
            # 检查起点到角点和角点到终点的路径
            if self.is_line_safe(start, corner) and self.is_line_safe(corner, end):
                # 检查角点本身是否安全
                if self.is_point_safe(corner):
                    # 生成完整路径
                    path1 = self.find_waypoints_on_line(start, corner)
                    path2 = self.find_waypoints_on_line(corner, end)
                    path = path1 + path2[1:]
                    
                    # 计算总距离
                    total_distance = 0
                    for i in range(len(path) - 1):
                        total_distance += self.calculate_distance(path[i], path[i+1])
                    
                    if total_distance < best_distance:
                        best_distance = total_distance
                        best_path = path
        
        return best_path
    
    def plan_route(self, start: List[float], end: List[float]) -> Optional[Dict]:
        """
        规划完整航线
        
        Args:
            start: 起飞点 [lat, lon]
            end: 目标点 [lat, lon]
        
        Returns:
            航线信息字典，包含航点列表、总距离、预计时间等
        """
        # 检查起终点是否安全
        if not self.is_point_safe(start):
            print(f"警告: 起飞点不安全 {start}")
            return None
        
        if not self.is_point_safe(end):
            print(f"警告: 目标点不安全 {end}")
            return None
        
        # 检查直线路径是否安全
        if self.is_line_safe(start, end):
            # 直线安全，使用直线路径
            waypoints = self.find_waypoints_on_line(start, end)
            total_distance = self.calculate_distance(start, end)
            path_type = "直线路径"
            is_safe = True
        else:
            # 需要绕行
            waypoints = self.find_alternative_path(start, end)
            if waypoints is None:
                print("无法找到安全路径")
                return None
            
            # 计算总距离
            total_distance = 0
            for i in range(len(waypoints) - 1):
                total_distance += self.calculate_distance(waypoints[i], waypoints[i+1])
            
            path_type = "避障绕行路径"
            is_safe = True
        
        # 计算预计飞行时间（假设速度15m/s）
        estimated_time = total_distance / 15
        
        return {
            'waypoints': waypoints,
            'total_distance': total_distance,
            'estimated_time': estimated_time,
            'is_safe': is_safe,
            'path_type': path_type,
            'start_point': start,
            'end_point': end,
            'num_waypoints': len(waypoints)
        }


class DroneSimulator:
    """无人机飞行模拟器 - 负责飞行过程模拟"""
    
    def __init__(self, waypoints: List[List[float]], speed: float = 15):
        """
        初始化飞行模拟器
        
        Args:
            waypoints: 航点列表 [[lat, lon], ...]
            speed: 飞行速度（米/秒）
        """
        self.waypoints = waypoints
        self.speed = speed
        self.current_waypoint_index = 0
        self.current_position = waypoints[0].copy()
        self.segment_progress = 0.0
        self.total_distance = self.calculate_total_distance()
        self.completed_distance = 0.0
        self.is_flying = True
        self.flight_log = []  # 飞行日志
        
        # 预计算各航段距离
        self.segment_distances = []
        for i in range(len(waypoints) - 1):
            self.segment_distances.append(self.calculate_distance(waypoints[i], waypoints[i+1]))
    
    def calculate_distance(self, point1: List[float], point2: List[float]) -> float:
        """计算两点间距离（米）"""
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        
        R = 6371000
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def calculate_total_distance(self) -> float:
        """计算总航程"""
        total = 0
        for i in range(len(self.waypoints) - 1):
            total += self.calculate_distance(self.waypoints[i], self.waypoints[i+1])
        return total
    
    def update(self, delta_time: float = 0.1) -> bool:
        """
        更新无人机位置
        
        Args:
            delta_time: 时间步长（秒）
        
        Returns:
            是否仍在飞行中
        """
        if not self.is_flying:
            return False
        
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.is_flying = False
            return False
        
        # 获取当前目标点
        target = self.waypoints[self.current_waypoint_index + 1]
        current = self.current_position
        
        # 计算到目标点的距离
        distance_to_target = self.calculate_distance(current, target)
        
        # 本帧移动距离
        step_distance = self.speed * delta_time
        
        if distance_to_target <= step_distance:
            # 到达目标点
            self.current_position = target
            self.completed_distance += distance_to_target
            self.current_waypoint_index += 1
            self.segment_progress = 100.0
            
            # 记录日志
            self.flight_log.append({
                'waypoint': self.current_waypoint_index,
                'position': target.copy(),
                'time': sum(self.segment_distances[:self.current_waypoint_index]) / self.speed
            })
        else:
            # 向目标点移动
            # 计算移动方向
            dx = target[1] - current[1]
            dy = target[0] - current[0]
            distance = distance_to_target
            
            # 归一化方向向量
            if distance > 0:
                fraction = step_distance / distance
                new_lat = current[0] + dy * fraction
                new_lon = current[1] + dx * fraction
                self.current_position = [new_lat, new_lon]
                self.completed_distance += step_distance
                
                # 更新当前航段进度
                remaining = self.calculate_distance(self.current_position, target)
                self.segment_progress = (1 - remaining / distance_to_target) * 100
        
        return True
    
    def get_status(self) -> Dict:
        """获取当前飞行状态"""
        total_progress = (self.completed_distance / self.total_distance) * 100 if self.total_distance > 0 else 0
        remaining_distance = self.total_distance - self.completed_distance
        
        # 计算当前航段剩余距离
        if self.current_waypoint_index < len(self.waypoints) - 1:
            current_segment_remaining = self.calculate_distance(
                self.current_position,
                self.waypoints[self.current_waypoint_index + 1]
            )
        else:
            current_segment_remaining = 0
        
        return {
            'position': self.current_position,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'completed_distance': self.completed_distance,
            'remaining_distance': remaining_distance,
            'progress': total_progress,
            'segment_progress': self.segment_progress,
            'current_segment_remaining': current_segment_remaining,
            'is_flying': self.is_flying
        }
    
    def check_safety(self, obstacles: List = None) -> bool:
        """
        检查当前位置是否安全（碰撞检测）
        
        Args:
            obstacles: 障碍区列表，用于碰撞检测
        
        Returns:
            是否安全
        """
        # 简化版：始终返回True
        # 实际应用中需要检查当前位置是否在障碍区内
        return True
    
    def get_flight_summary(self) -> Dict:
        """获取飞行总结"""
        return {
            'total_distance': self.total_distance,
            'completed_distance': self.completed_distance,
            'waypoints_count': len(self.waypoints),
            'completion_rate': (self.completed_distance / self.total_distance * 100) if self.total_distance > 0 else 0,
            'total_time': self.completed_distance / self.speed if self.speed > 0 else 0,
            'average_speed': self.speed
        }


def test_flight_planner():
    """测试航线规划器"""
    # 创建测试障碍区
    obstacles = [
        [[39.90, 116.40], [39.91, 116.41], [39.90, 116.42], [39.89, 116.41]]
    ]
    
    # 创建规划器
    planner = FlightPlanner(obstacles, safe_radius=50)
    
    # 测试点
    start = [39.88, 116.38]
    end = [39.92, 116.44]
    
    # 规划航线
    route = planner.plan_route(start, end)
    
    if route:
        print(f"路径类型: {route['path_type']}")
        print(f"总距离: {route['total_distance']:.2f} 米")
        print(f"预计时间: {route['estimated_time']:.2f} 秒")
        print(f"航点数量: {route['num_waypoints']}")
    else:
        print("无法规划安全路径")


if __name__ == "__main__":
    test_flight_planner()
