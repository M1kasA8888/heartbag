import numpy as np
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
import math

class FlightPlanner:
    def __init__(self, obstacles, safe_radius):
        self.obstacles = obstacles
        self.safe_radius = safe_radius
        self.obstacle_polygons = []
        
        # 创建障碍区多边形
        for obs in obstacles:
            if len(obs) >= 3:
                polygon = Polygon(obs)
                # 膨胀障碍区（安全半径）
                expanded = polygon.buffer(safe_radius / 111000)  # 粗略转换米到度
                self.obstacle_polygons.append(expanded)
    
    def is_point_safe(self, point):
        """检查点是否安全"""
        p = Point(point[1], point[0])  # 注意经纬度顺序
        for obstacle in self.obstacle_polygons:
            if obstacle.contains(p) or obstacle.distance(p) < 0.0001:
                return False
        return True
    
    def plan_route(self, start, end):
        """规划航线（简化的A*算法）"""
        if not self.is_point_safe(start) or not self.is_point_safe(end):
            return None
        
        # 简化版：如果直线安全，直接返回直线路径
        line = LineString([(start[1], start[0]), (end[1], end[0])])
        is_safe = True
        
        # 检查直线是否与障碍区相交
        for obstacle in self.obstacle_polygons:
            if line.intersects(obstacle):
                is_safe = False
                break
        
        if is_safe:
            # 计算距离
            distance = self.calculate_distance(start, end)
            waypoints = [start, end]
            
            return {
                'waypoints': waypoints,
                'total_distance': distance,
                'estimated_time': distance / 15,  # 假设速度15m/s
                'is_safe': True
            }
        else:
            # 使用简单的绕行策略：生成中间点
            waypoints = self.find_alternative_path(start, end)
            if waypoints:
                total_distance = 0
                for i in range(len(waypoints) - 1):
                    total_distance += self.calculate_distance(waypoints[i], waypoints[i+1])
                
                return {
                    'waypoints': waypoints,
                    'total_distance': total_distance,
                    'estimated_time': total_distance / 15,
                    'is_safe': True
                }
        
        return None
    
    def find_alternative_path(self, start, end):
        """寻找替代路径（简单绕行）"""
        # 计算中点
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        
        # 尝试不同的偏移方向
        offsets = [0.01, -0.01, 0.005, -0.005, 0.002, -0.002]
        
        for offset_lat in offsets:
            for offset_lon in offsets:
                mid_point = [mid_lat + offset_lat, mid_lon + offset_lon]
                if self.is_point_safe(mid_point):
                    # 检查两段路径是否都安全
                    path1 = LineString([(start[1], start[0]), (mid_point[1], mid_point[0])])
                    path2 = LineString([(mid_point[1], mid_point[0]), (end[1], end[0])])
                    
                    safe1 = True
                    safe2 = True
                    
                    for obstacle in self.obstacle_polygons:
                        if path1.intersects(obstacle):
                            safe1 = False
                        if path2.intersects(obstacle):
                            safe2 = False
                    
                    if safe1 and safe2:
                        return [start, mid_point, end]
        
        return None
    
    def calculate_distance(self, point1, point2):
        """计算两点间距离（米）"""
        # 简化的经纬度距离计算
        lat1, lon1 = point1
        lat2, lon2 = point2
        
        R = 6371000  # 地球半径（米）
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c


class DroneSimulator:
    def __init__(self, waypoints, speed):
        self.waypoints = waypoints
        self.speed = speed
        self.current_waypoint_index = 0
        self.current_position = waypoints[0].copy()
        self.segment_progress = 0
        self.total_distance = self.calculate_total_distance()
        self.completed_distance = 0
    
    def calculate_total_distance(self):
        """计算总航程"""
        total = 0
        for i in range(len(self.waypoints) - 1):
            total += self.calculate_distance(self.waypoints[i], self.waypoints[i+1])
        return total
    
    def calculate_distance(self, point1, point2):
        """计算两点间距离"""
        R = 6371000
        lat1, lon1 = point1
        lat2, lon2 = point2
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def update(self):
        """更新无人机位置"""
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            return False
        
        # 获取当前目标点
        target = self.waypoints[self.current_waypoint_index + 1]
        current = self.current_position
        
        # 计算到目标点的距离
        distance_to_target = self.calculate_distance(current, target)
        
        # 每帧移动的距离（假设每秒更新10次）
        step_distance = self.speed / 10
        
        if distance_to_target <= step_distance:
            # 到达目标点
            self.current_position = target
            self.completed_distance += distance_to_target
            self.current_waypoint_index += 1
            self.segment_progress = 0
        else:
            # 向目标点移动
            # 计算方向
            lat1, lon1 = current
            lat2, lon2 = target
            
            # 计算角度
            delta_lat = lat2 - lat1
            delta_lon = lon2 - lon1
            distance = self.calculate_distance(current, target)
            
            if distance > 0:
                # 移动
                fraction = step_distance / distance
                new_lat = lat1 + delta_lat * fraction
                new_lon = lon1 + delta_lon * fraction
                self.current_position = [new_lat, new_lon]
                self.completed_distance += step_distance
                
                # 更新当前航段进度
                remaining = self.calculate_distance(self.current_position, target)
                self.segment_progress = (1 - remaining / distance_to_target) * 100
        
        return True
    
    def get_status(self):
        """获取当前状态"""
        total_progress = (self.completed_distance / self.total_distance) * 100 if self.total_distance > 0 else 0
        remaining_distance = self.total_distance - self.completed_distance
        
        # 检查安全性
        is_safe = True  # 实际应用中需要检查当前位置
        
        return {
            'position': self.current_position,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'completed_distance': self.completed_distance,
            'remaining_distance': remaining_distance,
            'progress': total_progress,
            'segment_progress': self.segment_progress,
            'is_safe': is_safe
        }
