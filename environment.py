"""
environment.py - 精简环境管理系统
保留核心功能，移除过度设计
"""

import numpy as np
import math
import random
import time
import threading
import json
from typing import Dict, List, Tuple, Optional, Any
from collections import defaultdict, OrderedDict
from dataclasses import dataclass
from enum import Enum
from PyQt5.QtGui import QColor
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EnvironmentState(Enum):
    """环境状态"""
    UNINITIALIZED = "uninitialized"
    READY = "ready"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"

@dataclass
class VehicleInfo:
    """车辆信息"""
    vehicle_id: str
    position: Tuple[float, float, float]
    initial_position: Tuple[float, float, float]
    goal: Optional[Tuple[float, float, float]] = None
    
    # 基本属性
    vehicle_type: str = "dump_truck"
    max_load: float = 100
    current_load: float = 0
    speed: float = 1.0
    
    # 状态信息
    status: str = 'idle'
    path: Optional[List] = None
    path_index: int = 0
    progress: float = 0.0
    
    # 任务信息
    completed_cycles: int = 0
    path_structure: Dict = None
    
    # 性能统计
    total_distance: float = 0
    total_time: float = 0
    
    # 显示属性
    color: Optional[QColor] = None
    
    def __post_init__(self):
        if self.color is None:
            self.color = QColor(
                random.randint(100, 255), 
                random.randint(100, 255), 
                random.randint(100, 255)
            )
        
        if self.path_structure is None:
            self.path_structure = {}
    
    # 字典访问支持
    def __getitem__(self, key):
        if hasattr(self, key):
            return getattr(self, key)
        
        alias_map = {
            'load': 'current_load',
            'type': 'vehicle_type',
            'id': 'vehicle_id'
        }
        
        if key in alias_map:
            return getattr(self, alias_map[key])
        
        raise KeyError(f"'{key}' not found")
    
    def __setitem__(self, key, value):
        alias_map = {
            'load': 'current_load',
            'type': 'vehicle_type',
            'id': 'vehicle_id'
        }
        
        actual_key = alias_map.get(key, key)
        
        if hasattr(self, actual_key):
            setattr(self, actual_key, value)
        else:
            setattr(self, key, value)
    
    def __contains__(self, key):
        alias_map = {
            'load': 'current_load',
            'type': 'vehicle_type',
            'id': 'vehicle_id'
        }
        return hasattr(self, key) or key in alias_map
    
    def get(self, key, default=None):
        try:
            return self[key]
        except KeyError:
            return default

class OptimizedOpenPitMineEnv:
    """精简环境管理器"""
    
    def __init__(self, config=None):
        # 基本配置
        self.width = 500
        self.height = 500
        self.grid_resolution = 1.0
        
        if config:
            self.width = getattr(config, 'width', 500)
            self.height = getattr(config, 'height', 500)
            self.grid_resolution = getattr(config, 'grid_resolution', 1.0)
        
        # 状态管理
        self.state = EnvironmentState.UNINITIALIZED
        self.state_lock = threading.Lock()
        
        # 地图数据
        self.grid = np.zeros((self.width, self.height), dtype=np.uint8)
        self.obstacle_points = []
        
        # 关键点
        self.loading_points = []
        self.unloading_points = []
        self.parking_areas = []
        
        # 车辆管理
        self.vehicles = OrderedDict()
        
        # 组件引用
        self.components = {}
        
        # 仿真状态
        self.current_time = 0.0
        self.time_step = 0.5
        self.running = False
        self.paused = False
        
        # 基本统计
        self.stats = {
            'total_vehicles': 0,
            'active_vehicles': 0
        }
        
        # 事件回调
        self.changed_callback = None
        
        logger.info(f"环境初始化: {self.width}x{self.height}")
        self.state = EnvironmentState.READY
    
    # ==================== 状态管理 ====================
    
    def set_state(self, new_state: EnvironmentState):
        """设置环境状态"""
        with self.state_lock:
            self.state = new_state
    
    def get_state(self) -> EnvironmentState:
        """获取环境状态"""
        return self.state
    
    def is_ready(self) -> bool:
        """检查是否就绪"""
        return self.state in [EnvironmentState.READY, EnvironmentState.RUNNING, EnvironmentState.PAUSED]
    
    def is_running(self) -> bool:
        """检查是否运行中"""
        return self.state == EnvironmentState.RUNNING and not self.paused
    
    # ==================== 组件管理 ====================
    
    def register_component(self, name: str, component: Any):
        """注册组件"""
        self.components[name] = component
        logger.info(f"注册组件: {name}")
    
    def get_component(self, name: str) -> Optional[Any]:
        """获取组件"""
        return self.components.get(name)
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.register_component('backbone_network', backbone_network)
    
    def set_vehicle_scheduler(self, scheduler):
        """设置调度器"""
        self.register_component('vehicle_scheduler', scheduler)
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.register_component('traffic_manager', traffic_manager)
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.register_component('path_planner', path_planner)
    
    # ==================== 地图管理 ====================
    
    def add_obstacle_point(self, x: int, y: int) -> bool:
        """添加障碍物点"""
        try:
            x = max(0, min(int(x), self.width - 1))
            y = max(0, min(int(y), self.height - 1))
            
            self.grid[x, y] = 1
            
            if (x, y) not in self.obstacle_points:
                self.obstacle_points.append((x, y))
            
            self._trigger_changed()
            return True
            
        except Exception as e:
            logger.error(f"添加障碍物失败: {e}")
            return False
    
    def add_obstacle(self, x: int, y: int, width: int = 1, height: int = 1) -> bool:
        """添加矩形障碍物"""
        try:
            success_count = 0
            total_count = width * height
            
            for i in range(x, min(x + width, self.width)):
                for j in range(y, min(y + height, self.height)):
                    if self.add_obstacle_point(i, j):
                        success_count += 1
            
            return success_count > 0
            
        except Exception as e:
            logger.error(f"添加障碍物区域失败: {e}")
            return False
    
    def remove_obstacle_point(self, x: int, y: int) -> bool:
        """移除障碍物点"""
        try:
            x = max(0, min(int(x), self.width - 1))
            y = max(0, min(int(y), self.height - 1))
            
            if self.grid[x, y] == 1:
                self.grid[x, y] = 0
                
                if (x, y) in self.obstacle_points:
                    self.obstacle_points.remove((x, y))
                
                self._trigger_changed()
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"移除障碍物失败: {e}")
            return False
    
    def clear_obstacles(self):
        """清除所有障碍物"""
        self.grid.fill(0)
        self.obstacle_points.clear()
        self._trigger_changed()
        logger.info("已清除所有障碍物")
    
    # ==================== 关键点管理 ====================
    
    def add_loading_point(self, position: Tuple[float, float, float], capacity: int = 1) -> int:
        """添加装载点"""
        try:
            x, y = float(position[0]), float(position[1])
            theta = float(position[2]) if len(position) > 2 else 0.0
            
            if not (0 <= x < self.width and 0 <= y < self.height):
                raise ValueError(f"坐标超出范围: ({x}, {y})")
            
            point = (x, y, theta)
            self.loading_points.append(point)
            index = len(self.loading_points) - 1
            
            self._trigger_changed()
            logger.info(f"添加装载点 {index}: ({x:.1f}, {y:.1f})")
            return index
            
        except Exception as e:
            logger.error(f"添加装载点失败: {e}")
            return -1
    
    def add_unloading_point(self, position: Tuple[float, float, float], capacity: int = 1) -> int:
        """添加卸载点"""
        try:
            x, y = float(position[0]), float(position[1])
            theta = float(position[2]) if len(position) > 2 else 0.0
            
            if not (0 <= x < self.width and 0 <= y < self.height):
                raise ValueError(f"坐标超出范围: ({x}, {y})")
            
            point = (x, y, theta)
            self.unloading_points.append(point)
            index = len(self.unloading_points) - 1
            
            self._trigger_changed()
            logger.info(f"添加卸载点 {index}: ({x:.1f}, {y:.1f})")
            return index
            
        except Exception as e:
            logger.error(f"添加卸载点失败: {e}")
            return -1
    
    def add_parking_area(self, position: Tuple[float, float, float], capacity: int = 5) -> int:
        """添加停车区"""
        try:
            x, y = float(position[0]), float(position[1])
            theta = float(position[2]) if len(position) > 2 else 0.0
            
            if not (0 <= x < self.width and 0 <= y < self.height):
                raise ValueError(f"坐标超出范围: ({x}, {y})")
            
            point = (x, y, theta)
            self.parking_areas.append(point)
            index = len(self.parking_areas) - 1
            
            self._trigger_changed()
            logger.info(f"添加停车区 {index}: ({x:.1f}, {y:.1f})")
            return index
            
        except Exception as e:
            logger.error(f"添加停车区失败: {e}")
            return -1
    
    # ==================== 车辆管理 ====================
    
    def add_vehicle(self, vehicle_id: str, position: Tuple[float, float, float], 
                   goal: Tuple[float, float, float] = None, 
                   vehicle_type: str = "dump_truck", max_load: float = 100) -> bool:
        """添加车辆"""
        try:
            if vehicle_id in self.vehicles:
                raise ValueError(f"车辆ID已存在: {vehicle_id}")
            
            x, y = float(position[0]), float(position[1])
            theta = float(position[2]) if len(position) > 2 else 0.0
            
            if not (0 <= x < self.width and 0 <= y < self.height):
                raise ValueError(f"车辆位置超出范围: ({x}, {y})")
            
            vehicle_info = VehicleInfo(
                vehicle_id=vehicle_id,
                position=(x, y, theta),
                initial_position=(x, y, theta),
                goal=goal,
                vehicle_type=vehicle_type,
                max_load=max_load
            )
            
            self.vehicles[vehicle_id] = vehicle_info
            self.stats['total_vehicles'] = len(self.vehicles)
            
            self._trigger_changed()
            logger.info(f"添加车辆 {vehicle_id}: ({x:.1f}, {y:.1f})")
            return True
            
        except Exception as e:
            logger.error(f"添加车辆失败 {vehicle_id}: {e}")
            return False
    
    def remove_vehicle(self, vehicle_id: str) -> bool:
        """移除车辆"""
        try:
            if vehicle_id not in self.vehicles:
                return False
            
            del self.vehicles[vehicle_id]
            self.stats['total_vehicles'] = len(self.vehicles)
            
            self._trigger_changed()
            logger.info(f"移除车辆 {vehicle_id}")
            return True
            
        except Exception as e:
            logger.error(f"移除车辆失败 {vehicle_id}: {e}")
            return False
    
    def update_vehicle_position(self, vehicle_id: str, position: Tuple[float, float, float]) -> bool:
        """更新车辆位置"""
        try:
            if vehicle_id not in self.vehicles:
                return False
            
            x, y, theta = float(position[0]), float(position[1]), float(position[2])
            
            vehicle_info = self.vehicles[vehicle_id]
            old_position = vehicle_info.position
            vehicle_info.position = (x, y, theta)
            
            # 计算移动距离
            if old_position:
                distance = math.sqrt(
                    (x - old_position[0])**2 + (y - old_position[1])**2
                )
                vehicle_info.total_distance += distance
            
            return True
            
        except Exception as e:
            logger.error(f"更新车辆位置失败 {vehicle_id}: {e}")
            return False
    
    def get_vehicle_info(self, vehicle_id: str) -> Optional[VehicleInfo]:
        """获取车辆信息"""
        return self.vehicles.get(vehicle_id)
    
    # ==================== 碰撞检测 ====================
    
    def check_collision(self, position: Tuple[float, float, float], 
                       vehicle_dim: Tuple[float, float] = (6, 3),
                       exclude_vehicle: str = None) -> bool:
        """简化碰撞检测"""
        try:
            x, y, theta = float(position[0]), float(position[1]), float(position[2])
            
            # 边界检查
            if x < 0 or x >= self.width or y < 0 or y >= self.height:
                return True
            
            # 中心点障碍物检查
            ix, iy = int(x), int(y)
            if 0 <= ix < self.width and 0 <= iy < self.height:
                if self.grid[ix, iy] == 1:
                    return True
            
            # 与其他车辆的碰撞检测
            safety_radius = 5.0
            for vehicle_id, vehicle_info in self.vehicles.items():
                if vehicle_id == exclude_vehicle:
                    continue
                
                other_pos = vehicle_info.position
                distance = math.sqrt(
                    (x - other_pos[0])**2 + (y - other_pos[1])**2
                )
                
                if distance < safety_radius:
                    return True
            
            return False
            
        except Exception:
            return True
    
    # ==================== 仿真控制 ====================
    
    def start(self):
        """开始仿真"""
        if self.state in [EnvironmentState.READY, EnvironmentState.PAUSED]:
            self.running = True
            self.paused = False
            self.set_state(EnvironmentState.RUNNING)
            logger.info("仿真已开始")
    
    def pause(self):
        """暂停仿真"""
        if self.state == EnvironmentState.RUNNING:
            self.paused = True
            self.set_state(EnvironmentState.PAUSED)
            logger.info("仿真已暂停")
    
    def resume(self):
        """恢复仿真"""
        if self.state == EnvironmentState.PAUSED:
            self.paused = False
            self.set_state(EnvironmentState.RUNNING)
            logger.info("仿真已恢复")
    
    def stop(self):
        """停止仿真"""
        self.running = False
        self.paused = False
        self.set_state(EnvironmentState.READY)
        logger.info("仿真已停止")
    
    def reset(self):
        """重置环境"""
        try:
            self.stop()
            
            # 重置车辆状态
            for vehicle_info in self.vehicles.values():
                vehicle_info.position = vehicle_info.initial_position
                vehicle_info.current_load = 0
                vehicle_info.status = 'idle'
                vehicle_info.path = None
                vehicle_info.path_index = 0
                vehicle_info.progress = 0.0
                vehicle_info.completed_cycles = 0
                vehicle_info.total_distance = 0
                vehicle_info.total_time = 0
            
            self.current_time = 0.0
            
            self._trigger_changed()
            logger.info("环境已重置")
            return True
            
        except Exception as e:
            logger.error(f"环境重置失败: {e}")
            self.set_state(EnvironmentState.ERROR)
            return False
    
    def update(self, time_delta: float = None):
        """更新环境"""
        try:
            if not self.is_running():
                return False
            
            if time_delta is None:
                time_delta = self.time_step
            
            self.current_time += time_delta
            
            # 更新活跃车辆统计
            active_count = len([v for v in self.vehicles.values() 
                              if v.status != 'idle'])
            self.stats['active_vehicles'] = active_count
            
            # 更新车辆时间
            for vehicle_info in self.vehicles.values():
                if vehicle_info.status != 'idle':
                    vehicle_info.total_time += time_delta
            
            return True
            
        except Exception as e:
            logger.error(f"环境更新失败: {e}")
            return False
    
    # ==================== 文件操作 ====================
    
    def load_from_file(self, filename: str) -> bool:
        """从文件加载环境"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # 更新尺寸
            if "width" in data:
                self.width = int(data["width"])
            if "height" in data:
                self.height = int(data["height"])
            if "grid_resolution" in data:
                self.grid_resolution = float(data["grid_resolution"])
            
            # 重新初始化网格
            self.grid = np.zeros((self.width, self.height), dtype=np.uint8)
            
            # 加载障碍物
            self.obstacle_points = []
            for obstacle in data.get("obstacles", []):
                if isinstance(obstacle, dict):
                    x = obstacle.get("x", 0)
                    y = obstacle.get("y", 0)
                    if "width" in obstacle and "height" in obstacle:
                        self.add_obstacle(x, y, obstacle["width"], obstacle["height"])
                    else:
                        self.add_obstacle_point(x, y)
            
            # 加载装载点
            self.loading_points = []
            for point in data.get("loading_points", []):
                if isinstance(point, dict):
                    x, y = point.get("x", 0), point.get("y", 0)
                    theta = point.get("theta", 0.0)
                    self.add_loading_point((x, y, theta))
                elif isinstance(point, list) and len(point) >= 2:
                    row, col = point[0], point[1]
                    theta = 0.0 if len(point) <= 2 else point[2]
                    self.add_loading_point((col, row, theta))
            
            # 加载卸载点
            self.unloading_points = []
            for point in data.get("unloading_points", []):
                if isinstance(point, dict):
                    x, y = point.get("x", 0), point.get("y", 0)
                    theta = point.get("theta", 0.0)
                    self.add_unloading_point((x, y, theta))
                elif isinstance(point, list) and len(point) >= 2:
                    row, col = point[0], point[1]
                    theta = 0.0 if len(point) <= 2 else point[2]
                    self.add_unloading_point((col, row, theta))
            
            # 加载停车区
            self.parking_areas = []
            for point in data.get("parking_areas", []):
                if isinstance(point, dict):
                    # 处理字典格式
                    x, y = point.get("x", 0), point.get("y", 0)
                    theta = point.get("theta", 0.0)
                    self.add_parking_area((x, y, theta))
                elif isinstance(point, list) and len(point) >= 2:
                    # 处理列表格式 [row, col, theta, capacity]
                    row, col = point[0], point[1]
                    theta = 0.0 if len(point) <= 2 else point[2]
                    capacity = 5 if len(point) <= 3 else point[3]  # 第4个元素是容量
                    self.add_parking_area((col, row, theta), capacity)

            print(f"✓ 加载停车区: {len(self.parking_areas)} 个")
                            
            # 加载车辆
            self.vehicles = OrderedDict()
            
            # 处理vehicles_info字段
            for vehicle in data.get("vehicles_info", []):
                if isinstance(vehicle, dict):
                    vehicle_id = str(vehicle.get("id", f"v_{len(self.vehicles) + 1}"))
                    x = float(vehicle.get("x", 0))
                    y = float(vehicle.get("y", 0))
                    theta = float(vehicle.get("theta", 0.0))
                    v_type = vehicle.get("type", "dump_truck")
                    max_load = float(vehicle.get("max_load", 100))
                    
                    if self.add_vehicle(vehicle_id, (x, y, theta), None, v_type, max_load):
                        print(f"✓ 成功添加车辆: {vehicle_id}")
            
            # 处理vehicles字段
            for vehicle in data.get("vehicles", []):
                if isinstance(vehicle, dict):
                    vehicle_id = str(vehicle.get("id", f"v_{len(self.vehicles) + 1}"))
                    x = vehicle.get("x", 0)
                    y = vehicle.get("y", 0)
                    theta = vehicle.get("theta", 0.0)
                    v_type = vehicle.get("type", "dump_truck")
                    max_load = vehicle.get("max_load", 100)
                    
                    if self.add_vehicle(vehicle_id, (x, y, theta), None, v_type, max_load):
                        vehicle_info = self.vehicles[vehicle_id]
                        vehicle_info.status = vehicle.get("status", "idle")
                        vehicle_info.current_load = vehicle.get("load", 0)
                        vehicle_info.completed_cycles = vehicle.get("completed_cycles", 0)
            
            # 设置环境状态
            self.current_time = data.get("current_time", 0.0)
            self.running = data.get("running", False)
            self.paused = data.get("paused", False)
            
            print(f"✓ 环境加载成功: {len(self.vehicles)} 个车辆")
            return True
            
        except Exception as e:
            logger.error(f"加载环境失败: {e}")
            return False
    
    def save_to_file(self, filename: str) -> bool:
        """保存环境到文件"""
        try:
            data = {
                "width": self.width,
                "height": self.height,
                "grid_resolution": self.grid_resolution,
                "current_time": self.current_time,
                "running": self.running,
                "paused": self.paused,
                
                # 障碍物
                "obstacles": [
                    {"x": x, "y": y} for x, y in self.obstacle_points
                ],
                
                # 关键点
                "loading_points": [
                    {"x": p[0], "y": p[1], "theta": p[2]} 
                    for p in self.loading_points
                ],
                "unloading_points": [
                    {"x": p[0], "y": p[1], "theta": p[2]} 
                    for p in self.unloading_points
                ],
                "parking_areas": [
                    {"x": p[0], "y": p[1], "theta": p[2]} 
                    for p in self.parking_areas
                ],
                
                # 车辆
                "vehicles": [
                    {
                        "id": vehicle_id,
                        "x": vehicle_info.position[0],
                        "y": vehicle_info.position[1],
                        "theta": vehicle_info.position[2],
                        "type": vehicle_info.vehicle_type,
                        "max_load": vehicle_info.max_load,
                        "load": vehicle_info.current_load,
                        "status": vehicle_info.status,
                        "completed_cycles": vehicle_info.completed_cycles
                    }
                    for vehicle_id, vehicle_info in self.vehicles.items()
                ]
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"环境已保存到: {filename}")
            return True
            
        except Exception as e:
            logger.error(f"保存环境失败: {e}")
            return False
    
    # ==================== 工具方法 ====================
    
    def _trigger_changed(self):
        """触发变更回调"""
        if self.changed_callback:
            try:
                self.changed_callback(self)
            except Exception:
                pass
    
    def set_changed_callback(self, callback):
        """设置变更回调"""
        self.changed_callback = callback
    
    def get_environment_summary(self):
        """获取环境摘要"""
        return {
            'dimensions': f"{self.width}x{self.height}",
            'vehicles': len(self.vehicles),
            'loading_points': len(self.loading_points),
            'unloading_points': len(self.unloading_points),
            'parking_areas': len(self.parking_areas),
            'obstacles': len(self.obstacle_points),
            'state': self.state.value,
            'components': len(self.components)
        }
    
    def shutdown(self):
        """关闭环境"""
        try:
            self.stop()
            logger.info("环境已关闭")
        except Exception as e:
            logger.error(f"环境关闭失败: {e}")

# 向后兼容性
OpenPitMineEnv = OptimizedOpenPitMineEnv