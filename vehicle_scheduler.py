"""
vehicle_scheduler.py - 精简版车辆调度器
专注任务分配和状态管理，移除复杂性能分析
"""

import math
import time
import threading
from typing import List, Dict, Tuple, Optional, Any
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass
from enum import Enum

class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

class VehicleStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    LOADING = "loading"
    UNLOADING = "unloading"
    PLANNING = "planning"

@dataclass
class SimpleTask:
    """简化任务结构"""
    task_id: str
    task_type: str
    start: Tuple[float, float, float]
    goal: Tuple[float, float, float]
    status: TaskStatus = TaskStatus.PENDING
    
    # 基本属性
    assigned_vehicle: Optional[str] = None
    assignment_time: float = 0
    start_time: float = 0
    completion_time: float = 0
    
    # 路径信息
    path: Optional[List] = None
    path_structure: Dict = None
    
    def update_progress(self, progress: float):
        """更新进度"""
        if progress >= 1.0:
            self.status = TaskStatus.COMPLETED
            self.completion_time = time.time()

@dataclass
class VehicleState:
    """简化车辆状态"""
    vehicle_id: str
    status: VehicleStatus = VehicleStatus.IDLE
    position: Tuple[float, float, float] = (0, 0, 0)
    
    # 基本属性
    max_load: float = 100
    current_load: float = 0
    speed: float = 1.0
    
    # 任务相关
    current_task: Optional[str] = None
    completed_cycles: int = 0
    
    # 简单统计
    total_distance: float = 0
    total_time: float = 0

class SimplifiedVehicleScheduler:
    """精简车辆调度器 - 专注核心功能"""
    
    def __init__(self, env, path_planner=None, backbone_network=None, traffic_manager=None):
        # 核心组件
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 数据存储
        self.tasks = OrderedDict()  # {task_id: SimpleTask}
        self.vehicle_states = {}  # {vehicle_id: VehicleState}
        self.mission_templates = {}  # {template_id: mission_config}
        
        # 任务管理
        self.task_counter = 0
        self.active_assignments = defaultdict(list)  # {vehicle_id: [task_ids]}
        
        # 状态锁
        self.state_lock = threading.RLock()
        
        # 基本统计
        self.stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'total_distance': 0
        }
        
        print("初始化精简版车辆调度器")
    
    def initialize_vehicles(self):
        """初始化车辆状态"""
        with self.state_lock:
            for vehicle_id, vehicle_data in self.env.vehicles.items():
                position = vehicle_data.get('position', (0, 0, 0))
                max_load = vehicle_data.get('max_load', 100)
                
                self.vehicle_states[vehicle_id] = VehicleState(
                    vehicle_id=vehicle_id,
                    position=position,
                    max_load=max_load,
                    current_load=vehicle_data.get('load', 0)
                )
                
                self.active_assignments[vehicle_id] = []
        
        print(f"初始化了 {len(self.vehicle_states)} 个车辆状态")
    
    def create_enhanced_mission_template(self, template_id: str, 
                                       loading_point_id: int = None, 
                                       unloading_point_id: int = None) -> bool:
        """创建任务模板"""
        if not self.env.loading_points or not self.env.unloading_points:
            return False
        
        # 选择装载点和卸载点
        if loading_point_id is None:
            loading_point_id = 0
        if unloading_point_id is None:
            unloading_point_id = 0
        
        # 验证有效性
        if (loading_point_id >= len(self.env.loading_points) or 
            unloading_point_id >= len(self.env.unloading_points)):
            return False
        
        loading_point = self.env.loading_points[loading_point_id]
        unloading_point = self.env.unloading_points[unloading_point_id]
        
        # 创建模板
        template = {
            'loading_point_id': loading_point_id,
            'unloading_point_id': unloading_point_id,
            'loading_position': loading_point,
            'unloading_position': unloading_point,
            'tasks': [
                {
                    'task_type': 'to_loading',
                    'goal': loading_point,
                    'estimated_duration': 180
                },
                {
                    'task_type': 'to_unloading', 
                    'goal': unloading_point,
                    'estimated_duration': 150
                },
                {
                    'task_type': 'to_initial',
                    'goal': None,  # 将在分配时确定
                    'estimated_duration': 120
                }
            ]
        }
        
        self.mission_templates[template_id] = template
        print(f"创建任务模板 {template_id}: L{loading_point_id} -> U{unloading_point_id}")
        return True
    
    def assign_mission_intelligently(self, vehicle_id: str = None, 
                                   template_id: str = "default") -> bool:
        """智能任务分配"""
        with self.state_lock:
            # 选择车辆
            if vehicle_id is None:
                vehicle_id = self._select_available_vehicle()
                if not vehicle_id:
                    return False
            
            if vehicle_id not in self.vehicle_states:
                return False
            
            # 创建默认模板
            if template_id not in self.mission_templates:
                if not self.create_enhanced_mission_template(template_id):
                    return False
            
            template = self.mission_templates[template_id]
            vehicle_state = self.vehicle_states[vehicle_id]
            
            # 生成任务序列
            created_tasks = []
            current_position = vehicle_state.position
            
            for task_template in template['tasks']:
                task_id = f"task_{self.task_counter}"
                self.task_counter += 1
                
                # 确定目标位置
                if task_template['goal'] is None:
                    goal = vehicle_state.position  # 返回起始位置
                else:
                    goal = task_template['goal']
                
                # 创建任务
                task = SimpleTask(
                    task_id=task_id,
                    task_type=task_template['task_type'],
                    start=current_position,
                    goal=goal
                )
                
                self.tasks[task_id] = task
                created_tasks.append(task_id)
                current_position = goal
            
            # 分配给车辆
            self.active_assignments[vehicle_id].extend(created_tasks)
            self.stats['total_tasks'] += len(created_tasks)
            
            # 开始第一个任务
            if vehicle_state.status == VehicleStatus.IDLE:
                self._start_next_task(vehicle_id)
            
            print(f"为车辆 {vehicle_id} 分配了 {len(created_tasks)} 个任务")
            return True
    
    def _select_available_vehicle(self) -> Optional[str]:
        """选择可用车辆"""
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if vehicle_state.status == VehicleStatus.IDLE:
                return vehicle_id
        return None
    
    def _start_next_task(self, vehicle_id: str) -> bool:
        """开始下一个任务"""
        if vehicle_id not in self.active_assignments:
            return False
        
        assignments = self.active_assignments[vehicle_id]
        if not assignments:
            return False
        
        # 找到第一个待处理任务
        next_task_id = None
        for task_id in assignments:
            if task_id in self.tasks and self.tasks[task_id].status == TaskStatus.PENDING:
                next_task_id = task_id
                break
        
        if not next_task_id:
            return False
        
        return self._start_task_execution(next_task_id, vehicle_id)
    
    def _start_task_execution(self, task_id: str, vehicle_id: str) -> bool:
        """开始任务执行"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        vehicle_state = self.vehicle_states[vehicle_id]
        
        # 更新任务状态
        task.status = TaskStatus.IN_PROGRESS
        task.assigned_vehicle = vehicle_id
        task.assignment_time = time.time()
        task.start_time = time.time()
        
        # 更新车辆状态
        vehicle_state.status = VehicleStatus.PLANNING
        vehicle_state.current_task = task_id
        
        # 同步到环境
        if vehicle_id in self.env.vehicles:
            self.env.vehicles[vehicle_id]['status'] = 'planning'
        
        # 执行路径规划
        return self._plan_and_start_task(task, vehicle_state)
    
    def _plan_and_start_task(self, task: SimpleTask, 
                            vehicle_state: VehicleState) -> bool:
        """规划并开始任务"""
        if not self.path_planner:
            print(f"无路径规划器，任务 {task.task_id} 失败")
            return False
        
        try:
            # 路径规划
            result = self.path_planner.plan_path(
                vehicle_state.vehicle_id,
                task.start,
                task.goal,
                use_backbone=True
            )
            
            if result:
                # 处理规划结果
                if isinstance(result, tuple):
                    task.path, task.path_structure = result
                else:
                    task.path = result
                    task.path_structure = {'type': 'direct'}
                
                # 注册到交通管理器
                if self.traffic_manager:
                    self.traffic_manager.register_vehicle_path(
                        vehicle_state.vehicle_id, task.path, task.start_time
                    )
                
                # 开始移动
                return self._start_movement(task, vehicle_state)
        
        except Exception as e:
            print(f"任务 {task.task_id} 规划失败: {e}")
        
        # 规划失败
        task.status = TaskStatus.FAILED
        vehicle_state.status = VehicleStatus.IDLE
        self.stats['failed_tasks'] += 1
        return False
    
    def _start_movement(self, task: SimpleTask, vehicle_state: VehicleState) -> bool:
        """开始移动"""
        if not task.path:
            return False
        
        # 更新状态
        vehicle_state.status = VehicleStatus.MOVING
        
        # 同步到环境
        if vehicle_state.vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
            env_vehicle['status'] = 'moving'
            env_vehicle['path'] = task.path
            env_vehicle['path_index'] = 0
            env_vehicle['progress'] = 0.0
            env_vehicle['path_structure'] = task.path_structure or {}
        
        print(f"车辆 {vehicle_state.vehicle_id} 开始任务 {task.task_id} ({task.task_type})")
        return True
    
    def update(self, time_delta: float):
        """主更新循环"""
        with self.state_lock:
            for vehicle_id, vehicle_state in self.vehicle_states.items():
                self._update_vehicle(vehicle_id, vehicle_state, time_delta)
    
    def _update_vehicle(self, vehicle_id: str, vehicle_state: VehicleState, 
                       time_delta: float):
        """更新单个车辆"""
        # 同步环境数据
        if vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_id]
            vehicle_state.position = env_vehicle.get('position', vehicle_state.position)
            vehicle_state.current_load = env_vehicle.get('load', vehicle_state.current_load)
            
            # 状态映射
            env_status = env_vehicle.get('status', 'idle')
            status_map = {
                'idle': VehicleStatus.IDLE,
                'moving': VehicleStatus.MOVING,
                'loading': VehicleStatus.LOADING,
                'unloading': VehicleStatus.UNLOADING,
                'planning': VehicleStatus.PLANNING
            }
            
            if not vehicle_state.current_task:  # 只在无任务时同步状态
                vehicle_state.status = status_map.get(env_status, VehicleStatus.IDLE)
        
        # 处理当前任务
        if vehicle_state.current_task:
            self._update_task_execution(vehicle_state, time_delta)
        elif vehicle_state.status == VehicleStatus.IDLE:
            # 尝试开始下一个任务
            self._start_next_task(vehicle_id)
    
    def _update_task_execution(self, vehicle_state: VehicleState, time_delta: float):
        """更新任务执行"""
        task_id = vehicle_state.current_task
        if task_id not in self.tasks:
            return
        
        task = self.tasks[task_id]
        
        if vehicle_state.status == VehicleStatus.MOVING:
            self._update_movement(task, vehicle_state, time_delta)
        elif vehicle_state.status in [VehicleStatus.LOADING, VehicleStatus.UNLOADING]:
            self._update_operation(task, vehicle_state, time_delta)
    
    def _update_movement(self, task: SimpleTask, vehicle_state: VehicleState, 
                        time_delta: float):
        """更新移动状态"""
        if vehicle_state.vehicle_id not in self.env.vehicles:
            return
        
        env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
        progress = env_vehicle.get('progress', 0.0)
        
        # 计算新进度
        if task.path and len(task.path) > 1:
            path_length = self._calculate_path_length(task.path)
            if path_length > 0:
                speed = vehicle_state.speed
                distance_increment = speed * time_delta
                progress_increment = distance_increment / path_length
                new_progress = min(1.0, progress + progress_increment)
                
                env_vehicle['progress'] = new_progress
                
                # 更新位置
                if new_progress < 1.0:
                    new_position = self._interpolate_position(task.path, new_progress)
                    env_vehicle['position'] = new_position
                    vehicle_state.position = new_position
                
                # 检查到达
                if new_progress >= 0.95:
                    self._handle_arrival(task, vehicle_state)
    
    def _calculate_path_length(self, path: List) -> float:
        """计算路径长度"""
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.sqrt(dx*dx + dy*dy)
        
        return length
    
    def _interpolate_position(self, path: List, progress: float) -> Tuple:
        """路径位置插值"""
        if not path or len(path) < 2:
            return path[0] if path else (0, 0, 0)
        
        if progress <= 0:
            return path[0]
        if progress >= 1.0:
            return path[-1]
        
        # 简单线性插值
        total_length = self._calculate_path_length(path)
        target_distance = total_length * progress
        
        current_distance = 0.0
        for i in range(len(path) - 1):
            segment_length = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            
            if current_distance + segment_length >= target_distance:
                # 在此段内
                remaining = target_distance - current_distance
                ratio = remaining / segment_length if segment_length > 0 else 0
                
                x = path[i][0] + ratio * (path[i+1][0] - path[i][0])
                y = path[i][1] + ratio * (path[i+1][1] - path[i][1])
                theta = path[i][2] if len(path[i]) > 2 else 0
                
                return (x, y, theta)
            
            current_distance += segment_length
        
        return path[-1]
    
    def _handle_arrival(self, task: SimpleTask, vehicle_state: VehicleState):
        """处理到达事件"""
        if task.task_type == 'to_loading':
            vehicle_state.status = VehicleStatus.LOADING
            if vehicle_state.vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_state.vehicle_id]['status'] = 'loading'
            print(f"车辆 {vehicle_state.vehicle_id} 到达装载点")
            
        elif task.task_type == 'to_unloading':
            vehicle_state.status = VehicleStatus.UNLOADING
            if vehicle_state.vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_state.vehicle_id]['status'] = 'unloading'
            print(f"车辆 {vehicle_state.vehicle_id} 到达卸载点")
            
        elif task.task_type == 'to_initial':
            # 完成循环
            self._complete_task(task, vehicle_state)
            self._increment_cycle(vehicle_state)
            self._auto_assign_next_cycle(vehicle_state.vehicle_id)
        
        task.start_time = time.time()  # 重置用于操作计时
    
    def _update_operation(self, task: SimpleTask, vehicle_state: VehicleState, 
                         time_delta: float):
        """更新操作进度"""
        operation_time = 60 if vehicle_state.status == VehicleStatus.LOADING else 40
        elapsed = time.time() - task.start_time
        
        if elapsed >= operation_time:
            if vehicle_state.status == VehicleStatus.LOADING:
                vehicle_state.current_load = vehicle_state.max_load
                if vehicle_state.vehicle_id in self.env.vehicles:
                    self.env.vehicles[vehicle_state.vehicle_id]['load'] = vehicle_state.max_load
                print(f"车辆 {vehicle_state.vehicle_id} 装载完成")
            else:
                vehicle_state.current_load = 0
                if vehicle_state.vehicle_id in self.env.vehicles:
                    self.env.vehicles[vehicle_state.vehicle_id]['load'] = 0
                print(f"车辆 {vehicle_state.vehicle_id} 卸载完成")
            
            self._complete_task(task, vehicle_state)
    
    def _complete_task(self, task: SimpleTask, vehicle_state: VehicleState):
        """完成任务"""
        # 更新任务状态
        task.status = TaskStatus.COMPLETED
        task.completion_time = time.time()
        
        # 更新统计
        self.stats['completed_tasks'] += 1
        
        # 从分配中移除
        if task.task_id in self.active_assignments[vehicle_state.vehicle_id]:
            self.active_assignments[vehicle_state.vehicle_id].remove(task.task_id)
        
        # 释放交通管理器资源
        if self.traffic_manager:
            self.traffic_manager.release_vehicle_path(vehicle_state.vehicle_id)
        
        # 清理车辆状态
        vehicle_state.current_task = None
        vehicle_state.status = VehicleStatus.IDLE
        
        if vehicle_state.vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
            env_vehicle['status'] = 'idle'
            env_vehicle['progress'] = 0.0
            env_vehicle['path'] = None
        
        print(f"任务 {task.task_id} 完成")
        
        # 开始下一个任务
        self._start_next_task(vehicle_state.vehicle_id)
    
    def _increment_cycle(self, vehicle_state: VehicleState):
        """增加循环计数"""
        vehicle_state.completed_cycles += 1
        
        if vehicle_state.vehicle_id in self.env.vehicles:
            self.env.vehicles[vehicle_state.vehicle_id]['completed_cycles'] = vehicle_state.completed_cycles
        
        print(f"车辆 {vehicle_state.vehicle_id} 完成第 {vehicle_state.completed_cycles} 个循环")
    
    def _auto_assign_next_cycle(self, vehicle_id: str):
        """自动分配下一个循环"""
        if not self.active_assignments[vehicle_id]:
            # 自动分配新循环
            self.assign_mission_intelligently(vehicle_id, "default")
    
    def get_comprehensive_stats(self) -> Dict:
        """获取统计信息"""
        stats = self.stats.copy()
        
        # 实时状态
        active_vehicles = len([v for v in self.vehicle_states.values() 
                             if v.status != VehicleStatus.IDLE])
        idle_vehicles = len([v for v in self.vehicle_states.values() 
                           if v.status == VehicleStatus.IDLE])
        
        stats['real_time'] = {
            'active_vehicles': active_vehicles,
            'idle_vehicles': idle_vehicles,
            'total_vehicles': len(self.vehicle_states)
        }
        
        return stats
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
    
    def shutdown(self):
        """关闭调度器"""
        with self.state_lock:
            self.tasks.clear()
            self.vehicle_states.clear()
            self.active_assignments.clear()
        
        print("车辆调度器已关闭")

# 向后兼容性
SimplifiedECBSVehicleScheduler = SimplifiedVehicleScheduler
HybridAStarIntegratedScheduler = SimplifiedVehicleScheduler