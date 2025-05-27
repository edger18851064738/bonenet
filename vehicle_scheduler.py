"""
vehicle_scheduler.py - 深度集成优化版车辆调度器
基于混合A*规划器的智能调度系统
"""

import math
import time
import threading
import numpy as np
from typing import List, Dict, Tuple, Set, Optional, Any, Callable
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass, field
from enum import Enum
import heapq
import asyncio
from concurrent.futures import ThreadPoolExecutor, as_completed

class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
    OPTIMIZING = "optimizing"  # 新增：优化中状态

class VehicleStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    LOADING = "loading"
    UNLOADING = "unloading"
    WAITING = "waiting"
    MAINTENANCE = "maintenance"
    PLANNING = "planning"  # 新增：规划中状态

class TaskPriority(Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4
    EMERGENCY = 5

@dataclass
class EnhancedVehicleTask:
    """增强版车辆任务"""
    task_id: str
    task_type: str
    start: Tuple[float, float, float]
    goal: Tuple[float, float, float]
    priority: TaskPriority = TaskPriority.NORMAL
    status: TaskStatus = TaskStatus.PENDING
    
    # 任务属性
    loading_point_id: Optional[int] = None
    unloading_point_id: Optional[int] = None
    deadline: Optional[float] = None
    estimated_duration: float = 0
    max_attempts: int = 3
    
    # 分配信息
    assigned_vehicle: Optional[str] = None
    assignment_time: float = 0
    start_time: float = 0
    completion_time: float = 0
    attempt_count: int = 0
    
    # 路径信息
    path: Optional[List] = None
    path_structure: Dict = field(default_factory=dict)
    quality_score: float = 0.0
    planner_used: str = "unknown"
    
    # 性能指标
    planning_time: float = 0
    execution_progress: float = 0
    actual_duration: float = 0
    efficiency_score: float = 0.0
    
    # 优化信息
    alternative_paths: List = field(default_factory=list)
    optimization_attempts: int = 0
    backbone_utilization: float = 0.0
    
    # 依赖关系
    prerequisites: List[str] = field(default_factory=list)
    dependents: List[str] = field(default_factory=list)
    
    def update_progress(self, progress: float):
        """更新执行进度"""
        self.execution_progress = max(0, min(1.0, progress))
        if self.execution_progress >= 1.0:
            self.status = TaskStatus.COMPLETED
            self.completion_time = time.time()
            self.actual_duration = self.completion_time - self.start_time
            self.efficiency_score = self._calculate_efficiency()
    
    def _calculate_efficiency(self) -> float:
        """计算任务效率"""
        if self.estimated_duration <= 0:
            return 1.0
        
        # 基于实际用时与估计用时的比较
        time_efficiency = min(1.0, self.estimated_duration / max(1.0, self.actual_duration))
        
        # 结合路径质量
        combined_efficiency = (time_efficiency * 0.6 + self.quality_score * 0.4)
        
        return combined_efficiency
    
    def can_start(self, completed_tasks: Set[str]) -> bool:
        """检查是否可以开始（依赖已完成）"""
        return all(prereq in completed_tasks for prereq in self.prerequisites)
    
    def get_urgency_score(self, current_time: float) -> float:
        """计算紧急度评分"""
        base_score = self.priority.value
        
        # 时间因素
        if self.deadline:
            time_remaining = self.deadline - current_time
            if time_remaining <= 0:
                return 10.0  # 超期任务最高优先级
            elif time_remaining < self.estimated_duration * 1.2:
                base_score *= 2.0  # 接近截止时间
        
        # 尝试次数惩罚
        if self.attempt_count > 0:
            base_score *= (1.0 + self.attempt_count * 0.3)
        
        return base_score

@dataclass
class SmartVehicleState:
    """智能车辆状态"""
    vehicle_id: str
    status: VehicleStatus = VehicleStatus.IDLE
    position: Tuple[float, float, float] = (0, 0, 0)
    target_position: Tuple[float, float, float] = None
    
    # 基本属性
    max_load: float = 100
    current_load: float = 0
    speed: float = 1.0
    
    # 任务相关
    current_task: Optional[str] = None
    task_queue: deque = field(default_factory=deque)
    completed_tasks: Set[str] = field(default_factory=set)
    
    # 性能统计
    total_distance: float = 0
    total_time: float = 0
    idle_time: float = 0
    utilization_rate: float = 0
    
    # 智能统计
    backbone_usage_count: int = 0
    direct_path_count: int = 0
    interface_efficiency: float = 0.5
    average_quality_score: float = 0.5
    
    # 预测信息
    estimated_completion_time: float = 0
    next_available_time: float = 0
    workload_score: float = 0.0
    
    # 专长评估
    specialization_scores: Dict[str, float] = field(default_factory=lambda: {
        'short_distance': 0.5,
        'long_distance': 0.5,
        'complex_terrain': 0.5,
        'high_priority': 0.5
    })
    
    def update_utilization(self, time_delta: float):
        """更新利用率"""
        if self.status != VehicleStatus.IDLE:
            self.total_time += time_delta
        else:
            self.idle_time += time_delta
        
        total_elapsed = self.total_time + self.idle_time
        if total_elapsed > 0:
            self.utilization_rate = self.total_time / total_elapsed
    
    def update_specialization(self, task: EnhancedVehicleTask):
        """更新专长评分"""
        if not task.path:
            return
        
        # 距离专长
        path_distance = sum(
            math.sqrt((task.path[i+1][0] - task.path[i][0])**2 + 
                     (task.path[i+1][1] - task.path[i][1])**2)
            for i in range(len(task.path) - 1)
        )
        
        if path_distance < 50:
            self._update_score('short_distance', task.efficiency_score)
        else:
            self._update_score('long_distance', task.efficiency_score)
        
        # 复杂度专长
        complexity = task.path_structure.get('complexity', 0.5)
        if complexity > 0.7:
            self._update_score('complex_terrain', task.efficiency_score)
        
        # 优先级专长
        if task.priority.value >= 3:
            self._update_score('high_priority', task.efficiency_score)
    
    def _update_score(self, category: str, new_score: float, learning_rate: float = 0.1):
        """更新评分（指数移动平均）"""
        current_score = self.specialization_scores.get(category, 0.5)
        self.specialization_scores[category] = (
            (1 - learning_rate) * current_score + learning_rate * new_score
        )
    
    def get_suitability_score(self, task: EnhancedVehicleTask, current_time: float) -> float:
        """计算对任务的适合度"""
        # 基础可用性
        if self.status not in [VehicleStatus.IDLE, VehicleStatus.MOVING]:
            return 0.0
        
        # 距离因素
        distance_to_start = math.sqrt(
            (self.position[0] - task.start[0])**2 + 
            (self.position[1] - task.start[1])**2
        )
        distance_score = max(0, 1.0 - distance_to_start / 200.0)
        
        # 专长匹配
        task_distance = math.sqrt(
            (task.goal[0] - task.start[0])**2 + 
            (task.goal[1] - task.start[1])**2
        )
        
        if task_distance < 50:
            specialization_score = self.specialization_scores['short_distance']
        else:
            specialization_score = self.specialization_scores['long_distance']
        
        if task.priority.value >= 3:
            specialization_score = (specialization_score + 
                                  self.specialization_scores['high_priority']) / 2
        
        # 负载因素
        workload_score = max(0, 1.0 - self.workload_score / 5.0)
        
        # 综合评分
        suitability = (
            distance_score * 0.3 + 
            specialization_score * 0.4 + 
            workload_score * 0.2 + 
            self.utilization_rate * 0.1  # 高利用率车辆优先
        )
        
        return min(1.0, suitability)

class PerformanceProfiler:
    """性能分析器"""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.metrics = defaultdict(deque)
        self.timestamps = defaultdict(deque)
        
    def record_metric(self, name: str, value: float, timestamp: float = None):
        """记录性能指标"""
        if timestamp is None:
            timestamp = time.time()
        
        self.metrics[name].append(value)
        self.timestamps[name].append(timestamp)
        
        # 保持窗口大小
        if len(self.metrics[name]) > self.window_size:
            self.metrics[name].popleft()
            self.timestamps[name].popleft()
    
    def get_average(self, name: str, time_window: float = None) -> float:
        """获取平均值"""
        if name not in self.metrics or not self.metrics[name]:
            return 0.0
        
        values = list(self.metrics[name])
        
        if time_window is not None:
            current_time = time.time()
            timestamps = list(self.timestamps[name])
            # 过滤时间窗口内的数据
            filtered_values = [
                v for v, t in zip(values, timestamps)
                if current_time - t <= time_window
            ]
            values = filtered_values if filtered_values else values
        
        return sum(values) / len(values) if values else 0.0
    
    def get_trend(self, name: str) -> float:
        """获取趋势（正值上升，负值下降）"""
        if name not in self.metrics or len(self.metrics[name]) < 2:
            return 0.0
        
        values = list(self.metrics[name])
        if len(values) < 10:
            return 0.0
        
        # 简单线性回归趋势
        n = len(values)
        x = list(range(n))
        y = values
        
        sum_x = sum(x)
        sum_y = sum(y)
        sum_xy = sum(xi * yi for xi, yi in zip(x, y))
        sum_x2 = sum(xi * xi for xi in x)
        
        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
        return slope

class HybridAStarIntegratedScheduler:
    """混合A*深度集成的智能调度器"""
    
    def __init__(self, env, path_planner=None, backbone_network=None, traffic_manager=None):
        # 核心组件
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 数据存储
        self.tasks = OrderedDict()  # {task_id: EnhancedVehicleTask}
        self.vehicle_states = {}  # {vehicle_id: SmartVehicleState}
        self.mission_templates = {}  # {template_id: mission_config}
        
        # 任务管理
        self.task_counter = 0
        self.active_assignments = defaultdict(list)  # {vehicle_id: [task_ids]}
        self.task_dependencies = {}  # {task_id: [dependent_task_ids]}
        
        # 智能调度配置
        self.scheduling_config = {
            'max_concurrent_planning': 4,
            'replan_interval': 30.0,
            'optimization_interval': 60.0,
            'performance_window': 300.0,  # 5分钟性能窗口
            'quality_threshold': 0.7,
            'efficiency_threshold': 0.6,
            'predictive_scheduling': True,
            'adaptive_parameters': True
        }
        
        # 性能分析
        self.profiler = PerformanceProfiler(window_size=200)
        
        # 优化器
        self.executor = ThreadPoolExecutor(max_workers=self.scheduling_config['max_concurrent_planning'])
        
        # 状态锁
        self.state_lock = threading.RLock()
        
        # 定时器
        self.last_replan_time = 0
        self.last_optimization_time = 0
        
        # 统计信息
        self.stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'optimization_cycles': 0,
            'replanning_cycles': 0,
            'average_completion_time': 0,
            'total_distance': 0,
            'backbone_utilization_rate': 0,
            'astar_usage_rate': 0,
            'path_quality_average': 0,
            'planning_success_rate': 0,
            'vehicle_utilization': {},
            'performance_trends': {}
        }
        
        print("初始化混合A*深度集成智能调度器")
        self._initialize_performance_monitoring()
    
    def _initialize_performance_monitoring(self):
        """初始化性能监控"""
        # 关键性能指标
        self.key_metrics = [
            'planning_time', 'path_quality', 'completion_time',
            'vehicle_utilization', 'backbone_usage', 'conflict_rate'
        ]
        
        # 初始化监控
        for metric in self.key_metrics:
            self.profiler.record_metric(metric, 0.5)  # 初始基准值
    
    def initialize_vehicles(self):
        """初始化车辆状态"""
        with self.state_lock:
            for vehicle_id, vehicle_data in self.env.vehicles.items():
                position = vehicle_data.get('position', (0, 0, 0))
                max_load = vehicle_data.get('max_load', 100)
                
                self.vehicle_states[vehicle_id] = SmartVehicleState(
                    vehicle_id=vehicle_id,
                    position=position,
                    max_load=max_load,
                    current_load=vehicle_data.get('load', 0)
                )
                
                self.active_assignments[vehicle_id] = []
        
        print(f"初始化了 {len(self.vehicle_states)} 个智能车辆状态")
    
    def create_enhanced_mission_template(self, template_id: str, 
                                       loading_point_id: int = None, 
                                       unloading_point_id: int = None,
                                       priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """创建增强任务模板"""
        if not self.env.loading_points or not self.env.unloading_points:
            return False
        
        # 智能选择最优装载点和卸载点
        if loading_point_id is None:
            loading_point_id = self._select_optimal_loading_point()
        
        if unloading_point_id is None:
            unloading_point_id = self._select_optimal_unloading_point()
        
        # 验证点位有效性
        if (loading_point_id >= len(self.env.loading_points) or 
            unloading_point_id >= len(self.env.unloading_points)):
            return False
        
        loading_point = self.env.loading_points[loading_point_id]
        unloading_point = self.env.unloading_points[unloading_point_id]
        
        # 创建智能任务模板
        template = {
            'loading_point_id': loading_point_id,
            'unloading_point_id': unloading_point_id,
            'loading_position': loading_point,
            'unloading_position': unloading_point,
            'base_priority': priority,
            'estimated_cycle_time': self._estimate_cycle_time(loading_point, unloading_point),
            'tasks': [
                {
                    'task_type': 'to_loading',
                    'goal': loading_point,
                    'priority': priority,
                    'estimated_duration': 180,
                    'quality_requirement': 0.7
                },
                {
                    'task_type': 'to_unloading',
                    'goal': unloading_point,
                    'priority': TaskPriority(min(5, priority.value + 1)),  # 提高运输优先级
                    'estimated_duration': 150,
                    'quality_requirement': 0.8
                },
                {
                    'task_type': 'to_initial',
                    'goal': None,
                    'priority': TaskPriority.LOW,
                    'estimated_duration': 120,
                    'quality_requirement': 0.6
                }
            ]
        }
        
        self.mission_templates[template_id] = template
        print(f"创建增强任务模板 {template_id}: L{loading_point_id} -> U{unloading_point_id}")
        return True
    
    def _select_optimal_loading_point(self) -> int:
        """智能选择最优装载点"""
        if not self.env.loading_points:
            return 0
        
        # 基于当前负载选择最少使用的装载点
        usage_count = defaultdict(int)
        
        for task in self.tasks.values():
            if task.loading_point_id is not None:
                usage_count[task.loading_point_id] += 1
        
        # 选择使用最少的装载点
        min_usage = min(usage_count.values()) if usage_count else 0
        candidates = [i for i in range(len(self.env.loading_points))
                     if usage_count[i] == min_usage]
        
        return candidates[0] if candidates else 0
    
    def _select_optimal_unloading_point(self) -> int:
        """智能选择最优卸载点"""
        if not self.env.unloading_points:
            return 0
        
        # 类似装载点的选择逻辑
        usage_count = defaultdict(int)
        
        for task in self.tasks.values():
            if task.unloading_point_id is not None:
                usage_count[task.unloading_point_id] += 1
        
        min_usage = min(usage_count.values()) if usage_count else 0
        candidates = [i for i in range(len(self.env.unloading_points))
                     if usage_count[i] == min_usage]
        
        return candidates[0] if candidates else 0
    
    def _estimate_cycle_time(self, loading_point: Tuple, unloading_point: Tuple) -> float:
        """估算循环时间"""
        # 简化的时间估算
        distance = math.sqrt(
            (unloading_point[0] - loading_point[0])**2 + 
            (unloading_point[1] - loading_point[1])**2
        )
        
        # 假设平均速度和操作时间
        travel_time = distance / 1.0  # 1 unit/second
        operation_time = 120  # 2分钟操作时间
        
        return travel_time * 2 + operation_time  # 往返 + 操作
    
    def assign_mission_intelligently(self, vehicle_id: str = None, 
                                   template_id: str = None,
                                   priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """智能任务分配"""
        with self.state_lock:
            # 自动选择最适合的车辆
            if vehicle_id is None:
                vehicle_id = self._select_optimal_vehicle_for_mission(template_id, priority)
                if not vehicle_id:
                    print("没有可用车辆分配任务")
                    return False
            
            if vehicle_id not in self.vehicle_states:
                return False
            
            # 自动选择最佳模板
            if template_id is None:
                template_id = "default"
                if template_id not in self.mission_templates:
                    if not self.create_enhanced_mission_template(template_id, priority=priority):
                        return False
            
            if template_id not in self.mission_templates:
                return False
            
            template = self.mission_templates[template_id]
            vehicle_state = self.vehicle_states[vehicle_id]
            
            # 生成智能任务序列
            created_tasks = []
            current_position = vehicle_state.position
            current_time = time.time()
            
            for i, task_template in enumerate(template['tasks']):
                task_id = f"task_{self.task_counter}"
                self.task_counter += 1
                
                # 确定目标位置
                if task_template['goal'] is None:
                    goal = self.env.vehicles[vehicle_id].get('position', current_position)
                else:
                    goal = task_template['goal']
                
                # 计算动态优先级
                dynamic_priority = self._calculate_dynamic_priority(
                    task_template['priority'], vehicle_state, current_time
                )
                
                # 创建增强任务
                task = EnhancedVehicleTask(
                    task_id=task_id,
                    task_type=task_template['task_type'],
                    start=current_position,
                    goal=goal,
                    priority=dynamic_priority,
                    loading_point_id=template.get('loading_point_id'),
                    unloading_point_id=template.get('unloading_point_id'),
                    estimated_duration=task_template['estimated_duration'],
                    deadline=current_time + task_template['estimated_duration'] * 1.5
                )
                
                # 设置依赖关系
                if created_tasks:
                    task.prerequisites = [created_tasks[-1]]
                    self.tasks[created_tasks[-1]].dependents.append(task_id)
                
                self.tasks[task_id] = task
                created_tasks.append(task_id)
                
                # 更新下一任务的起点
                current_position = goal
            
            # 分配任务到车辆
            self.active_assignments[vehicle_id].extend(created_tasks)
            
            # 更新车辆工作负载
            vehicle_state.workload_score += len(created_tasks) * 0.5
            
            # 如果车辆空闲，预规划第一个任务
            if vehicle_state.status == VehicleStatus.IDLE:
                self._preplan_next_task(vehicle_id)
            
            self.stats['total_tasks'] += len(created_tasks)
            
            print(f"智能分配：车辆 {vehicle_id} 获得 {len(created_tasks)} 个任务 "
                  f"(模板: {template_id}, 优先级: {priority.name})")
            
            return True
    
    def _select_optimal_vehicle_for_mission(self, template_id: str, 
                                          priority: TaskPriority) -> Optional[str]:
        """选择最优车辆执行任务"""
        if not self.vehicle_states:
            return None
        
        # 评估所有可用车辆
        vehicle_scores = []
        current_time = time.time()
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if vehicle_state.status not in [VehicleStatus.IDLE, VehicleStatus.MOVING]:
                continue
            
            # 创建临时任务用于评估
            if template_id and template_id in self.mission_templates:
                template = self.mission_templates[template_id]
                first_goal = template['tasks'][0]['goal']
                
                temp_task = EnhancedVehicleTask(
                    task_id="temp",
                    task_type="evaluation",
                    start=vehicle_state.position,
                    goal=first_goal,
                    priority=priority
                )
                
                suitability = vehicle_state.get_suitability_score(temp_task, current_time)
                vehicle_scores.append((vehicle_id, suitability))
        
        if not vehicle_scores:
            return None
        
        # 选择最高评分的车辆
        best_vehicle = max(vehicle_scores, key=lambda x: x[1])
        return best_vehicle[0] if best_vehicle[1] > 0.3 else None
    
    def _calculate_dynamic_priority(self, base_priority: TaskPriority, 
                                   vehicle_state: SmartVehicleState, 
                                   current_time: float) -> TaskPriority:
        """计算动态优先级"""
        priority_value = base_priority.value
        
        # 车辆利用率调整
        if vehicle_state.utilization_rate > 0.8:
            priority_value = max(1, priority_value - 1)  # 高利用率车辆降低优先级
        elif vehicle_state.utilization_rate < 0.3:
            priority_value = min(5, priority_value + 1)  # 低利用率车辆提高优先级
        
        # 时间因素
        hour = time.localtime(current_time).tm_hour
        if 8 <= hour <= 18:  # 工作时间
            priority_value = min(5, priority_value + 1)
        
        return TaskPriority(int(priority_value))
    
    def _preplan_next_task(self, vehicle_id: str) -> bool:
        """预规划下一个任务"""
        if vehicle_id not in self.active_assignments:
            return False
        
        assignments = self.active_assignments[vehicle_id]
        if not assignments:
            return False
        
        # 找到下一个可执行的任务
        next_task_id = None
        completed_tasks = self.vehicle_states[vehicle_id].completed_tasks
        
        for task_id in assignments:
            if task_id not in self.tasks:
                continue
            
            task = self.tasks[task_id]
            if (task.status == TaskStatus.PENDING and 
                task.can_start(completed_tasks)):
                next_task_id = task_id
                break
        
        if not next_task_id:
            return False
        
        # 异步预规划
        future = self.executor.submit(self._plan_task_async, next_task_id)
        
        return True
    
    def _plan_task_async(self, task_id: str) -> bool:
        """异步任务规划"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        vehicle_id = task.assigned_vehicle
        
        if not vehicle_id or vehicle_id not in self.vehicle_states:
            return False
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        planning_start = time.time()
        
        try:
            # 使用混合A*规划器
            result = self.path_planner.plan_path(
                vehicle_id, 
                task.start, 
                task.goal,
                use_backbone=True,
                check_conflicts=True,
                planner_type="hybrid_astar"  # 优先使用混合A*
            )
            
            if result:
                success = self._process_planning_result(task, vehicle_state, result)
                
                if success:
                    # 记录性能指标
                    planning_time = time.time() - planning_start
                    self.profiler.record_metric('planning_time', planning_time)
                    self.profiler.record_metric('path_quality', task.quality_score)
                    
                    return True
        
        except Exception as e:
            print(f"异步规划失败 {task_id}: {e}")
        
        return False
    
    def _process_planning_result(self, task: EnhancedVehicleTask, 
                               vehicle_state: SmartVehicleState, 
                               result: Any) -> bool:
        """处理规划结果"""
        if isinstance(result, tuple) and len(result) == 2:
            path, structure = result
            task.path = path
            task.path_structure = structure
            task.quality_score = structure.get('final_quality', 0.5)
            task.planner_used = structure.get('planner_used', 'unknown')
            
            # 分析骨干网络利用率
            if structure.get('type') in ['interface_assisted', 'backbone_only']:
                task.backbone_utilization = structure.get('backbone_utilization', 1.0)
                vehicle_state.backbone_usage_count += 1
            else:
                task.backbone_utilization = 0.0
                vehicle_state.direct_path_count += 1
            
            # 存储备选路径
            if hasattr(self.path_planner, 'get_alternative_paths'):
                alternatives = self.path_planner.get_alternative_paths(
                    task.start, task.goal, max_alternatives=3
                )
                task.alternative_paths = alternatives or []
            
            return True
        
        else:
            # 处理简单路径结果
            task.path = result
            task.path_structure = {'type': 'direct'}
            task.quality_score = 0.6  # 默认质量
            task.planner_used = 'fallback'
            vehicle_state.direct_path_count += 1
            
            return True
    
    def update(self, time_delta: float):
        """主更新循环 - 智能调度核心"""
        current_time = time.time()
        
        # 更新所有车辆状态
        self._update_all_vehicles(time_delta)
        
        # 定期重规划
        if current_time - self.last_replan_time > self.scheduling_config['replan_interval']:
            self._intelligent_replanning()
            self.last_replan_time = current_time
        
        # 定期优化
        if current_time - self.last_optimization_time > self.scheduling_config['optimization_interval']:
            self._global_optimization()
            self.last_optimization_time = current_time
        
        # 更新性能统计
        self._update_performance_metrics()
        
        # 自适应参数调整
        if self.scheduling_config['adaptive_parameters']:
            self._adaptive_parameter_adjustment()
    
    def _update_all_vehicles(self, time_delta: float):
        """更新所有车辆状态"""
        with self.state_lock:
            for vehicle_id, vehicle_state in self.vehicle_states.items():
                self._update_single_vehicle(vehicle_id, vehicle_state, time_delta)
    
    def _update_single_vehicle(self, vehicle_id: str, vehicle_state: SmartVehicleState, 
                              time_delta: float):
        """更新单个车辆状态"""
        # 更新利用率
        vehicle_state.update_utilization(time_delta)
        
        # 同步环境数据
        self._sync_vehicle_with_environment(vehicle_id, vehicle_state)
        
        # 处理当前任务
        if vehicle_state.current_task:
            self._update_task_execution(vehicle_state.current_task, vehicle_state, time_delta)
        
        # 检查是否需要开始新任务
        elif vehicle_state.status == VehicleStatus.IDLE:
            self._try_start_next_task(vehicle_id)
    
    def _sync_vehicle_with_environment(self, vehicle_id: str, vehicle_state: SmartVehicleState):
        """同步车辆与环境状态"""
        if vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_id]
            
            # 同步位置（只在非移动状态下）
            if vehicle_state.status != VehicleStatus.MOVING:
                vehicle_state.position = env_vehicle.get('position', vehicle_state.position)
            
            # 同步载重
            vehicle_state.current_load = env_vehicle.get('load', vehicle_state.current_load)
            
            # 同步状态
            env_status = env_vehicle.get('status', 'idle')
            status_mapping = {
                'idle': VehicleStatus.IDLE,
                'moving': VehicleStatus.MOVING,
                'loading': VehicleStatus.LOADING,
                'unloading': VehicleStatus.UNLOADING
            }
            
            # 只在没有活跃任务时同步状态
            if not vehicle_state.current_task:
                vehicle_state.status = status_mapping.get(env_status, VehicleStatus.IDLE)
    
    def _update_task_execution(self, task_id: str, vehicle_state: SmartVehicleState, 
                              time_delta: float):
        """更新任务执行状态"""
        if task_id not in self.tasks:
            return
        
        task = self.tasks[task_id]
        
        if vehicle_state.status == VehicleStatus.MOVING and task.path:
            self._update_movement_progress(task, vehicle_state, time_delta)
        
        elif vehicle_state.status in [VehicleStatus.LOADING, VehicleStatus.UNLOADING]:
            self._update_operation_progress(task, vehicle_state, time_delta)
    
    def _update_movement_progress(self, task: EnhancedVehicleTask, 
                                 vehicle_state: SmartVehicleState, time_delta: float):
        """更新移动进度 - 优化版本"""
        if vehicle_state.vehicle_id not in self.env.vehicles:
            return
        
        env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
        path = task.path
        
        if not path or len(path) < 2:
            return
        
        # 获取当前进度
        current_progress = env_vehicle.get('progress', 0.0)
        
        # 计算移动速度（考虑路径复杂度）
        base_speed = vehicle_state.speed
        complexity_factor = 1.0 - task.path_structure.get('complexity', 0) * 0.3
        effective_speed = base_speed * complexity_factor
        
        # 计算路径总长度
        total_distance = self._calculate_path_distance(path)
        
        if total_distance > 0:
            # 计算进度增量
            distance_increment = effective_speed * time_delta
            progress_increment = distance_increment / total_distance
            
            # 更新进度
            new_progress = min(1.0, current_progress + progress_increment)
            env_vehicle['progress'] = new_progress
            
            # 计算新位置
            new_position, new_index = self._interpolate_path_position(path, new_progress)
            
            # 更新位置
            env_vehicle['position'] = new_position
            env_vehicle['path_index'] = new_index
            vehicle_state.position = new_position
            
            # 更新任务进度
            task.update_progress(new_progress)
            
            # 记录移动距离
            if new_progress > current_progress:
                distance_moved = distance_increment
                vehicle_state.total_distance += distance_moved
                self.stats['total_distance'] += distance_moved
            
            # 检查到达状态（95%以上认为到达）
            if new_progress >= 0.95:
                self._handle_task_completion(task, vehicle_state)
    
    def _calculate_path_distance(self, path: List) -> float:
        """计算路径总距离"""
        if not path or len(path) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(len(path) - 1):
            segment_distance = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            total_distance += segment_distance
        
        return total_distance
    
    def _interpolate_path_position(self, path: List, progress: float) -> Tuple[Tuple, int]:
        """路径位置插值"""
        if not path or len(path) < 2:
            return path[0] if path else (0, 0, 0), 0
        
        if progress <= 0:
            return path[0], 0
        if progress >= 1.0:
            return path[-1], len(path) - 1
        
        # 计算目标距离
        total_distance = self._calculate_path_distance(path)
        target_distance = total_distance * progress
        
        # 沿路径查找位置
        current_distance = 0.0
        
        for i in range(len(path) - 1):
            segment_distance = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            
            if current_distance + segment_distance >= target_distance:
                # 在此段内插值
                remaining = target_distance - current_distance
                ratio = remaining / segment_distance if segment_distance > 0 else 0
                
                x = path[i][0] + ratio * (path[i+1][0] - path[i][0])
                y = path[i][1] + ratio * (path[i+1][1] - path[i][1])
                theta = path[i][2] if len(path[i]) > 2 else 0
                
                return (x, y, theta), i
            
            current_distance += segment_distance
        
        return path[-1], len(path) - 1
    
    def _update_operation_progress(self, task: EnhancedVehicleTask, 
                                  vehicle_state: SmartVehicleState, time_delta: float):
        """更新操作进度（装载/卸载）"""
        operation_duration = 60 if vehicle_state.status == VehicleStatus.LOADING else 40
        elapsed_time = time.time() - task.start_time
        
        progress = min(1.0, elapsed_time / operation_duration)
        task.update_progress(progress)
        
        if progress >= 1.0:
            self._complete_operation(task, vehicle_state)
    
    def _handle_task_completion(self, task: EnhancedVehicleTask, 
                               vehicle_state: SmartVehicleState):
        """处理任务完成到达"""
        if task.task_type == 'to_loading':
            # 到达装载点
            vehicle_state.status = VehicleStatus.LOADING
            if vehicle_state.vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_state.vehicle_id]['status'] = 'loading'
                self.env.vehicles[vehicle_state.vehicle_id]['progress'] = 0.0
            
            task.start_time = time.time()  # 重置用于装载计时
            print(f"车辆 {vehicle_state.vehicle_id} 到达装载点，开始装载")
        
        elif task.task_type == 'to_unloading':
            # 到达卸载点
            vehicle_state.status = VehicleStatus.UNLOADING
            if vehicle_state.vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_state.vehicle_id]['status'] = 'unloading'
                self.env.vehicles[vehicle_state.vehicle_id]['progress'] = 0.0
            
            task.start_time = time.time()  # 重置用于卸载计时
            print(f"车辆 {vehicle_state.vehicle_id} 到达卸载点，开始卸载")
        
        elif task.task_type == 'to_initial':
            # 返回起点，完成循环
            self._complete_task_fully(task, vehicle_state)
            self._increment_cycle_count(vehicle_state.vehicle_id)
            self._auto_assign_next_cycle(vehicle_state.vehicle_id)
    
    def _complete_operation(self, task: EnhancedVehicleTask, 
                           vehicle_state: SmartVehicleState):
        """完成装载/卸载操作"""
        if task.task_type == 'to_loading':
            # 装载完成
            vehicle_state.current_load = vehicle_state.max_load
            if vehicle_state.vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_state.vehicle_id]['load'] = vehicle_state.max_load
            
            print(f"车辆 {vehicle_state.vehicle_id} 装载完成")
        
        elif task.task_type == 'to_unloading':
            # 卸载完成
            vehicle_state.current_load = 0
            if vehicle_state.vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_state.vehicle_id]['load'] = 0
            
            print(f"车辆 {vehicle_state.vehicle_id} 卸载完成")
        
        # 完成当前任务
        self._complete_task_fully(task, vehicle_state)
    
    def _complete_task_fully(self, task: EnhancedVehicleTask, 
                            vehicle_state: SmartVehicleState):
        """完全完成任务"""
        # 更新任务状态
        task.status = TaskStatus.COMPLETED
        task.completion_time = time.time()
        task.actual_duration = task.completion_time - task.assignment_time
        
        # 更新车辆专长
        vehicle_state.update_specialization(task)
        
        # 更新统计
        self.stats['completed_tasks'] += 1
        vehicle_state.completed_tasks.add(task.task_id)
        
        # 从活跃分配中移除
        if vehicle_state.vehicle_id in self.active_assignments:
            if task.task_id in self.active_assignments[vehicle_state.vehicle_id]:
                self.active_assignments[vehicle_state.vehicle_id].remove(task.task_id)
        
        # 释放交通管理器资源
        if self.traffic_manager:
            self.traffic_manager.release_vehicle_path(vehicle_state.vehicle_id)
        
        # 清理车辆状态
        vehicle_state.current_task = None
        vehicle_state.status = VehicleStatus.IDLE
        vehicle_state.workload_score = max(0, vehicle_state.workload_score - 0.5)
        
        if vehicle_state.vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
            env_vehicle['status'] = 'idle'
            env_vehicle['progress'] = 0.0
            env_vehicle['path'] = None
            env_vehicle['path_index'] = 0
        
        # 记录性能指标
        self.profiler.record_metric('completion_time', task.actual_duration)
        self.profiler.record_metric('path_quality', task.quality_score)
        
        print(f"任务 {task.task_id} 完全完成，车辆 {vehicle_state.vehicle_id}，"
              f"用时 {task.actual_duration:.1f}s，质量 {task.quality_score:.2f}，"
              f"规划器 {task.planner_used}")
        
        # 开始下一个任务
        self._try_start_next_task(vehicle_state.vehicle_id)
    
    def _try_start_next_task(self, vehicle_id: str) -> bool:
        """尝试开始下一个任务"""
        if vehicle_id not in self.active_assignments:
            return False
        
        assignments = self.active_assignments[vehicle_id]
        if not assignments:
            return False
        
        # 寻找下一个可执行任务
        vehicle_state = self.vehicle_states[vehicle_id]
        completed_tasks = vehicle_state.completed_tasks
        
        for task_id in assignments:
            if task_id not in self.tasks:
                continue
            
            task = self.tasks[task_id]
            if (task.status == TaskStatus.PENDING and 
                task.can_start(completed_tasks)):
                
                return self._start_task_execution(task_id, vehicle_id)
        
        return False
    
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
        if task.path:
            # 已有路径，直接开始移动
            return self._start_movement(task, vehicle_state)
        else:
            # 需要规划路径
            return self._plan_and_start_task(task, vehicle_state)
    
    def _plan_and_start_task(self, task: EnhancedVehicleTask, 
                            vehicle_state: SmartVehicleState) -> bool:
        """规划并开始任务"""
        planning_start = time.time()
        
        try:
            # 使用最佳规划器
            planner_type = self._select_best_planner(task, vehicle_state)
            
            result = self.path_planner.plan_path(
                vehicle_state.vehicle_id,
                task.start,
                task.goal,
                use_backbone=True,
                check_conflicts=True,
                planner_type=planner_type
            )
            
            if result:
                success = self._process_planning_result(task, vehicle_state, result)
                
                if success:
                    task.planning_time = time.time() - planning_start
                    
                    # 记录规划器性能
                    self.profiler.record_metric('planning_time', task.planning_time)
                    
                    # 注册到交通管理器
                    if self.traffic_manager:
                        self.traffic_manager.register_vehicle_path_enhanced(
                            vehicle_state.vehicle_id, task.path, 
                            task.path_structure, task.start_time
                        )
                    
                    return self._start_movement(task, vehicle_state)
        
        except Exception as e:
            print(f"任务 {task.task_id} 规划失败: {e}")
        
        # 规划失败处理
        task.status = TaskStatus.FAILED
        task.attempt_count += 1
        vehicle_state.status = VehicleStatus.IDLE
        self.stats['failed_tasks'] += 1
        
        return False
    
    def _select_best_planner(self, task: EnhancedVehicleTask, 
                            vehicle_state: SmartVehicleState) -> str:
        """选择最佳规划器"""
        # 基于任务特征选择规划器
        distance = math.sqrt(
            (task.goal[0] - task.start[0])**2 + 
            (task.goal[1] - task.start[1])**2
        )
        
        # 优先使用混合A*
        if distance > 100 or task.priority.value >= 3:
            return "hybrid_astar"
        elif vehicle_state.specialization_scores.get('complex_terrain', 0.5) > 0.7:
            return "hybrid_astar"
        else:
            return "auto"  # 自动选择
    
    def _start_movement(self, task: EnhancedVehicleTask, 
                       vehicle_state: SmartVehicleState) -> bool:
        """开始移动"""
        if not task.path:
            return False
        
        # 更新车辆状态
        vehicle_state.status = VehicleStatus.MOVING
        
        # 同步到环境
        if vehicle_state.vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
            env_vehicle['status'] = 'moving'
            env_vehicle['path'] = task.path
            env_vehicle['path_index'] = 0
            env_vehicle['progress'] = 0.0
            env_vehicle['path_structure'] = task.path_structure
        
        print(f"车辆 {vehicle_state.vehicle_id} 开始执行任务 {task.task_id} "
              f"({task.task_type})，路径长度: {len(task.path)}，"
              f"规划器: {task.planner_used}")
        
        return True
    
    def _increment_cycle_count(self, vehicle_id: str):
        """增加循环计数"""
        if vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_id]
            current_cycles = env_vehicle.get('completed_cycles', 0)
            env_vehicle['completed_cycles'] = current_cycles + 1
            
            print(f"车辆 {vehicle_id} 完成第 {current_cycles + 1} 个工作循环")
    
    def _auto_assign_next_cycle(self, vehicle_id: str):
        """自动分配下一个循环"""
        if (vehicle_id in self.active_assignments and 
            not self.active_assignments[vehicle_id]):
            
            # 自动分配新循环
            self.assign_mission_intelligently(vehicle_id, "default", TaskPriority.NORMAL)
    
    def _intelligent_replanning(self):
        """智能重规划"""
        print("执行智能重规划...")
        
        # 识别需要重规划的任务
        replan_candidates = []
        
        for task_id, task in self.tasks.items():
            if (task.status == TaskStatus.IN_PROGRESS and 
                task.quality_score < self.scheduling_config['quality_threshold']):
                replan_candidates.append(task_id)
        
        # 并发重规划
        if replan_candidates:
            futures = []
            for task_id in replan_candidates[:self.scheduling_config['max_concurrent_planning']]:
                future = self.executor.submit(self._replan_task, task_id)
                futures.append(future)
            
            # 等待完成
            for future in as_completed(futures, timeout=10):
                try:
                    result = future.result()
                    if result:
                        self.stats['replanning_cycles'] += 1
                except Exception as e:
                    print(f"重规划失败: {e}")
    
    def _replan_task(self, task_id: str) -> bool:
        """重规划单个任务"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        vehicle_id = task.assigned_vehicle
        
        if not vehicle_id or vehicle_id not in self.vehicle_states:
            return False
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        # 尝试规划更好的路径
        try:
            result = self.path_planner.plan_path(
                vehicle_id,
                vehicle_state.position,  # 从当前位置开始
                task.goal,
                use_backbone=True,
                check_conflicts=True,
                planner_type="hybrid_astar"
            )
            
            if result:
                new_quality = 0.5
                if isinstance(result, tuple):
                    _, structure = result
                    new_quality = structure.get('final_quality', 0.5)
                
                # 只在质量提升时更新
                if new_quality > task.quality_score + 0.1:
                    old_quality = task.quality_score
                    self._process_planning_result(task, vehicle_state, result)
                    
                    print(f"任务 {task_id} 重规划成功，质量提升: "
                          f"{old_quality:.2f} -> {task.quality_score:.2f}")
                    
                    return True
        
        except Exception as e:
            print(f"任务 {task_id} 重规划失败: {e}")
        
        return False
    
    def _global_optimization(self):
        """全局优化"""
        print("执行全局优化...")
        
        optimization_start = time.time()
        
        # 负载均衡
        self._load_balancing()
        
        # 资源优化
        self._resource_optimization()
        
        # 参数调优
        self._parameter_tuning()
        
        optimization_time = time.time() - optimization_start
        self.stats['optimization_cycles'] += 1
        
        print(f"全局优化完成，耗时 {optimization_time:.2f}s")
    
    def _load_balancing(self):
        """负载均衡"""
        # 计算车辆负载
        vehicle_loads = []
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            load = len(self.active_assignments.get(vehicle_id, []))
            vehicle_loads.append((vehicle_id, load, vehicle_state.workload_score))
        
        # 按负载排序
        vehicle_loads.sort(key=lambda x: x[1])
        
        # 重新分配负载过重的车辆任务
        if len(vehicle_loads) >= 2:
            highest_load = vehicle_loads[-1]
            lowest_load = vehicle_loads[0]
            
            if highest_load[1] - lowest_load[1] > 2:
                # 尝试转移任务
                self._transfer_tasks(highest_load[0], lowest_load[0])
    
    def _transfer_tasks(self, from_vehicle: str, to_vehicle: str):
        """转移任务"""
        if (from_vehicle not in self.active_assignments or 
            to_vehicle not in self.active_assignments):
            return
        
        from_assignments = self.active_assignments[from_vehicle]
        to_assignments = self.active_assignments[to_vehicle]
        
        # 寻找可转移的待处理任务
        transferable_tasks = []
        from_vehicle_state = self.vehicle_states[from_vehicle]
        
        for task_id in from_assignments:
            if task_id not in self.tasks:
                continue
            
            task = self.tasks[task_id]
            if (task.status == TaskStatus.PENDING and 
                task.can_start(from_vehicle_state.completed_tasks)):
                transferable_tasks.append(task_id)
        
        # 转移一个任务
        if transferable_tasks:
            task_id = transferable_tasks[0]
            task = self.tasks[task_id]
            
            # 更新分配
            from_assignments.remove(task_id)
            to_assignments.append(task_id)
            
            # 更新任务
            task.assigned_vehicle = to_vehicle
            
            print(f"任务 {task_id} 从车辆 {from_vehicle} 转移到 {to_vehicle}")
    
    def _resource_optimization(self):
        """资源优化"""
        # 优化骨干网络使用
        backbone_tasks = []
        direct_tasks = []
        
        for task in self.tasks.values():
            if task.status == TaskStatus.IN_PROGRESS:
                if task.backbone_utilization > 0:
                    backbone_tasks.append(task)
                else:
                    direct_tasks.append(task)
        
        # 记录利用率
        total_tasks = len(backbone_tasks) + len(direct_tasks)
        if total_tasks > 0:
            self.stats['backbone_utilization_rate'] = len(backbone_tasks) / total_tasks
    
    def _parameter_tuning(self):
        """参数调优"""
        # 基于性能趋势调整参数
        planning_trend = self.profiler.get_trend('planning_time')
        quality_trend = self.profiler.get_trend('path_quality')
        
        # 调整规划间隔
        if planning_trend > 0.1:  # 规划时间上升
            self.scheduling_config['replan_interval'] = min(60.0, 
                self.scheduling_config['replan_interval'] * 1.1)
        elif planning_trend < -0.1:  # 规划时间下降
            self.scheduling_config['replan_interval'] = max(15.0,
                self.scheduling_config['replan_interval'] * 0.9)
        
        # 调整质量阈值
        avg_quality = self.profiler.get_average('path_quality', time_window=300)
        if avg_quality > 0.8:
            self.scheduling_config['quality_threshold'] = min(0.9, 
                self.scheduling_config['quality_threshold'] + 0.05)
        elif avg_quality < 0.5:
            self.scheduling_config['quality_threshold'] = max(0.4,
                self.scheduling_config['quality_threshold'] - 0.05)
    
    def _update_performance_metrics(self):
        """更新性能指标"""
        # 更新车辆利用率
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            self.stats['vehicle_utilization'][vehicle_id] = vehicle_state.utilization_rate
        
        # 计算平均质量
        completed_tasks = [t for t in self.tasks.values() if t.status == TaskStatus.COMPLETED]
        if completed_tasks:
            avg_quality = sum(t.quality_score for t in completed_tasks) / len(completed_tasks)
            self.stats['path_quality_average'] = avg_quality
        
        # 计算A*使用率
        astar_tasks = [t for t in self.tasks.values() if 'astar' in t.planner_used.lower()]
        total_planned_tasks = [t for t in self.tasks.values() if t.planner_used != 'unknown']
        
        if total_planned_tasks:
            self.stats['astar_usage_rate'] = len(astar_tasks) / len(total_planned_tasks)
        
        # 计算规划成功率
        total_attempts = sum(t.attempt_count for t in self.tasks.values())
        if total_attempts > 0:
            self.stats['planning_success_rate'] = self.stats['completed_tasks'] / total_attempts
    
    def _adaptive_parameter_adjustment(self):
        """自适应参数调整"""
        # 基于系统性能动态调整参数
        avg_utilization = np.mean(list(self.stats['vehicle_utilization'].values())) if self.stats['vehicle_utilization'] else 0
        
        # 高利用率时减少重规划频率
        if avg_utilization > 0.8:
            self.scheduling_config['replan_interval'] *= 1.05
        elif avg_utilization < 0.5:
            self.scheduling_config['replan_interval'] *= 0.95
        
        # 限制调整范围
        self.scheduling_config['replan_interval'] = np.clip(
            self.scheduling_config['replan_interval'], 15.0, 120.0
        )
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息"""
        stats = self.stats.copy()
        
        # 实时状态
        stats['real_time'] = {
            'active_vehicles': len([v for v in self.vehicle_states.values() 
                                  if v.status != VehicleStatus.IDLE]),
            'idle_vehicles': len([v for v in self.vehicle_states.values() 
                                if v.status == VehicleStatus.IDLE]),
            'active_tasks': len([t for t in self.tasks.values() 
                               if t.status == TaskStatus.IN_PROGRESS]),
            'pending_tasks': len([t for t in self.tasks.values() 
                                if t.status == TaskStatus.PENDING]),
            'current_time': time.time()
        }
        
        # 性能趋势
        stats['performance_trends'] = {}
        for metric in self.key_metrics:
            stats['performance_trends'][metric] = {
                'average': self.profiler.get_average(metric),
                'trend': self.profiler.get_trend(metric),
                'recent_average': self.profiler.get_average(metric, time_window=300)
            }
        
        # 路径规划器统计
        if self.path_planner:
            planner_stats = self.path_planner.get_performance_stats()
            stats['path_planner_stats'] = planner_stats
        
        # 骨干网络统计
        if self.backbone_network:
            backbone_stats = self.backbone_network.get_planner_performance_stats()
            stats['backbone_network_stats'] = backbone_stats
        
        return stats
    
    def set_backbone_network(self, backbone_network):
        """设置骨干路径网络"""
        self.backbone_network = backbone_network
        print("已设置骨干路径网络到智能调度器")
    
    def shutdown(self):
        """关闭调度器"""
        print("关闭智能调度器...")
        
        # 关闭线程池
        self.executor.shutdown(wait=True)
        
        # 清理资源
        with self.state_lock:
            self.tasks.clear()
            self.vehicle_states.clear()
            self.active_assignments.clear()
        
        print("智能调度器已关闭")
    def get_vehicle_info(self, vehicle_id: str) -> Optional[Dict]:
        """获取车辆详细信息 - GUI兼容性方法"""
        if vehicle_id not in self.vehicle_states:
            return None
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        # 获取环境中的车辆数据
        env_vehicle = None
        if vehicle_id in self.env.vehicles:
            env_vehicle = self.env.vehicles[vehicle_id]
        
        # 构建车辆信息字典
        vehicle_info = {
            # 基本信息
            'vehicle_id': vehicle_id,
            'position': vehicle_state.position,
            'speed': vehicle_state.speed,
            'status': vehicle_state.status.value,
            'max_load': vehicle_state.max_load,
            'current_load': vehicle_state.current_load,
            'total_distance': vehicle_state.total_distance,
            'total_time': vehicle_state.total_time,
            'utilization_rate': vehicle_state.utilization_rate,
            'completed_tasks': vehicle_state.completed_tasks,
            
            # 当前任务信息
            'current_task': None,
            'task_queue': [],
            
            # 效率指标
            'efficiency_metrics': {
                'backbone_usage_rate': 0.0,
                'interface_efficiency': vehicle_state.interface_efficiency,
                'direct_path_rate': 0.0
            },
            
            # 性能历史
            'performance_history': []
        }
        
        # 处理当前任务
        if vehicle_state.current_task and vehicle_state.current_task in self.tasks:
            current_task = self.tasks[vehicle_state.current_task]
            
            vehicle_info['current_task'] = {
                'task_id': current_task.task_id,
                'task_type': current_task.task_type,
                'progress': current_task.execution_progress,
                'quality_score': current_task.quality_score,
                'planning_time': current_task.planning_time,
                'estimated_duration': current_task.estimated_duration,
                'actual_duration': current_task.actual_duration,
                'path_structure': current_task.path_structure.copy() if current_task.path_structure else {}
            }
        
        # 处理任务队列
        if vehicle_id in self.active_assignments:
            for task_id in self.active_assignments[vehicle_id]:
                if task_id in self.tasks and task_id != vehicle_state.current_task:
                    task = self.tasks[task_id]
                    vehicle_info['task_queue'].append({
                        'task_id': task.task_id,
                        'task_type': task.task_type,
                        'priority': task.priority,
                        'status': task.status.value,
                        'estimated_duration': task.estimated_duration
                    })
        
        # 计算效率指标
        total_path_count = vehicle_state.backbone_usage_count + vehicle_state.direct_path_count
        if total_path_count > 0:
            vehicle_info['efficiency_metrics']['backbone_usage_rate'] = (
                vehicle_state.backbone_usage_count / total_path_count
            )
            vehicle_info['efficiency_metrics']['direct_path_rate'] = (
                vehicle_state.direct_path_count / total_path_count
            )
        
        # 生成性能历史（基于完成的任务）
        completed_tasks = [
            task for task in self.tasks.values() 
            if (task.assigned_vehicle == vehicle_id and 
                task.status == TaskStatus.COMPLETED)
        ]
        
        # 取最近的10个完成任务作为历史
        recent_tasks = sorted(completed_tasks, 
                            key=lambda t: t.completion_time, 
                            reverse=True)[:10]
        
        for task in recent_tasks:
            history_record = {
                'timestamp': task.completion_time,
                'task_metrics': {
                    'task_type': task.task_type,
                    'quality_score': task.quality_score,
                    'execution_time': task.actual_duration,
                    'planning_time': task.planning_time,
                    'path_structure_type': task.path_structure.get('type', 'unknown') if task.path_structure else 'unknown'
                }
            }
            vehicle_info['performance_history'].append(history_record)
        
        return vehicle_info    
# 向后兼容性
SimplifiedVehicleScheduler = HybridAStarIntegratedScheduler
SimplifiedECBSVehicleScheduler = HybridAStarIntegratedScheduler
VehicleScheduler = HybridAStarIntegratedScheduler
OptimizedVehicleScheduler = HybridAStarIntegratedScheduler