"""
vehicle_scheduler.py - 集成ECBS功能的完整增强版车辆调度器
整合了效率最大化算法、智能任务分配、动态优先级调整和ECBS多车辆协调功能
"""

import math
import time
import threading
from typing import List, Dict, Tuple, Optional, Any
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass, field
from enum import Enum
import heapq
import random
class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    OPTIMIZING = "optimizing"

class VehicleStatus(Enum):
    IDLE = "idle"
    MOVING = "moving"
    LOADING = "loading"
    UNLOADING = "unloading"
    PLANNING = "planning"
    WAITING = "waiting"

class TaskPriority(Enum):
    LOW = 0.3
    NORMAL = 0.5
    HIGH = 0.7
    URGENT = 0.9
    CRITICAL = 1.0

class CoordinationMode(Enum):
    """协调模式"""
    SINGLE_VEHICLE = "single_vehicle"
    BATCH_COORDINATION = "batch_coordination"
    REAL_TIME_COORDINATION = "real_time_coordination"
    PERIODIC_COORDINATION = "periodic_coordination"

@dataclass
class EnhancedTask:
    """增强任务结构"""
    task_id: str
    task_type: str
    start: Tuple[float, float, float]
    goal: Tuple[float, float, float]
    status: TaskStatus = TaskStatus.PENDING
    priority: TaskPriority = TaskPriority.NORMAL
    
    # 基本属性
    assigned_vehicle: Optional[str] = None
    assignment_time: float = 0
    start_time: float = 0
    completion_time: float = 0
    deadline: Optional[float] = None
    
    # 路径信息
    path: Optional[List] = None
    path_structure: Dict = field(default_factory=dict)
    estimated_duration: float = 0
    actual_duration: float = 0
    
    # 效率相关
    backbone_utilization: float = 0.0
    planning_attempts: int = 0
    efficiency_score: float = 0.0
    
    def update_progress(self, progress: float):
        """更新进度"""
        if progress >= 1.0:
            self.status = TaskStatus.COMPLETED
            self.completion_time = time.time()
            self.actual_duration = self.completion_time - self.start_time
    
    def calculate_efficiency(self) -> float:
        """计算任务效率"""
        if self.estimated_duration > 0 and self.actual_duration > 0:
            time_efficiency = self.estimated_duration / self.actual_duration
            return min(1.0, time_efficiency * 0.7 + self.backbone_utilization * 0.3)
        return 0.5

@dataclass
class EnhancedVehicleState:
    """增强车辆状态"""
    vehicle_id: str
    status: VehicleStatus = VehicleStatus.IDLE
    position: Tuple[float, float, float] = (0, 0, 0)
    
    # 基本属性
    max_load: float = 100
    current_load: float = 0
    speed: float = 1.0
    
    # 车辆安全参数
    vehicle_length: float = 6.0
    vehicle_width: float = 3.0
    safety_margin: float = 1.5
    turning_radius: float = 8.0
    
    # 任务相关
    current_task: Optional[str] = None
    task_queue: List[str] = field(default_factory=list)
    completed_cycles: int = 0
    priority_level: float = 0.5
    
    # 骨干路径稳定性
    last_backbone_path_id: Optional[str] = None
    backbone_switch_count: int = 0
    last_backbone_switch_time: float = 0.0
    backbone_path_stability: float = 1.0
    
    # 冲突相关统计
    conflict_count: int = 0
    last_conflict_time: float = 0.0
    conflict_resolution_success_rate: float = 1.0
    
    # 效率统计
    total_distance: float = 0.0
    total_time: float = 0.0
    idle_time: float = 0.0
    productive_time: float = 0.0
    efficiency_history: List[float] = field(default_factory=list)
    backbone_usage_ratio: float = 0.0
    average_speed: float = 0.0
    
    # 性能指标
    last_efficiency_update: float = 0.0
    task_completion_count: int = 0
    
    def __post_init__(self):
        """初始化后处理"""
        if not self.efficiency_history:
            self.efficiency_history = [0.5]
        if self.last_efficiency_update == 0.0:
            self.last_efficiency_update = time.time()
    
    def calculate_efficiency_score(self) -> float:
        """计算车辆综合效率分数"""
        if not self.efficiency_history:
            return 0.5
        
        avg_efficiency = sum(self.efficiency_history) / len(self.efficiency_history)
        total_time = self.total_time if self.total_time > 0 else 1.0
        idle_ratio = self.idle_time / total_time
        idle_penalty = 1.0 - (idle_ratio * 0.5)
        backbone_bonus = 1.0 + (self.backbone_usage_ratio * 0.2)
        
        return avg_efficiency * idle_penalty * backbone_bonus
    
    def update_efficiency_history(self, efficiency: float):
        """更新效率历史"""
        self.efficiency_history.append(efficiency)
        
        if len(self.efficiency_history) > 20:
            self.efficiency_history = self.efficiency_history[-10:]
        
        self.last_efficiency_update = time.time()
    
    def update_average_speed(self):
        """更新平均速度"""
        if self.total_time > 0 and self.total_distance > 0:
            self.average_speed = self.total_distance / self.total_time
        else:
            self.average_speed = 0.0
    
    def get_safety_params(self) -> Dict:
        """获取车辆安全参数"""
        return {
            'length': self.vehicle_length,
            'width': self.vehicle_width,
            'safety_margin': self.safety_margin,
            'turning_radius': self.turning_radius
        }
    
    def update_backbone_stability(self, new_backbone_id: Optional[str]):
        """更新骨干路径稳定性"""
        current_time = time.time()
        
        if new_backbone_id and new_backbone_id != self.last_backbone_path_id:
            if self.last_backbone_path_id is not None:
                self.backbone_switch_count += 1
                self.last_backbone_switch_time = current_time
                
                switch_penalty = 0.15 if self.backbone_switch_count <= 3 else 0.25
                self.backbone_path_stability = max(0.1, 
                    self.backbone_path_stability - switch_penalty)
                
                print(f"车辆 {self.vehicle_id} 切换骨干路径: {self.last_backbone_path_id} -> {new_backbone_id} "
                      f"(第{self.backbone_switch_count}次)")
            
            self.last_backbone_path_id = new_backbone_id
        elif new_backbone_id == self.last_backbone_path_id:
            time_since_switch = current_time - self.last_backbone_switch_time
            if time_since_switch > 120:
                stability_bonus = min(0.05, (time_since_switch - 120) / 3600 * 0.1)
                self.backbone_path_stability = min(1.0, 
                    self.backbone_path_stability + stability_bonus)
    
    def record_conflict(self, resolved: bool = False):
        """记录冲突事件"""
        self.conflict_count += 1
        self.last_conflict_time = time.time()
        
        if resolved:
            success_count = getattr(self, '_conflict_success_count', 0) + 1
            setattr(self, '_conflict_success_count', success_count)
            self.conflict_resolution_success_rate = success_count / self.conflict_count
    
    def update_position_and_metrics(self, new_position: Tuple[float, float, float], 
                                   time_delta: float):
        """更新位置和相关指标"""
        if self.position and time_delta > 0:
            distance = math.sqrt(
                (new_position[0] - self.position[0])**2 +
                (new_position[1] - self.position[1])**2
            )
            self.total_distance += distance
            self.total_time += time_delta
            
            self.update_average_speed()
            
            if self.status == VehicleStatus.IDLE:
                self.idle_time += time_delta
            else:
                self.productive_time += time_delta
        
        self.position = new_position

@dataclass
class CoordinationRequest:
    """协调请求"""
    request_id: str
    vehicle_requests: Dict[str, Dict]
    coordination_mode: CoordinationMode = CoordinationMode.BATCH_COORDINATION
    max_solve_time: float = 30.0
    quality_threshold: float = 0.7
    prefer_backbone: bool = True
    backbone_priority_weight: float = 0.3
    allow_conflicts: bool = False
    max_conflicts: int = 0

@dataclass
class CoordinationResult:
    """协调结果"""
    request_id: str
    success: bool
    coordinated_paths: Dict[str, List[Tuple]] = field(default_factory=dict)
    path_structures: Dict[str, Dict] = field(default_factory=dict)
    total_cost: float = 0.0
    backbone_utilization: float = 0.0
    conflict_count: int = 0
    solve_time: float = 0.0
    ecbs_expansions: int = 0
    constraints_generated: int = 0
    initial_conflicts: int = 0
    final_conflicts: int = 0
    error_message: Optional[str] = None

class ECBSCoordinator:
    """ECBS协调器 - 集成在调度器中"""
    
    def __init__(self, scheduler, traffic_manager, backbone_network):
        self.scheduler = scheduler
        self.traffic_manager = traffic_manager
        self.backbone_network = backbone_network
        
        # ECBS参数
        self.max_expansions = 500
        self.timeout = 30.0
        self.suboptimality_bound = 1.5
        
        # 协调统计
        self.coordination_stats = {
            'total_requests': 0,
            'successful_coordinations': 0,
            'average_solve_time': 0.0,
            'average_conflict_reduction': 0.0,
            'backbone_utilization_improvement': 0.0,
            'ecbs_expansions_avg': 0.0
        }
        
        # 缓存
        self.coordination_cache = OrderedDict()
        self.cache_limit = 100
        
        print("初始化ECBS协调器")
    
    def coordinate_vehicles(self, coordination_request: CoordinationRequest) -> CoordinationResult:
        """主协调方法"""
        start_time = time.time()
        self.coordination_stats['total_requests'] += 1
        
        print(f"开始ECBS协调: {len(coordination_request.vehicle_requests)} 个车辆")
        
        # 检查缓存
        cache_key = self._generate_cache_key(coordination_request)
        if cache_key in self.coordination_cache:
            cached_result = self.coordination_cache[cache_key]
            print("使用缓存的协调结果")
            return cached_result
        
        # 初始化结果
        result = CoordinationResult(
            request_id=coordination_request.request_id,
            success=False
        )
        
        try:
            # 第一阶段：骨干路径预分配
            backbone_allocations = self._allocate_backbone_paths(coordination_request)
            
            # 第二阶段：ECBS协调求解
            if self.traffic_manager and hasattr(self.traffic_manager, 'ecbs_coordinate_paths'):
                ecbs_result = self.traffic_manager.ecbs_coordinate_paths(
                    coordination_request.vehicle_requests,
                    backbone_allocations,
                    max_solve_time=coordination_request.max_solve_time
                )
                
                if ecbs_result and ecbs_result.get('success', False):
                    result.success = True
                    result.coordinated_paths = ecbs_result.get('paths', {})
                    result.path_structures = ecbs_result.get('structures', {})
                    result.total_cost = ecbs_result.get('total_cost', 0.0)
                    result.conflict_count = ecbs_result.get('final_conflicts', 0)
                    result.ecbs_expansions = ecbs_result.get('expansions', 0)
                    result.constraints_generated = ecbs_result.get('constraints', 0)
                    result.initial_conflicts = ecbs_result.get('initial_conflicts', 0)
                    result.final_conflicts = ecbs_result.get('final_conflicts', 0)
                    
                    # 计算骨干网络利用率
                    result.backbone_utilization = self._calculate_backbone_utilization(
                        result.coordinated_paths
                    )
                    
                    print(f"✅ ECBS协调成功: {result.conflict_count} 个冲突, "
                          f"骨干利用率: {result.backbone_utilization:.2%}")
                else:
                    result.error_message = ecbs_result.get('error', 'ECBS求解失败')
                    print(f"❌ ECBS协调失败: {result.error_message}")
            else:
                # 回退到单车辆规划
                result = self._fallback_individual_planning(coordination_request)
                
        except Exception as e:
            result.error_message = f"协调异常: {str(e)}"
            print(f"协调异常: {e}")
        
        # 记录统计
        result.solve_time = time.time() - start_time
        self._update_coordination_stats(result)
        
        # 缓存结果
        if result.success:
            self._cache_result(cache_key, result)
            self.coordination_stats['successful_coordinations'] += 1
        
        return result
    
    def _allocate_backbone_paths(self, request: CoordinationRequest) -> Dict[str, str]:
        """骨干路径预分配"""
        allocations = {}
        
        if not self.backbone_network or not request.prefer_backbone:
            return allocations
        
        sorted_requests = sorted(
            request.vehicle_requests.items(),
            key=lambda x: x[1].get('priority', 0.5),
            reverse=True
        )
        
        print(f"为 {len(sorted_requests)} 个车辆分配骨干路径")
        
        for vehicle_id, vehicle_request in sorted_requests:
            try:
                if hasattr(self.backbone_network, 'get_path_from_position_to_target'):
                    target_type = 'unloading'
                    target_id = 0
                    
                    path_result = self.backbone_network.get_path_from_position_to_target(
                        vehicle_request['start'],
                        target_type,
                        target_id,
                        vehicle_id
                    )
                    
                    if path_result and isinstance(path_result, tuple) and len(path_result) > 1:
                        path_structure = path_result[1]
                        backbone_path_id = path_structure.get('path_id')
                        if backbone_path_id:
                            allocations[vehicle_id] = backbone_path_id
                            print(f"  车辆 {vehicle_id} -> 骨干路径 {backbone_path_id}")
                        
            except Exception as e:
                print(f"为车辆 {vehicle_id} 分配骨干路径失败: {e}")
        
        return allocations
    
    def _calculate_backbone_utilization(self, paths: Dict[str, List[Tuple]]) -> float:
        """计算骨干网络利用率"""
        if not paths or not self.backbone_network:
            return 0.0
        
        total_backbone_usage = 0.0
        total_vehicles = len(paths)
        
        for vehicle_id, path in paths.items():
            if hasattr(self.backbone_network, 'analyze_path_backbone_usage'):
                try:
                    usage_info = self.backbone_network.analyze_path_backbone_usage(path, vehicle_id)
                    total_backbone_usage += usage_info.get('utilization_ratio', 0.0)
                except:
                    pass
        
        return total_backbone_usage / max(1, total_vehicles)
    
    def _fallback_individual_planning(self, request: CoordinationRequest) -> CoordinationResult:
        """回退到单车辆规划"""
        result = CoordinationResult(
            request_id=request.request_id,
            success=True
        )
        
        print("回退到单车辆规划模式")
        
        for vehicle_id, vehicle_request in request.vehicle_requests.items():
            try:
                if self.scheduler.path_planner:
                    path_result = self.scheduler.path_planner.plan_path(
                        vehicle_id=vehicle_id,
                        start=vehicle_request['start'],
                        goal=vehicle_request['goal'],
                        use_backbone=request.prefer_backbone
                    )
                    
                    if path_result:
                        if isinstance(path_result, tuple):
                            path, structure = path_result
                        else:
                            path = path_result
                            structure = {'type': 'individual'}
                        
                        result.coordinated_paths[vehicle_id] = path
                        result.path_structures[vehicle_id] = structure
                        
            except Exception as e:
                print(f"单车辆规划失败 {vehicle_id}: {e}")
        
        return result
    
    def _generate_cache_key(self, request: CoordinationRequest) -> str:
        """生成缓存键"""
        vehicles_str = "_".join(sorted(request.vehicle_requests.keys()))
        return f"{vehicles_str}_{request.coordination_mode.value}_{request.max_solve_time}"
    
    def _cache_result(self, cache_key: str, result: CoordinationResult):
        """缓存结果"""
        if len(self.coordination_cache) >= self.cache_limit:
            self.coordination_cache.popitem(last=False)
        
        self.coordination_cache[cache_key] = result
    
    def _update_coordination_stats(self, result: CoordinationResult):
        """更新协调统计"""
        stats = self.coordination_stats
        
        total_requests = stats['total_requests']
        if total_requests > 1:
            stats['average_solve_time'] = (
                stats['average_solve_time'] * (total_requests - 1) + result.solve_time
            ) / total_requests
        else:
            stats['average_solve_time'] = result.solve_time
        
        if result.success:
            stats['average_conflict_reduction'] = (
                stats['average_conflict_reduction'] * 0.9 + 
                (max(0, result.initial_conflicts - result.final_conflicts) / max(1, result.initial_conflicts)) * 0.1
            )
            
            stats['backbone_utilization_improvement'] = (
                stats['backbone_utilization_improvement'] * 0.9 + 
                result.backbone_utilization * 0.1
            )
            
            stats['ecbs_expansions_avg'] = (
                stats['ecbs_expansions_avg'] * 0.9 + 
                result.ecbs_expansions * 0.1
            )

class SystemEfficiencyOptimizer:
    """系统效率优化器"""
    
    def __init__(self, env, scheduler):
        self.env = env
        self.scheduler = scheduler
        
        # 优化配置
        self.optimization_config = {
            'efficiency_target': 0.85,
            'rebalancing_threshold': 0.7,
            'optimization_interval': 120.0,
            'max_concurrent_optimizations': 3,
            'load_balancing_weight': 0.4,
            'distance_minimization_weight': 0.3,
            'backbone_utilization_weight': 0.3
        }
        
        # 优化状态
        self.last_optimization_time = 0
        self.active_optimizations = set()
        self.optimization_history = []
        
        # 性能指标
        self.system_metrics = {
            'overall_efficiency': 0.0,
            'vehicle_utilization': 0.0,
            'backbone_utilization': 0.0,
            'average_task_time': 0.0,
            'idle_ratio': 0.0
        }
    
    def should_optimize_system(self) -> bool:
        """判断是否需要系统优化"""
        current_time = time.time()
        
        if current_time - self.last_optimization_time < self.optimization_config['optimization_interval']:
            return False
        
        current_efficiency = self.calculate_system_efficiency()
        if current_efficiency < self.optimization_config['rebalancing_threshold']:
            return True
        
        if self._detect_load_imbalance():
            return True
        
        return False
    
    def calculate_system_efficiency(self) -> float:
        """计算系统总体效率"""
        if not self.scheduler.vehicle_states:
            return 0.0
        
        total_tasks = self.scheduler.stats['total_tasks']
        completed_tasks = self.scheduler.stats['completed_tasks']
        task_completion_rate = completed_tasks / max(1, total_tasks)
        
        vehicle_efficiencies = [v.calculate_efficiency_score() 
                               for v in self.scheduler.vehicle_states.values()]
        avg_vehicle_efficiency = sum(vehicle_efficiencies) / len(vehicle_efficiencies) if vehicle_efficiencies else 0.0
        
        backbone_utilization = self._calculate_backbone_utilization()
        avg_task_efficiency = self._calculate_average_task_efficiency()
        
        system_efficiency = (
            task_completion_rate * 0.3 +
            avg_vehicle_efficiency * 0.3 +
            backbone_utilization * 0.2 +
            avg_task_efficiency * 0.2
        )
        
        self.system_metrics.update({
            'overall_efficiency': system_efficiency,
            'vehicle_utilization': avg_vehicle_efficiency,
            'backbone_utilization': backbone_utilization,
            'average_task_time': avg_task_efficiency
        })
        
        return system_efficiency
    
    def _calculate_backbone_utilization(self) -> float:
        """计算骨干网络利用率"""
        if not self.scheduler.backbone_network:
            return 0.0
        
        total_backbone_usage = 0
        total_vehicles = len(self.scheduler.vehicle_states)
        
        for vehicle_state in self.scheduler.vehicle_states.values():
            total_backbone_usage += vehicle_state.backbone_usage_ratio
        
        return total_backbone_usage / max(1, total_vehicles)
    
    def _calculate_average_task_efficiency(self) -> float:
        """计算平均任务效率"""
        completed_tasks = [task for task in self.scheduler.tasks.values() 
                          if task.status == TaskStatus.COMPLETED]
        
        if not completed_tasks:
            return 0.5
        
        total_efficiency = sum(task.calculate_efficiency() for task in completed_tasks)
        return total_efficiency / len(completed_tasks)
    
    def _detect_load_imbalance(self) -> bool:
        """检测负载不平衡"""
        if not self.scheduler.vehicle_states:
            return False
        
        efficiencies = [v.calculate_efficiency_score() 
                       for v in self.scheduler.vehicle_states.values()]
        
        if len(efficiencies) < 2:
            return False
        
        mean_efficiency = sum(efficiencies) / len(efficiencies)
        variance = sum((e - mean_efficiency) ** 2 for e in efficiencies) / len(efficiencies)
        std_deviation = math.sqrt(variance)
        
        return std_deviation > 0.15
    
    def optimize_system(self) -> Dict[str, Any]:
        """执行系统优化"""
        optimization_start = time.time()
        self.last_optimization_time = optimization_start
        
        print("🚀 开始系统效率优化...")
        
        optimization_results = {
            'vehicle_rebalancing': 0,
            'task_reassignments': 0,
            'backbone_optimizations': 0,
            'efficiency_improvement': 0.0,
            'optimization_time': 0.0
        }
        
        initial_efficiency = self.calculate_system_efficiency()
        
        try:
            rebalancing_result = self._rebalance_vehicle_loads()
            optimization_results['vehicle_rebalancing'] = rebalancing_result
            
            reassignment_result = self._optimize_task_assignments()
            optimization_results['task_reassignments'] = reassignment_result
            
            backbone_result = self._optimize_backbone_usage()
            optimization_results['backbone_optimizations'] = backbone_result
            
        except Exception as e:
            print(f"优化过程出错: {e}")
        
        final_efficiency = self.calculate_system_efficiency()
        efficiency_improvement = final_efficiency - initial_efficiency
        optimization_results['efficiency_improvement'] = efficiency_improvement
        
        optimization_time = time.time() - optimization_start
        optimization_results['optimization_time'] = optimization_time
        
        self.optimization_history.append({
            'timestamp': optimization_start,
            'initial_efficiency': initial_efficiency,
            'final_efficiency': final_efficiency,
            'improvement': efficiency_improvement,
            'results': optimization_results
        })
        
        if len(self.optimization_history) > 50:
            self.optimization_history = self.optimization_history[-25:]
        
        print(f"✅ 系统优化完成: 效率提升 {efficiency_improvement:.2%}, 耗时 {optimization_time:.2f}s")
        
        return optimization_results
    
    def _rebalance_vehicle_loads(self) -> int:
        """重平衡车辆负载"""
        rebalanced_count = 0
        
        vehicle_efficiencies = [(vid, v.calculate_efficiency_score()) 
                               for vid, v in self.scheduler.vehicle_states.items()]
        
        if len(vehicle_efficiencies) < 2:
            return 0
        
        vehicle_efficiencies.sort(key=lambda x: x[1])
        
        low_efficiency_vehicles = vehicle_efficiencies[:len(vehicle_efficiencies)//3]
        high_efficiency_vehicles = vehicle_efficiencies[-len(vehicle_efficiencies)//3:]
        
        for low_vid, low_eff in low_efficiency_vehicles:
            low_vehicle = self.scheduler.vehicle_states[low_vid]
            
            if len(low_vehicle.task_queue) > 1:
                for high_vid, high_eff in high_efficiency_vehicles:
                    high_vehicle = self.scheduler.vehicle_states[high_vid]
                    
                    if len(high_vehicle.task_queue) < len(low_vehicle.task_queue) - 1:
                        task_to_transfer = low_vehicle.task_queue.pop()
                        high_vehicle.task_queue.append(task_to_transfer)
                        
                        if task_to_transfer in self.scheduler.tasks:
                            self.scheduler.tasks[task_to_transfer].assigned_vehicle = high_vid
                        
                        rebalanced_count += 1
                        print(f"  转移任务 {task_to_transfer}: {low_vid} -> {high_vid}")
                        break
        
        return rebalanced_count
    
    def _optimize_task_assignments(self) -> int:
        """优化任务分配"""
        return 0  # 简化实现
    
    def _optimize_backbone_usage(self) -> int:
        """优化骨干网络使用"""
        return 0  # 简化实现

class EnhancedVehicleScheduler:
    """集成ECBS功能的增强版车辆调度器"""
    
    def __init__(self, env, path_planner=None, backbone_network=None, traffic_manager=None):
        # 核心组件
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 增强数据存储
        self.tasks = OrderedDict()
        self.vehicle_states = {}
        self.mission_templates = {}
        
        # 任务管理
        self.task_counter = 0
        self.active_assignments = defaultdict(list)
        self.task_priority_queue = []
        
        # 效率优化器
        self.efficiency_optimizer = SystemEfficiencyOptimizer(env, self)
        
        # 状态锁
        self.state_lock = threading.RLock()
        
        # ECBS协调器 - 集成功能
        self.ecbs_coordinator = ECBSCoordinator(self, traffic_manager, backbone_network)
        
        # 协调配置
        self.coordination_config = {
            'enable_ecbs': True,
            'default_coordination_mode': CoordinationMode.BATCH_COORDINATION,
            'coordination_trigger_threshold': 3,
            'coordination_interval': 120.0,
            'max_coordination_time': 30.0,
            'prefer_backbone_in_coordination': True
        }
        
        # 协调状态
        self.coordination_state = {
            'last_coordination_time': 0,
            'pending_coordination_requests': [],
            'active_coordinations': {},
            'coordination_queue': deque()
        }
        
        # 增强统计
        self.stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'failed_tasks': 0,
            'total_distance': 0,
            'total_efficiency_score': 0.0,
            'backbone_usage_count': 0,
            'optimization_cycles': 0,
            'average_task_time': 0.0,
            'system_efficiency_trend': [],
            # ECBS相关统计
            'ecbs_coordinations': 0,
            'ecbs_success_rate': 0.0,
            'avg_coordination_time': 0.0,
            'conflict_reduction_rate': 0.0,
            'backbone_utilization_improvement': 0.0
        }
        
        # 性能监控
        self.performance_monitor = {
            'last_efficiency_check': 0,
            'efficiency_check_interval': 60.0,
            'performance_alerts': []
        }
        
        print("初始化集成ECBS功能的增强车辆调度器")
    
    # ==================== ECBS协调功能 ====================
    
    def coordinate_multiple_vehicles(self, vehicle_ids: List[str], 
                                   mission_templates: Dict[str, str] = None,
                                   coordination_mode: CoordinationMode = None,
                                   priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """批量协调多个车辆 - ECBS核心功能"""
        
        if not self.coordination_config['enable_ecbs'] or len(vehicle_ids) < 2:
            return self._assign_individual_missions(vehicle_ids, mission_templates, priority)
        
        print(f"开始多车辆协调: {len(vehicle_ids)} 个车辆")
        
        try:
            coordination_request = self._prepare_coordination_request(
                vehicle_ids, mission_templates, coordination_mode, priority
            )
            
            if not coordination_request:
                print("协调请求准备失败，回退到单车辆分配")
                return self._assign_individual_missions(vehicle_ids, mission_templates, priority)
            
            result = self.ecbs_coordinator.coordinate_vehicles(coordination_request)
            
            if result.success:
                return self._apply_coordination_result(result, vehicle_ids, priority)
            else:
                print(f"ECBS协调失败: {result.error_message}")
                return self._assign_individual_missions(vehicle_ids, mission_templates, priority)
                
        except Exception as e:
            print(f"多车辆协调异常: {e}")
            return self._assign_individual_missions(vehicle_ids, mission_templates, priority)
    
    def _prepare_coordination_request(self, vehicle_ids: List[str], 
                                    mission_templates: Dict[str, str],
                                    coordination_mode: CoordinationMode,
                                    priority: TaskPriority) -> Optional[CoordinationRequest]:
        """准备协调请求"""
        
        if coordination_mode is None:
            coordination_mode = self.coordination_config['default_coordination_mode']
        
        vehicle_requests = {}
        
        for vehicle_id in vehicle_ids:
            if vehicle_id not in self.vehicle_states:
                print(f"车辆 {vehicle_id} 不存在")
                continue
            
            vehicle_state = self.vehicle_states[vehicle_id]
            template_id = mission_templates.get(vehicle_id, 'default') if mission_templates else 'default'
            
            if template_id not in self.mission_templates:
                if not self.create_enhanced_mission_template(template_id, priority=priority):
                    print(f"创建任务模板失败: {template_id}")
                    continue
            
            template = self.mission_templates[template_id]
            start = vehicle_state.position
            goal = template['loading_position']
            
            vehicle_requests[vehicle_id] = {
                'start': start,
                'goal': goal,
                'priority': priority.value,
                'deadline': time.time() + 300,
                'template_id': template_id
            }
        
        if not vehicle_requests:
            return None
        
        return CoordinationRequest(
            request_id=f"coord_{int(time.time())}",
            vehicle_requests=vehicle_requests,
            coordination_mode=coordination_mode,
            max_solve_time=self.coordination_config['max_coordination_time'],
            prefer_backbone=self.coordination_config['prefer_backbone_in_coordination']
        )
    
    def _apply_coordination_result(self, result: CoordinationResult, 
                                 vehicle_ids: List[str], priority: TaskPriority) -> bool:
        """应用协调结果"""
        
        success_count = 0
        
        with self.state_lock:
            for vehicle_id in vehicle_ids:
                if vehicle_id not in result.coordinated_paths:
                    continue
                
                try:
                    if self._create_coordinated_task(
                        vehicle_id, result.coordinated_paths[vehicle_id],
                        result.path_structures.get(vehicle_id, {}), priority
                    ):
                        success_count += 1
                        
                except Exception as e:
                    print(f"应用协调结果失败 {vehicle_id}: {e}")
        
        # 更新统计
        if success_count > 0:
            self.stats['ecbs_coordinations'] += 1
            self.stats['ecbs_success_rate'] = (
                self.stats['ecbs_success_rate'] * 0.9 + 
                (success_count / len(vehicle_ids)) * 0.1
            )
            self.stats['avg_coordination_time'] = (
                self.stats['avg_coordination_time'] * 0.9 + 
                result.solve_time * 0.1
            )
            self.stats['backbone_utilization_improvement'] = (
                self.stats['backbone_utilization_improvement'] * 0.9 + 
                result.backbone_utilization * 0.1
            )
        
        print(f"协调结果应用完成: {success_count}/{len(vehicle_ids)} 成功")
        return success_count > 0
    
    def _create_coordinated_task(self, vehicle_id: str, coordinated_path: List[Tuple],
                               path_structure: Dict, priority: TaskPriority) -> bool:
        """创建协调任务"""
        
        task_id = f"coordinated_task_{self.task_counter}"
        self.task_counter += 1
        
        task = EnhancedTask(
            task_id=task_id,
            task_type='coordinated_mission',
            start=coordinated_path[0],
            goal=coordinated_path[-1],
            priority=priority,
            path=coordinated_path,
            path_structure=path_structure
        )
        
        if path_structure:
            task.backbone_utilization = path_structure.get('backbone_utilization', 0.0)
        
        self.tasks[task_id] = task
        self.active_assignments[vehicle_id].append(task_id)
        self.stats['total_tasks'] += 1
        
        vehicle_state = self.vehicle_states[vehicle_id]
        vehicle_state.task_queue.append(task_id)
        
        if vehicle_state.status == VehicleStatus.IDLE:
            return self._start_task_execution(task_id, vehicle_id)
        
        return True
    
    def _assign_individual_missions(self, vehicle_ids: List[str], 
                                  mission_templates: Dict[str, str],
                                  priority: TaskPriority) -> bool:
        """回退到单车辆任务分配"""
        
        success_count = 0
        
        for vehicle_id in vehicle_ids:
            template_id = mission_templates.get(vehicle_id, 'default') if mission_templates else 'default'
            
            if self.assign_mission_intelligently(vehicle_id, template_id, priority):
                success_count += 1
        
        return success_count > 0
    
    def trigger_periodic_coordination(self) -> bool:
        """触发定期协调"""
        
        current_time = time.time()
        
        if (current_time - self.coordination_state['last_coordination_time'] < 
            self.coordination_config['coordination_interval']):
            return False
        
        coordination_candidates = self._find_coordination_candidates()
        
        if len(coordination_candidates) >= self.coordination_config['coordination_trigger_threshold']:
            print(f"触发定期协调: {len(coordination_candidates)} 个车辆")
            
            success = self.coordinate_multiple_vehicles(
                coordination_candidates,
                coordination_mode=CoordinationMode.PERIODIC_COORDINATION
            )
            
            self.coordination_state['last_coordination_time'] = current_time
            return success
        
        return False
    
    def _find_coordination_candidates(self) -> List[str]:
        """找到协调候选车辆"""
        candidates = []
        
        for vehicle_id, vehicle_state in self.vehicle_states.items():
            if (vehicle_state.status == VehicleStatus.IDLE or 
                (vehicle_state.current_task and 
                 self._is_task_near_completion(vehicle_state.current_task))):
                candidates.append(vehicle_id)
        
        return candidates
    
    def _is_task_near_completion(self, task_id: str) -> bool:
        """判断任务是否接近完成"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        
        if task.estimated_duration > 0:
            elapsed = time.time() - task.start_time
            return elapsed > task.estimated_duration * 0.8
        
        return False
    
    def get_coordination_statistics(self) -> Dict:
        """获取协调统计信息"""
        ecbs_stats = self.ecbs_coordinator.coordination_stats.copy()
        
        return {
            'ecbs_coordinator': ecbs_stats,
            'coordination_config': self.coordination_config.copy(),
            'coordination_state': {
                'last_coordination_time': self.coordination_state['last_coordination_time'],
                'pending_requests': len(self.coordination_state['pending_coordination_requests']),
                'active_coordinations': len(self.coordination_state['active_coordinations'])
            },
            'performance_metrics': {
                'ecbs_coordinations': self.stats['ecbs_coordinations'],
                'ecbs_success_rate': self.stats['ecbs_success_rate'],
                'avg_coordination_time': self.stats['avg_coordination_time'],
                'conflict_reduction_rate': self.stats['conflict_reduction_rate'],
                'backbone_utilization_improvement': self.stats['backbone_utilization_improvement']
            }
        }
    
    def enable_ecbs_coordination(self, enable: bool = True):
        """启用/禁用ECBS协调"""
        self.coordination_config['enable_ecbs'] = enable
        print(f"ECBS协调: {'启用' if enable else '禁用'}")
    
    def set_coordination_parameters(self, **kwargs):
        """设置协调参数"""
        for key, value in kwargs.items():
            if key in self.coordination_config:
                self.coordination_config[key] = value
                print(f"更新协调参数 {key}: {value}")
    
    # ==================== 基础调度功能 ====================
    
    def initialize_vehicles(self):
        """初始化车辆状态"""
        with self.state_lock:
            for vehicle_id, vehicle_data in self.env.vehicles.items():
                try:
                    if hasattr(vehicle_data, 'position'):
                        position = vehicle_data.position
                    else:
                        position = vehicle_data.get('position', (0, 0, 0))
                    
                    if hasattr(vehicle_data, 'max_load'):
                        max_load = vehicle_data.max_load
                    else:
                        max_load = vehicle_data.get('max_load', 100)
                    
                    if hasattr(vehicle_data, 'current_load'):
                        current_load = vehicle_data.current_load
                    else:
                        current_load = vehicle_data.get('load', 0)
                    
                    # 获取安全参数
                    if hasattr(vehicle_data, 'safety_params'):
                        safety_params = vehicle_data.safety_params
                        vehicle_length = safety_params.length
                        vehicle_width = safety_params.width
                        safety_margin = safety_params.safety_margin
                        turning_radius = safety_params.turning_radius
                    else:
                        vehicle_length = 6.0
                        vehicle_width = 3.0
                        safety_margin = 1.5
                        turning_radius = 8.0
                    
                    self.vehicle_states[vehicle_id] = EnhancedVehicleState(
                        vehicle_id=vehicle_id,
                        position=position,
                        max_load=max_load,
                        current_load=current_load,
                        priority_level=0.5,
                        vehicle_length=vehicle_length,
                        vehicle_width=vehicle_width,
                        safety_margin=safety_margin,
                        turning_radius=turning_radius
                    )
                    
                    self.active_assignments[vehicle_id] = []
                    
                    if self.traffic_manager:
                        self.traffic_manager.set_vehicle_priority(vehicle_id, 0.5)
                    
                except Exception as e:
                    print(f"初始化车辆 {vehicle_id} 失败: {e}")
                    self.vehicle_states[vehicle_id] = EnhancedVehicleState(
                        vehicle_id=vehicle_id,
                        position=(10, 10, 0),
                        max_load=100,
                        current_load=0
                    )
                    self.active_assignments[vehicle_id] = []
        
        print(f"初始化了 {len(self.vehicle_states)} 个车辆状态")
    
    def create_enhanced_mission_template(self, template_id: str, 
                                    loading_point_id: int = None, 
                                    unloading_point_id: int = None,
                                    priority: TaskPriority = TaskPriority.NORMAL,
                                    randomize: bool = True) -> bool:
        """创建增强任务模板 - 支持随机化"""
        try:
            if not self.env.loading_points or not self.env.unloading_points:
                print("缺少装载点或卸载点")
                return False
            
            # 随机选择装载点和卸载点
            if randomize:
                if loading_point_id is None:
                    loading_point_id = random.randint(0, len(self.env.loading_points) - 1)
                if unloading_point_id is None:
                    unloading_point_id = random.randint(0, len(self.env.unloading_points) - 1)
            else:
                # 非随机模式，使用默认值
                if loading_point_id is None:
                    loading_point_id = 0
                if unloading_point_id is None:
                    unloading_point_id = 0
            
            if (loading_point_id >= len(self.env.loading_points) or 
                unloading_point_id >= len(self.env.unloading_points)):
                print(f"点ID超出范围: L{loading_point_id}, U{unloading_point_id}")
                return False
            
            loading_point = self.env.loading_points[loading_point_id]
            unloading_point = self.env.unloading_points[unloading_point_id]
            
            template = {
                'loading_point_id': loading_point_id,
                'unloading_point_id': unloading_point_id,
                'loading_position': loading_point,
                'unloading_position': unloading_point,
                'priority': priority,
                'tasks': [
                    {
                        'task_type': 'to_loading',
                        'goal': loading_point,
                        'estimated_duration': 180,
                        'priority': priority
                    },
                    {
                        'task_type': 'to_unloading', 
                        'goal': unloading_point,
                        'estimated_duration': 150,
                        'priority': priority
                    },
                    {
                        'task_type': 'to_initial',
                        'goal': None,
                        'estimated_duration': 120,
                        'priority': TaskPriority.LOW
                    }
                ]
            }
            
            self.mission_templates[template_id] = template
            print(f"创建任务模板 {template_id}: L{loading_point_id} -> U{unloading_point_id}")
            return True
            
        except Exception as e:
            print(f"创建任务模板失败: {e}")
            return False
    def handle_parking_decision(self, vehicle_id: str, parking_decision: Dict):
        """处理停车决策"""
        if vehicle_id in self.vehicle_states:
            vehicle_state = self.vehicle_states[vehicle_id]
            
            # 更新车辆状态为等待
            vehicle_state.status = VehicleStatus.WAITING
            
            # 记录停车信息
            vehicle_state.parking_info = {
                'start_time': parking_decision.get('start_time'),
                'duration': parking_decision.get('parking_duration', 15.0),
                'position': parking_decision.get('parking_position'),
                'reason': 'conflict_avoidance'
            }
            
            # 同步到环境
            self._sync_vehicle_status_to_env(vehicle_id, 'waiting')    
    def assign_mission_intelligently(self, vehicle_id: str = None, 
                                   template_id: str = "default",
                                   priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """智能任务分配"""
        with self.state_lock:
            try:
                if vehicle_id is None:
                    vehicle_id = self._select_optimal_vehicle(priority)
                    if not vehicle_id:
                        print("没有可用车辆")
                        return False
                
                if vehicle_id not in self.vehicle_states:
                    print(f"车辆 {vehicle_id} 不存在")
                    return False
                
                if template_id not in self.mission_templates:
                    if not self.create_enhanced_mission_template(template_id, priority=priority):
                        print("创建任务模板失败")
                        return False
                
                template = self.mission_templates[template_id]
                vehicle_state = self.vehicle_states[vehicle_id]
                
                created_tasks = []
                current_position = vehicle_state.position
                current_time = time.time()
                
                for task_template in template['tasks']:
                    task_id = f"task_{self.task_counter}"
                    self.task_counter += 1
                    
                    if task_template['goal'] is None:
                        goal = vehicle_state.position
                    else:
                        goal = task_template['goal']
                    
                    deadline = current_time + task_template['estimated_duration'] * 2
                    
                    task = EnhancedTask(
                        task_id=task_id,
                        task_type=task_template['task_type'],
                        start=current_position,
                        goal=goal,
                        priority=task_template.get('priority', priority),
                        deadline=deadline,
                        estimated_duration=task_template['estimated_duration']
                    )
                    
                    self.tasks[task_id] = task
                    created_tasks.append(task_id)
                    current_position = goal
                    
                    heapq.heappush(self.task_priority_queue, (-task.priority.value, task_id))
                
                self.active_assignments[vehicle_id].extend(created_tasks)
                self.stats['total_tasks'] += len(created_tasks)
                
                self._update_vehicle_priority(vehicle_id, priority)
                
                if vehicle_state.status == VehicleStatus.IDLE:
                    self._start_next_task(vehicle_id)
                
                print(f"为车辆 {vehicle_id} 分配了 {len(created_tasks)} 个任务")
                return True
                
            except Exception as e:
                print(f"智能任务分配失败: {e}")
                return False
    
    def _select_optimal_vehicle(self, task_priority: TaskPriority) -> Optional[str]:
        """选择最优车辆"""
        available_vehicles = [(vid, v) for vid, v in self.vehicle_states.items() 
                             if v.status == VehicleStatus.IDLE]
        
        if not available_vehicles:
            return None
        
        best_vehicle = None
        best_score = float('inf')
        
        for vehicle_id, vehicle_state in available_vehicles:
            efficiency_score = 1.0 - vehicle_state.calculate_efficiency_score()
            queue_score = len(vehicle_state.task_queue) / 10.0
            priority_diff = abs(vehicle_state.priority_level - task_priority.value)
            idle_bonus = min(1.0, vehicle_state.idle_time / 300.0)
            
            total_score = (efficiency_score * 0.4 + 
                          queue_score * 0.3 + 
                          priority_diff * 0.2 - 
                          idle_bonus * 0.1)
            
            if total_score < best_score:
                best_score = total_score
                best_vehicle = vehicle_id
        
        return best_vehicle
    
    def _update_vehicle_priority(self, vehicle_id: str, task_priority: TaskPriority):
        """更新车辆优先级"""
        vehicle_state = self.vehicle_states[vehicle_id]
        
        new_priority = (vehicle_state.priority_level * 0.7 + task_priority.value * 0.3)
        vehicle_state.priority_level = min(1.0, max(0.1, new_priority))
        
        if self.traffic_manager:
            self.traffic_manager.set_vehicle_priority(vehicle_id, vehicle_state.priority_level)
    
    def _start_next_task(self, vehicle_id: str) -> bool:
        """开始下一个任务"""
        if vehicle_id not in self.active_assignments:
            return False
        
        assignments = self.active_assignments[vehicle_id]
        if not assignments:
            return False
        
        next_task_id = None
        highest_priority = -1
        
        for task_id in assignments:
            if task_id in self.tasks and self.tasks[task_id].status == TaskStatus.PENDING:
                task_priority = self.tasks[task_id].priority.value
                if task_priority > highest_priority:
                    highest_priority = task_priority
                    next_task_id = task_id
        
        if not next_task_id:
            return False
        
        return self._start_task_execution(next_task_id, vehicle_id)
    
    def _start_task_execution(self, task_id: str, vehicle_id: str) -> bool:
        """开始任务执行"""
        if task_id not in self.tasks:
            return False
        
        task = self.tasks[task_id]
        vehicle_state = self.vehicle_states[vehicle_id]
        
        task.status = TaskStatus.IN_PROGRESS
        task.assigned_vehicle = vehicle_id
        task.assignment_time = time.time()
        task.start_time = time.time()
        
        vehicle_state.status = VehicleStatus.PLANNING
        vehicle_state.current_task = task_id
        
        try:
            if vehicle_id in self.env.vehicles:
                if hasattr(self.env.vehicles[vehicle_id], 'status'):
                    self.env.vehicles[vehicle_id].status = 'planning'
                else:
                    self.env.vehicles[vehicle_id]['status'] = 'planning'
        except Exception as e:
            print(f"同步环境状态失败: {e}")
        
        return self._plan_and_start_enhanced_task(task, vehicle_state)
    
    def _plan_and_start_enhanced_task(self, task: EnhancedTask, 
                                    vehicle_state: EnhancedVehicleState) -> bool:
        """规划并开始增强任务"""
        if not self.path_planner:
            print(f"无路径规划器，任务 {task.task_id} 失败")
            return False
        
        try:
            task.planning_attempts += 1
            
            # 根据任务优先级选择上下文
            context = "navigation"
            if task.priority in [TaskPriority.URGENT, TaskPriority.CRITICAL]:
                context = "emergency"
            elif task.priority == TaskPriority.HIGH:
                context = "navigation"  # 注意：不要用"backbone"避免循环
            
            # 获取车辆安全参数
            vehicle_params = vehicle_state.get_safety_params()
            
            # ===== 解析任务的目标类型和ID =====
            target_type = None
            target_id = None
            
            # 根据任务类型推断目标信息
            if task.task_type in ['to_loading', 'loading']:
                target_type = 'loading'
                # 从目标坐标匹配装载点ID
                if self.env and self.env.loading_points:
                    for i, point in enumerate(self.env.loading_points):
                        if self._is_same_position(task.goal, point):
                            target_id = i
                            print(f"  任务 {task.task_id}: 目标是装载点 {i}")
                            break
                        
            elif task.task_type in ['to_unloading', 'unloading']:
                target_type = 'unloading'
                # 从目标坐标匹配卸载点ID
                if self.env and self.env.unloading_points:
                    for i, point in enumerate(self.env.unloading_points):
                        if self._is_same_position(task.goal, point):
                            target_id = i
                            print(f"  任务 {task.task_id}: 目标是卸载点 {i}")
                            break
                            
            elif task.task_type in ['to_parking', 'parking', 'to_initial']:
                target_type = 'parking'
                # 从目标坐标匹配停车点ID
                if self.env and hasattr(self.env, 'parking_areas'):
                    for i, point in enumerate(self.env.parking_areas):
                        if self._is_same_position(task.goal, point):
                            target_id = i
                            print(f"  任务 {task.task_id}: 目标是停车点 {i}")
                            break
            
            # 如果没有匹配到具体点，打印警告
            if target_type and target_id is None:
                print(f"  ⚠️ 任务 {task.task_id}: 无法匹配目标点ID，目标位置={task.goal}")
            
            print(f"规划任务 {task.task_id}: {vehicle_state.vehicle_id} "
                f"从 {task.start} 到 {task.goal} "
                f"(类型={task.task_type}, 目标={target_type}_{target_id if target_id is not None else '?'})")
            
            # ===== 调用路径规划器 =====
            result = self.path_planner.plan_path(
                vehicle_state.vehicle_id,
                task.start,
                task.goal,
                use_backbone=True,  # 启用骨干网络
                context=context,
                vehicle_params=vehicle_params,
                target_type=target_type,    # 传递目标类型
                target_id=target_id,        # 传递目标ID
                check_conflicts=True,
                return_object=False  # 返回(path, structure)格式
            )
            
            if result:
                # 处理返回结果
                if isinstance(result, tuple) and len(result) >= 2:
                    task.path, task.path_structure = result[0], result[1]
                else:
                    task.path = result
                    task.path_structure = {'type': 'direct'}
                
                # 更新任务的骨干利用率信息
                if task.path_structure:
                    task.backbone_utilization = task.path_structure.get('backbone_utilization', 0.0)
                    
                    # 更新车辆的骨干使用率统计
                    vehicle_state.backbone_usage_ratio = (
                        vehicle_state.backbone_usage_ratio * 0.8 + 
                        task.backbone_utilization * 0.2
                    )
                    
                    # 记录骨干路径ID（如果有）
                    backbone_path_id = task.path_structure.get('path_id')
                    if backbone_path_id:
                        vehicle_state.update_backbone_stability(backbone_path_id)
                    
                    print(f"  路径规划成功: 类型={task.path_structure.get('type', 'unknown')}, "
                        f"长度={len(task.path)}, "
                        f"骨干利用率={task.backbone_utilization:.2%}")
                
                # 在交通管理器中注册路径
                if self.traffic_manager:
                    self.traffic_manager.register_vehicle_path(
                        vehicle_state.vehicle_id, task.path, task.start_time
                    )
                
                # 开始移动
                return self._start_enhanced_movement(task, vehicle_state)
            else:
                print(f"  ❌ 路径规划失败")
                
        except Exception as e:
            print(f"任务 {task.task_id} 规划异常: {e}")
            import traceback
            traceback.print_exc()
        
        # 规划失败处理
        task.status = TaskStatus.FAILED
        vehicle_state.status = VehicleStatus.IDLE
        vehicle_state.current_task = None
        self.stats['failed_tasks'] += 1
        
        # 同步状态到环境
        self._sync_vehicle_status_to_env(vehicle_state.vehicle_id, 'idle')
        
        return False

    def _is_same_position(self, pos1: Tuple, pos2: Tuple, tolerance: float = 2.0) -> bool:
        """判断两个位置是否相同（辅助方法）"""
        if not pos1 or not pos2:
            return False
        
        # 只比较x,y坐标
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        
        return dx < tolerance and dy < tolerance
    
    def _start_enhanced_movement(self, task: EnhancedTask, 
                               vehicle_state: EnhancedVehicleState) -> bool:
        """开始增强移动"""
        if not task.path:
            return False
        
        vehicle_state.status = VehicleStatus.MOVING
        
        backbone_id = None
        if task.path_structure:
            backbone_id = task.path_structure.get('path_id')
        
        vehicle_state.update_backbone_stability(backbone_id)
        
        try:
            if vehicle_state.vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
                
                if hasattr(env_vehicle, 'status'):
                    env_vehicle.status = 'moving'
                    env_vehicle.path = task.path
                    env_vehicle.path_structure = task.path_structure or {}
                else:
                    env_vehicle['status'] = 'moving'
                    env_vehicle['path'] = task.path
                    env_vehicle['path_structure'] = task.path_structure or {}
                
        except Exception as e:
            print(f"同步环境失败: {e}")
        
        print(f"车辆 {vehicle_state.vehicle_id} 开始任务 {task.task_id}")
        return True
    
    def update(self, time_delta: float):
        """主更新循环"""
        current_time = time.time()
        
        with self.state_lock:
            # 更新所有车辆
            for vehicle_id, vehicle_state in self.vehicle_states.items():
                self._update_enhanced_vehicle(vehicle_id, vehicle_state, time_delta)
            
            # 定期性能检查
            if (current_time - self.performance_monitor['last_efficiency_check'] > 
                self.performance_monitor['efficiency_check_interval']):
                
                self._perform_efficiency_check()
                self.performance_monitor['last_efficiency_check'] = current_time
            
            # 系统优化检查
            if self.efficiency_optimizer.should_optimize_system():
                try:
                    optimization_result = self.efficiency_optimizer.optimize_system()
                    self.stats['optimization_cycles'] += 1
                except Exception as e:
                    print(f"系统优化失败: {e}")
            
            # ECBS定期协调检查
            try:
                if self.coordination_config['enable_ecbs']:
                    self.trigger_periodic_coordination()
            except Exception as e:
                print(f"定期协调检查失败: {e}")
    
    def _update_enhanced_vehicle(self, vehicle_id: str, vehicle_state: EnhancedVehicleState, 
                               time_delta: float):
        """更新增强车辆"""
        try:
            if vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_id]
                
                if hasattr(env_vehicle, 'position'):
                    new_position = env_vehicle.position
                else:
                    new_position = env_vehicle.get('position', vehicle_state.position)
                
                vehicle_state.update_position_and_metrics(new_position, time_delta)
                
                if hasattr(env_vehicle, 'current_load'):
                    vehicle_state.current_load = env_vehicle.current_load
                else:
                    vehicle_state.current_load = env_vehicle.get('load', vehicle_state.current_load)
                
                if hasattr(env_vehicle, 'status'):
                    env_status = env_vehicle.status
                else:
                    env_status = env_vehicle.get('status', 'idle')
                
                status_map = {
                    'idle': VehicleStatus.IDLE,
                    'moving': VehicleStatus.MOVING,
                    'loading': VehicleStatus.LOADING,
                    'unloading': VehicleStatus.UNLOADING,
                    'planning': VehicleStatus.PLANNING,
                    'waiting': VehicleStatus.WAITING
                }
                
                if not vehicle_state.current_task:
                    vehicle_state.status = status_map.get(env_status, VehicleStatus.IDLE)
            
            if vehicle_state.current_task:
                self._update_enhanced_task_execution(vehicle_state, time_delta)
            elif vehicle_state.status == VehicleStatus.IDLE:
                self._start_next_task(vehicle_id)
                
        except Exception as e:
            print(f"更新车辆 {vehicle_id} 失败: {e}")
    
    def _update_enhanced_task_execution(self, vehicle_state: EnhancedVehicleState, 
                                      time_delta: float):
        """更新任务执行"""
        task_id = vehicle_state.current_task
        if task_id not in self.tasks:
            return
        
        task = self.tasks[task_id]
        
        if vehicle_state.status == VehicleStatus.MOVING:
            self._update_enhanced_movement(task, vehicle_state, time_delta)
        elif vehicle_state.status in [VehicleStatus.LOADING, VehicleStatus.UNLOADING]:
            self._update_enhanced_operation(task, vehicle_state, time_delta)
    
    def _update_enhanced_movement(self, task: EnhancedTask, 
                                vehicle_state: EnhancedVehicleState, time_delta: float):
        """更新移动状态"""
        try:
            if vehicle_state.vehicle_id not in self.env.vehicles:
                return
            
            env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
            
            if hasattr(env_vehicle, 'progress'):
                progress = env_vehicle.progress
            else:
                progress = env_vehicle.get('progress', 0.0)
            
            if task.path and len(task.path) > 1:
                path_length = self._calculate_path_length(task.path)
                if path_length > 0:
                    speed = vehicle_state.speed
                    distance_increment = speed * time_delta
                    progress_increment = distance_increment / path_length
                    new_progress = min(1.0, progress + progress_increment)
                    
                    if hasattr(env_vehicle, 'progress'):
                        env_vehicle.progress = new_progress
                    else:
                        env_vehicle['progress'] = new_progress
                    
                    if new_progress < 1.0:
                        new_position = self._interpolate_position(task.path, new_progress)
                        if hasattr(env_vehicle, 'position'):
                            env_vehicle.position = new_position
                        else:
                            env_vehicle['position'] = new_position
                        vehicle_state.position = new_position
                    
                    if new_progress >= 0.95:
                        self._handle_enhanced_arrival(task, vehicle_state)
                        
        except Exception as e:
            print(f"更新移动失败: {e}")
    
    def _update_enhanced_operation(self, task: EnhancedTask, 
                                 vehicle_state: EnhancedVehicleState, time_delta: float):
        """更新操作进度"""
        operation_time = 60 if vehicle_state.status == VehicleStatus.LOADING else 40
        elapsed = time.time() - task.start_time
        
        if elapsed >= operation_time:
            if vehicle_state.status == VehicleStatus.LOADING:
                vehicle_state.current_load = vehicle_state.max_load
                self._sync_vehicle_load_to_env(vehicle_state.vehicle_id, vehicle_state.max_load)
                print(f"车辆 {vehicle_state.vehicle_id} 装载完成")
            else:
                vehicle_state.current_load = 0
                self._sync_vehicle_load_to_env(vehicle_state.vehicle_id, 0)
                print(f"车辆 {vehicle_state.vehicle_id} 卸载完成")
            
            self._complete_enhanced_task(task, vehicle_state)
    
    def _sync_vehicle_load_to_env(self, vehicle_id: str, load: float):
        """同步车辆负载到环境"""
        try:
            if vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_id]
                if hasattr(env_vehicle, 'current_load'):
                    env_vehicle.current_load = load
                else:
                    env_vehicle['load'] = load
        except Exception as e:
            print(f"同步负载失败: {e}")
    
    def _handle_enhanced_arrival(self, task: EnhancedTask, 
                               vehicle_state: EnhancedVehicleState):
        """处理到达事件"""
        if task.task_type == 'to_loading':
            vehicle_state.status = VehicleStatus.LOADING
            self._sync_vehicle_status_to_env(vehicle_state.vehicle_id, 'loading')
            print(f"车辆 {vehicle_state.vehicle_id} 到达装载点")
            
        elif task.task_type == 'to_unloading':
            vehicle_state.status = VehicleStatus.UNLOADING
            self._sync_vehicle_status_to_env(vehicle_state.vehicle_id, 'unloading')
            print(f"车辆 {vehicle_state.vehicle_id} 到达卸载点")
            
        elif task.task_type == 'to_initial':
            self._complete_enhanced_task(task, vehicle_state)
            self._increment_enhanced_cycle(vehicle_state)
            self._auto_assign_next_cycle(vehicle_state.vehicle_id)
        
        task.start_time = time.time()
    
    def _sync_vehicle_status_to_env(self, vehicle_id: str, status: str):
        """同步车辆状态到环境"""
        try:
            if vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_id]
                if hasattr(env_vehicle, 'status'):
                    env_vehicle.status = status
                else:
                    env_vehicle['status'] = status
        except Exception as e:
            print(f"同步状态失败: {e}")
    
    def _complete_enhanced_task(self, task: EnhancedTask, 
                              vehicle_state: EnhancedVehicleState):
        """完成任务"""
        task.status = TaskStatus.COMPLETED
        task.completion_time = time.time()
        task.actual_duration = task.completion_time - task.start_time
        
        task.efficiency_score = task.calculate_efficiency()
        vehicle_state.update_efficiency_history(task.efficiency_score)
        
        self.stats['completed_tasks'] += 1
        self.stats['total_efficiency_score'] += task.efficiency_score
        
        if task.task_id in self.active_assignments[vehicle_state.vehicle_id]:
            self.active_assignments[vehicle_state.vehicle_id].remove(task.task_id)
        
        if self.traffic_manager:
            self.traffic_manager.release_vehicle_path(vehicle_state.vehicle_id)
        
        vehicle_state.current_task = None
        vehicle_state.status = VehicleStatus.IDLE
        vehicle_state.task_completion_count += 1
        
        self._sync_vehicle_status_to_env(vehicle_state.vehicle_id, 'idle')
        
        print(f"任务 {task.task_id} 完成，效率: {task.efficiency_score:.2f}")
        
        self._start_next_task(vehicle_state.vehicle_id)
    
    def _increment_enhanced_cycle(self, vehicle_state: EnhancedVehicleState):
        """增加循环计数"""
        vehicle_state.completed_cycles += 1
        
        try:
            if vehicle_state.vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_state.vehicle_id]
                if hasattr(env_vehicle, 'completed_cycles'):
                    env_vehicle.completed_cycles = vehicle_state.completed_cycles
                else:
                    env_vehicle['completed_cycles'] = vehicle_state.completed_cycles
        except Exception as e:
            print(f"同步循环计数失败: {e}")
        
        print(f"车辆 {vehicle_state.vehicle_id} 完成第 {vehicle_state.completed_cycles} 个循环")
        
    def _auto_assign_next_cycle(self, vehicle_id: str):
        """自动分配下一个循环"""
        if not self.active_assignments[vehicle_id]:
            vehicle_state = self.vehicle_states[vehicle_id]
            efficiency = vehicle_state.calculate_efficiency_score()
            
            if efficiency > 0.8:
                priority = TaskPriority.HIGH
            elif efficiency > 0.6:
                priority = TaskPriority.NORMAL
            else:
                priority = TaskPriority.LOW
            
            self.assign_mission_intelligently(vehicle_id, "default", priority)
    
    def _perform_efficiency_check(self):
        """执行效率检查"""
        try:
            current_efficiency = self.efficiency_optimizer.calculate_system_efficiency()
            
            self.stats['system_efficiency_trend'].append({
                'timestamp': time.time(),
                'efficiency': current_efficiency
            })
            
            if len(self.stats['system_efficiency_trend']) > 100:
                self.stats['system_efficiency_trend'] = self.stats['system_efficiency_trend'][-50:]
            
            if current_efficiency < 0.6:
                alert = {
                    'timestamp': time.time(),
                    'type': 'low_efficiency',
                    'value': current_efficiency,
                    'message': f'系统效率偏低: {current_efficiency:.2%}'
                }
                
                self.performance_monitor['performance_alerts'].append(alert)
                print(f"⚠️ 效率警报: {alert['message']}")
        except Exception as e:
            print(f"效率检查失败: {e}")
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径总长度"""
        if not path or len(path) < 2:
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
        
        total_length = self._calculate_path_length(path)
        target_distance = total_length * progress
        
        current_distance = 0.0
        for i in range(len(path) - 1):
            segment_length = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            
            if current_distance + segment_length >= target_distance:
                remaining = target_distance - current_distance
                ratio = remaining / segment_length if segment_length > 0 else 0
                
                x = path[i][0] + ratio * (path[i+1][0] - path[i][0])
                y = path[i][1] + ratio * (path[i+1][1] - path[i][1])
                theta = path[i][2] if len(path[i]) > 2 else 0
                
                return (x, y, theta)
            
            current_distance += segment_length
        
        return path[-1]
    
    # ==================== 外部接口 ====================
    
    def handle_conflict_resolution_result(self, vehicle_id: str, resolution_success: bool):
        """处理冲突解决结果"""
        if vehicle_id in self.vehicle_states:
            vehicle_state = self.vehicle_states[vehicle_id]
            vehicle_state.record_conflict(resolution_success)
            
            print(f"车辆 {vehicle_id} 冲突解决: {'成功' if resolution_success else '失败'}")
    
    def get_vehicle_safety_report(self, vehicle_id: str) -> Dict:
        """获取车辆安全报告"""
        if vehicle_id not in self.vehicle_states:
            return {}
        
        vehicle_state = self.vehicle_states[vehicle_id]
        
        return {
            'vehicle_id': vehicle_id,
            'safety_params': vehicle_state.get_safety_params(),
            'backbone_stability': {
                'score': vehicle_state.backbone_path_stability,
                'switch_count': vehicle_state.backbone_switch_count,
                'current_backbone_id': vehicle_state.last_backbone_path_id
            },
            'conflict_stats': {
                'total_conflicts': vehicle_state.conflict_count,
                'resolution_success_rate': vehicle_state.conflict_resolution_success_rate
            }
        }
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息"""
        stats = self.stats.copy()
        
        active_vehicles = len([v for v in self.vehicle_states.values() 
                             if v.status != VehicleStatus.IDLE])
        idle_vehicles = len([v for v in self.vehicle_states.values() 
                           if v.status == VehicleStatus.IDLE])
        
        stats['real_time'] = {
            'active_vehicles': active_vehicles,
            'idle_vehicles': idle_vehicles,
            'total_vehicles': len(self.vehicle_states)
        }
        
        try:
            current_efficiency = self.efficiency_optimizer.calculate_system_efficiency()
            stats['efficiency_metrics'] = {
                'current_system_efficiency': current_efficiency,
                'target_efficiency': self.efficiency_optimizer.optimization_config['efficiency_target'],
                'optimization_cycles': self.stats['optimization_cycles']
            }
        except Exception as e:
            print(f"获取效率指标失败: {e}")
            stats['efficiency_metrics'] = {
                'current_system_efficiency': 0.5,
                'target_efficiency': 0.85,
                'optimization_cycles': 0
            }
        
        # 添加ECBS协调统计
        coordination_stats = self.get_coordination_statistics()
        stats['coordination'] = coordination_stats
        
        return stats
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.ecbs_coordinator.backbone_network = backbone_network
    
    def get_efficiency_report(self) -> Dict:
        """获取效率报告"""
        return {
            'system_efficiency': self.efficiency_optimizer.calculate_system_efficiency(),
            'system_metrics': self.efficiency_optimizer.system_metrics,
            'optimization_history': self.efficiency_optimizer.optimization_history[-10:],
            'coordination_efficiency': self.get_coordination_statistics()
        }
    
    def shutdown(self):
        """关闭调度器"""
        with self.state_lock:
            self.tasks.clear()
            self.vehicle_states.clear()
            self.active_assignments.clear()
            self.task_priority_queue.clear()
            self.coordination_state['active_coordinations'].clear()
            self.coordination_state['pending_coordination_requests'].clear()
        
        print("集成ECBS功能的车辆调度器已关闭")

# 向后兼容性别名
SimplifiedVehicleScheduler = EnhancedVehicleScheduler
HybridAStarIntegratedScheduler = EnhancedVehicleScheduler
EnhancedVehicleSchedulerWithECBS = EnhancedVehicleScheduler