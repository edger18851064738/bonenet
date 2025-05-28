"""
enhanced_vehicle_scheduler_with_ecbs.py - 集成ECBS的增强车辆调度器
在原有调度器基础上添加多车辆协调功能，通过ECBS算法实现冲突避免的路径规划
"""

import math
import time
import threading
import heapq
from typing import List, Dict, Tuple, Optional, Any, Set
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass, field
from enum import Enum

# 导入原有的枚举和数据类
from vehicle_scheduler import (
    TaskStatus, VehicleStatus, TaskPriority, EnhancedTask, 
    EnhancedVehicleState, SystemEfficiencyOptimizer,EnhancedVehicleScheduler
)

class CoordinationMode(Enum):
    """协调模式"""
    SINGLE_VEHICLE = "single_vehicle"      # 单车辆模式（原有）
    BATCH_COORDINATION = "batch_coordination"  # 批量协调模式
    REAL_TIME_COORDINATION = "real_time_coordination"  # 实时协调模式
    PERIODIC_COORDINATION = "periodic_coordination"    # 定期协调模式

@dataclass
class CoordinationRequest:
    """协调请求"""
    request_id: str
    vehicle_requests: Dict[str, Dict]  # {vehicle_id: {'start', 'goal', 'priority', 'deadline'}}
    coordination_mode: CoordinationMode = CoordinationMode.BATCH_COORDINATION
    max_solve_time: float = 30.0
    quality_threshold: float = 0.7
    
    # 骨干路径相关
    prefer_backbone: bool = True
    backbone_priority_weight: float = 0.3
    
    # 冲突处理相关
    allow_conflicts: bool = False
    max_conflicts: int = 0

@dataclass
class CoordinationResult:
    """协调结果"""
    request_id: str
    success: bool
    coordinated_paths: Dict[str, List[Tuple]] = field(default_factory=dict)
    path_structures: Dict[str, Dict] = field(default_factory=dict)
    
    # 质量指标
    total_cost: float = 0.0
    backbone_utilization: float = 0.0
    conflict_count: int = 0
    solve_time: float = 0.0
    
    # 详细信息
    ecbs_expansions: int = 0
    constraints_generated: int = 0
    initial_conflicts: int = 0
    final_conflicts: int = 0
    
    # 错误信息
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
        
        # 按优先级排序车辆请求
        sorted_requests = sorted(
            request.vehicle_requests.items(),
            key=lambda x: x[1].get('priority', 0.5),
            reverse=True
        )
        
        print(f"为 {len(sorted_requests)} 个车辆分配骨干路径")
        
        for vehicle_id, vehicle_request in sorted_requests:
            try:
                # 使用骨干网络的路径查找接口
                if hasattr(self.backbone_network, 'get_path_from_position_to_target'):
                    # 简化：假设goal是某个特殊点
                    target_type = 'unloading'  # 这里需要根据实际任务类型判断
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
            # 使用骨干网络的路径分析功能
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
                # 使用调度器的路径规划功能
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
        # 简化的缓存键生成
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
        
        # 更新平均求解时间
        total_requests = stats['total_requests']
        if total_requests > 1:
            stats['average_solve_time'] = (
                stats['average_solve_time'] * (total_requests - 1) + result.solve_time
            ) / total_requests
        else:
            stats['average_solve_time'] = result.solve_time
        
        # 更新其他统计
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

class EnhancedVehicleSchedulerWithECBS:
    """集成ECBS的增强车辆调度器"""
    
    def __init__(self, env, path_planner=None, backbone_network=None, traffic_manager=None):
        # 调用原有初始化
        super().__init__()
        
        # 核心组件（原有）
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 原有数据结构
        self.tasks = OrderedDict()
        self.vehicle_states = {}
        self.mission_templates = {}
        self.task_counter = 0
        self.active_assignments = defaultdict(list)
        self.task_priority_queue = []
        
        # 原有组件
        self.efficiency_optimizer = SystemEfficiencyOptimizer(env, self)
        self.state_lock = threading.RLock()
        
        # 新增：ECBS协调器
        self.ecbs_coordinator = ECBSCoordinator(self, traffic_manager, backbone_network)
        
        # 新增：协调配置
        self.coordination_config = {
            'enable_ecbs': True,
            'default_coordination_mode': CoordinationMode.BATCH_COORDINATION,
            'coordination_trigger_threshold': 3,  # 3个以上车辆触发协调
            'coordination_interval': 120.0,      # 定期协调间隔
            'max_coordination_time': 30.0,       # 最大协调时间
            'prefer_backbone_in_coordination': True
        }
        
        # 新增：协调状态
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
            # 新增：ECBS相关统计
            'ecbs_coordinations': 0,
            'ecbs_success_rate': 0.0,
            'avg_coordination_time': 0.0,
            'conflict_reduction_rate': 0.0,
            'backbone_utilization_improvement': 0.0
        }
        
        print("初始化集成ECBS的增强车辆调度器")
    
    def coordinate_multiple_vehicles(self, vehicle_ids: List[str], 
                                   mission_templates: Dict[str, str] = None,
                                   coordination_mode: CoordinationMode = None,
                                   priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """批量协调多个车辆 - 新增核心方法"""
        
        if not self.coordination_config['enable_ecbs'] or len(vehicle_ids) < 2:
            # 回退到单车辆分配
            return self._assign_individual_missions(vehicle_ids, mission_templates, priority)
        
        print(f"开始多车辆协调: {len(vehicle_ids)} 个车辆")
        
        try:
            # 准备协调请求
            coordination_request = self._prepare_coordination_request(
                vehicle_ids, mission_templates, coordination_mode, priority
            )
            
            if not coordination_request:
                print("协调请求准备失败，回退到单车辆分配")
                return self._assign_individual_missions(vehicle_ids, mission_templates, priority)
            
            # 执行ECBS协调
            result = self.ecbs_coordinator.coordinate_vehicles(coordination_request)
            
            if result.success:
                # 应用协调结果
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
            
            # 确定任务模板
            template_id = mission_templates.get(vehicle_id, 'default') if mission_templates else 'default'
            
            # 创建默认模板（如果不存在）
            if template_id not in self.mission_templates:
                if not self.create_enhanced_mission_template(template_id, priority=priority):
                    print(f"创建任务模板失败: {template_id}")
                    continue
            
            template = self.mission_templates[template_id]
            
            # 确定起点和目标
            start = vehicle_state.position
            goal = template['loading_position']  # 第一个任务通常是去装载点
            
            vehicle_requests[vehicle_id] = {
                'start': start,
                'goal': goal,
                'priority': priority.value,
                'deadline': time.time() + 300,  # 5分钟deadline
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
                    # 创建任务并分配路径
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
        
        # 创建任务
        task = EnhancedTask(
            task_id=task_id,
            task_type='coordinated_mission',
            start=coordinated_path[0],
            goal=coordinated_path[-1],
            priority=priority,
            path=coordinated_path,
            path_structure=path_structure
        )
        
        # 计算骨干网络利用率
        if path_structure:
            task.backbone_utilization = path_structure.get('backbone_utilization', 0.0)
        
        # 存储任务
        self.tasks[task_id] = task
        self.active_assignments[vehicle_id].append(task_id)
        self.stats['total_tasks'] += 1
        
        # 更新车辆状态
        vehicle_state = self.vehicle_states[vehicle_id]
        vehicle_state.task_queue.append(task_id)
        
        # 开始任务执行
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
        """触发定期协调 - 新增方法"""
        
        current_time = time.time()
        
        # 检查是否需要定期协调
        if (current_time - self.coordination_state['last_coordination_time'] < 
            self.coordination_config['coordination_interval']):
            return False
        
        # 找到需要协调的车辆
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
            # 空闲车辆或即将完成任务的车辆
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
        
        # 简单判断：如果任务已运行超过预期时间的80%
        if task.estimated_duration > 0:
            elapsed = time.time() - task.start_time
            return elapsed > task.estimated_duration * 0.8
        
        return False
    
    def get_coordination_statistics(self) -> Dict:
        """获取协调统计信息 - 新增方法"""
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
        """启用/禁用ECBS协调 - 新增方法"""
        self.coordination_config['enable_ecbs'] = enable
        print(f"ECBS协调: {'启用' if enable else '禁用'}")
    
    def set_coordination_parameters(self, **kwargs):
        """设置协调参数 - 新增方法"""
        for key, value in kwargs.items():
            if key in self.coordination_config:
                self.coordination_config[key] = value
                print(f"更新协调参数 {key}: {value}")
    
    # 保持原有的所有方法...
    def initialize_vehicles(self):
        """保持原有的车辆初始化方法"""
        pass  # 这里保持原有实现
    
    def assign_mission_intelligently(self, vehicle_id: str = None, 
                                   template_id: str = "default",
                                   priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """保持原有的智能任务分配方法"""
        pass  # 这里保持原有实现
    
    def update(self, time_delta: float):
        """增强的更新方法"""
        # 调用原有更新逻辑
        super().update(time_delta)
        
        # 新增：检查是否需要触发定期协调
        try:
            if self.coordination_config['enable_ecbs']:
                self.trigger_periodic_coordination()
        except Exception as e:
            print(f"定期协调检查失败: {e}")
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息 - 增强版"""
        stats = super().get_comprehensive_stats()
        
        # 添加ECBS协调统计
        coordination_stats = self.get_coordination_statistics()
        stats['coordination'] = coordination_stats
        
        return stats

    def create_enhanced_mission_template(self, template_id: str, 
                                       loading_point_id: int = None, 
                                       unloading_point_id: int = None,
                                       priority: TaskPriority = TaskPriority.NORMAL) -> bool:
        """创建增强任务模板"""
        try:
            if not self.env.loading_points or not self.env.unloading_points:
                print("缺少装载点或卸载点")
                return False
            
            # 选择装载点和卸载点
            if loading_point_id is None:
                loading_point_id = 0
            if unloading_point_id is None:
                unloading_point_id = 0
            
            # 验证有效性
            if (loading_point_id >= len(self.env.loading_points) or 
                unloading_point_id >= len(self.env.unloading_points)):
                print(f"点ID超出范围: L{loading_point_id}, U{unloading_point_id}")
                return False
            
            loading_point = self.env.loading_points[loading_point_id]
            unloading_point = self.env.unloading_points[unloading_point_id]
            
            # 创建增强模板
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
    def shutdown(self):
        """关闭调度器"""
        with self.state_lock:
            self.tasks.clear()
            self.vehicle_states.clear()
            self.active_assignments.clear()
            self.task_priority_queue.clear()
        
        print("车辆调度器已关闭")