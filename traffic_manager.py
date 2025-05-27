"""
traffic_manager.py - 深度集成智能交通管理器
基于混合A*的分层冲突解决与协同优化系统
"""

import math
import heapq
import time
import numpy as np
from typing import List, Dict, Tuple, Set, Optional, Any, Callable
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass, field
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
import weakref
from enum import Enum
import asyncio

class ConflictType(Enum):
    """冲突类型枚举"""
    VERTEX = "vertex"           # 顶点冲突
    EDGE = "edge"              # 边冲突
    INTERFACE = "interface"     # 接口冲突
    BACKBONE = "backbone"       # 骨干路径冲突
    TIME_WINDOW = "time_window" # 时间窗口冲突
    PREDICTIVE = "predictive"   # 预测性冲突
    DYNAMIC = "dynamic"         # 动态冲突

class ConflictSeverity(Enum):
    """冲突严重性等级"""
    LOW = 1          # 轻微冲突
    MEDIUM = 2       # 中等冲突
    HIGH = 3         # 严重冲突
    CRITICAL = 4     # 关键冲突
    EMERGENCY = 5    # 紧急冲突

class ResolutionStrategy(Enum):
    """冲突解决策略"""
    TIME_SHIFT = "time_shift"           # 时间错开
    REPLANNING = "replanning"           # 重新规划
    ALTERNATIVE_PATH = "alternative"     # 替代路径
    PRIORITY_OVERRIDE = "priority"       # 优先级覆盖
    COOPERATIVE = "cooperative"          # 协作解决
    INTERFACE_SWITCH = "interface_switch" # 接口切换

@dataclass
class EnhancedConflict:
    """增强冲突表示 - 支持混合A*集成"""
    conflict_id: str
    agent1: str
    agent2: str
    location: Tuple[float, float]
    time_step: float
    conflict_type: ConflictType = ConflictType.VERTEX
    severity: ConflictSeverity = ConflictSeverity.MEDIUM
    confidence: float = 1.0
    
    # 混合A*集成信息
    path_quality_impact: float = 0.0     # 对路径质量的影响
    astar_complexity: float = 0.0        # A*复杂度影响
    curvature_conflict: bool = False     # 是否为曲率冲突
    vehicle_dynamics_violation: bool = False  # 车辆动力学违规
    
    # 解决策略信息
    recommended_strategy: ResolutionStrategy = ResolutionStrategy.REPLANNING
    alternative_strategies: List[ResolutionStrategy] = field(default_factory=list)
    resolution_cost: float = 1.0        # 解决代价
    
    # 资源相关
    interface_id: Optional[str] = None
    resource_type: Optional[str] = None
    backbone_path_id: Optional[str] = None
    
    # 预测信息
    prediction_horizon: float = 0.0     # 预测时间窗口
    occurrence_probability: float = 1.0  # 发生概率
    
    # 历史信息
    creation_time: float = field(default_factory=time.time)
    last_update: float = field(default_factory=time.time)
    resolution_attempts: int = 0
    
    def __lt__(self, other):
        """优先级比较"""
        # 优先级：严重性 > 紧急程度 > 置信度 > 时间
        if self.severity != other.severity:
            return self.severity.value > other.severity.value
        
        urgency_self = self._calculate_urgency()
        urgency_other = other._calculate_urgency()
        if abs(urgency_self - urgency_other) > 0.1:
            return urgency_self > urgency_other
        
        if abs(self.confidence - other.confidence) > 0.1:
            return self.confidence > other.confidence
        
        return self.time_step < other.time_step
    
    def _calculate_urgency(self) -> float:
        """计算紧急程度"""
        current_time = time.time()
        time_to_conflict = self.time_step - (current_time - self.creation_time)
        
        # 时间越近越紧急
        urgency = max(0, 1.0 - time_to_conflict / 60.0)  # 1分钟内的冲突最紧急
        
        # 严重性影响
        urgency *= self.severity.value / 5.0
        
        # 概率影响
        urgency *= self.occurrence_probability
        
        return urgency
    
    def update_severity(self, new_severity: ConflictSeverity):
        """更新严重性"""
        self.severity = new_severity
        self.last_update = time.time()
    
    def add_resolution_attempt(self):
        """记录解决尝试"""
        self.resolution_attempts += 1
        self.last_update = time.time()

@dataclass
class SmartConstraint:
    """智能约束条件 - 支持混合A*优化"""
    agent_id: str
    constraint_type: ConflictType
    location: Optional[Tuple[float, float]] = None
    time_step: Optional[float] = None
    time_window: Optional[Tuple[float, float]] = None
    interface_id: Optional[str] = None
    priority: int = 1
    
    # 混合A*集成
    path_quality_requirement: float = 0.7   # 路径质量要求
    curvature_limit: float = 0.3           # 曲率限制
    smoothness_requirement: float = 0.8     # 平滑度要求
    
    # 灵活性参数
    flexibility: float = 0.0               # 约束灵活性 [0,1]
    alternative_options: List = field(default_factory=list)
    relaxation_factor: float = 1.0         # 松弛因子
    
    # 有效期
    valid_from: float = field(default_factory=time.time)
    valid_until: float = float('inf')
    
    def is_valid(self, current_time: float = None) -> bool:
        """检查约束是否有效"""
        if current_time is None:
            current_time = time.time()
        return self.valid_from <= current_time <= self.valid_until
    
    def get_relaxed_constraint(self) -> 'SmartConstraint':
        """获取松弛版本的约束"""
        relaxed = SmartConstraint(
            agent_id=self.agent_id,
            constraint_type=self.constraint_type,
            location=self.location,
            time_step=self.time_step,
            time_window=self.time_window,
            interface_id=self.interface_id,
            priority=max(1, self.priority - 1)
        )
        
        # 放松质量要求
        relaxed.path_quality_requirement = max(0.3, self.path_quality_requirement - 0.2)
        relaxed.curvature_limit = min(1.0, self.curvature_limit + 0.2)
        relaxed.smoothness_requirement = max(0.3, self.smoothness_requirement - 0.2)
        
        return relaxed

class PredictiveConflictDetector:
    """预测性冲突检测器"""
    
    def __init__(self, prediction_horizon: float = 120.0):
        self.prediction_horizon = prediction_horizon
        self.motion_models = {}  # 车辆运动模型
        self.uncertainty_models = {}  # 不确定性模型
        
    def predict_conflicts(self, vehicle_paths: Dict[str, Dict], 
                         current_time: float) -> List[EnhancedConflict]:
        """预测未来冲突"""
        predicted_conflicts = []
        
        # 更新运动模型
        self._update_motion_models(vehicle_paths, current_time)
        
        # 生成预测轨迹
        predicted_trajectories = self._generate_predicted_trajectories(
            vehicle_paths, current_time
        )
        
        # 检测轨迹交叉
        conflicts = self._detect_trajectory_conflicts(
            predicted_trajectories, current_time
        )
        
        predicted_conflicts.extend(conflicts)
        
        return predicted_conflicts
    
    def _update_motion_models(self, vehicle_paths: Dict, current_time: float):
        """更新车辆运动模型"""
        for vehicle_id, path_info in vehicle_paths.items():
            if vehicle_id not in self.motion_models:
                self.motion_models[vehicle_id] = {
                    'speed_history': deque(maxlen=10),
                    'acceleration_history': deque(maxlen=10),
                    'curvature_history': deque(maxlen=10),
                    'last_update': current_time
                }
            
            model = self.motion_models[vehicle_id]
            
            # 更新速度历史
            current_speed = path_info.get('speed', 1.0)
            model['speed_history'].append(current_speed)
            
            # 计算加速度
            if len(model['speed_history']) >= 2:
                dt = current_time - model['last_update']
                if dt > 0:
                    acceleration = (current_speed - model['speed_history'][-2]) / dt
                    model['acceleration_history'].append(acceleration)
            
            model['last_update'] = current_time
    
    def _generate_predicted_trajectories(self, vehicle_paths: Dict, 
                                       current_time: float) -> Dict[str, List[Tuple]]:
        """生成预测轨迹"""
        trajectories = {}
        
        for vehicle_id, path_info in vehicle_paths.items():
            path = path_info.get('path', [])
            if not path:
                continue
            
            current_position = path_info.get('current_position', path[0])
            predicted_path = self._predict_vehicle_path(
                vehicle_id, path, current_position, current_time
            )
            
            trajectories[vehicle_id] = predicted_path
        
        return trajectories
    
    def _predict_vehicle_path(self, vehicle_id: str, original_path: List,
                             current_position: Tuple, current_time: float) -> List[Tuple]:
        """预测单个车辆的未来轨迹"""
        if vehicle_id not in self.motion_models:
            return original_path
        
        model = self.motion_models[vehicle_id]
        predicted_path = []
        
        # 获取平均速度
        avg_speed = np.mean(model['speed_history']) if model['speed_history'] else 1.0
        
        # 获取加速度趋势
        avg_acceleration = (np.mean(model['acceleration_history']) 
                          if model['acceleration_history'] else 0.0)
        
        # 生成预测点
        time_steps = np.arange(0, self.prediction_horizon, 2.0)  # 每2秒一个预测点
        
        for t in time_steps:
            # 简化的运动预测
            predicted_speed = max(0.1, avg_speed + avg_acceleration * t)
            distance_traveled = predicted_speed * t
            
            # 沿原路径插值
            predicted_pos = self._interpolate_along_path(
                original_path, current_position, distance_traveled
            )
            
            if predicted_pos:
                predicted_path.append((predicted_pos[0], predicted_pos[1], 
                                     current_time + t, predicted_speed))
        
        return predicted_path
    
    def _interpolate_along_path(self, path: List, start_pos: Tuple, 
                               distance: float) -> Optional[Tuple]:
        """沿路径插值"""
        if not path:
            return None
        
        current_distance = 0.0
        
        for i in range(len(path) - 1):
            segment_distance = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            
            if current_distance + segment_distance >= distance:
                # 在此段内插值
                remaining = distance - current_distance
                ratio = remaining / segment_distance if segment_distance > 0 else 0
                
                x = path[i][0] + ratio * (path[i+1][0] - path[i][0])
                y = path[i][1] + ratio * (path[i+1][1] - path[i][1])
                theta = path[i][2] if len(path[i]) > 2 else 0
                
                return (x, y, theta)
            
            current_distance += segment_distance
        
        return path[-1] if path else None
    
    def _detect_trajectory_conflicts(self, trajectories: Dict[str, List[Tuple]], 
                                   current_time: float) -> List[EnhancedConflict]:
        """检测轨迹冲突"""
        conflicts = []
        
        vehicle_ids = list(trajectories.keys())
        
        for i in range(len(vehicle_ids)):
            for j in range(i + 1, len(vehicle_ids)):
                vehicle1, vehicle2 = vehicle_ids[i], vehicle_ids[j]
                traj1, traj2 = trajectories[vehicle1], trajectories[vehicle2]
                
                # 检测轨迹交叉
                trajectory_conflicts = self._check_trajectory_intersection(
                    vehicle1, traj1, vehicle2, traj2, current_time
                )
                conflicts.extend(trajectory_conflicts)
        
        return conflicts
    
    def _check_trajectory_intersection(self, vehicle1: str, traj1: List[Tuple],
                                     vehicle2: str, traj2: List[Tuple],
                                     current_time: float) -> List[EnhancedConflict]:
        """检查两条轨迹的交叉点"""
        conflicts = []
        
        # 按时间对齐轨迹点
        for point1 in traj1:
            for point2 in traj2:
                if len(point1) >= 4 and len(point2) >= 4:
                    x1, y1, t1, _ = point1[:4]
                    x2, y2, t2, _ = point2[:4]
                    
                    # 时间窗口匹配
                    if abs(t1 - t2) <= 5.0:  # 5秒时间窗口
                        distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
                        
                        if distance < 8.0:  # 安全距离
                            # 计算冲突概率
                            time_diff = abs(t1 - t2)
                            distance_factor = max(0, 1.0 - distance / 8.0)
                            time_factor = max(0, 1.0 - time_diff / 5.0)
                            probability = distance_factor * time_factor
                            
                            if probability > 0.3:  # 概率阈值
                                conflict = EnhancedConflict(
                                    conflict_id=f"pred_{vehicle1}_{vehicle2}_{int(t1)}",
                                    agent1=vehicle1,
                                    agent2=vehicle2,
                                    location=((x1 + x2) / 2, (y1 + y2) / 2),
                                    time_step=t1,
                                    conflict_type=ConflictType.PREDICTIVE,
                                    severity=self._calculate_predictive_severity(distance, probability),
                                    confidence=probability,
                                    occurrence_probability=probability,
                                    prediction_horizon=t1 - current_time
                                )
                                conflicts.append(conflict)
        
        return conflicts
    
    def _calculate_predictive_severity(self, distance: float, 
                                     probability: float) -> ConflictSeverity:
        """计算预测冲突的严重性"""
        severity_score = (1.0 - distance / 8.0) * probability
        
        if severity_score > 0.8:
            return ConflictSeverity.CRITICAL
        elif severity_score > 0.6:
            return ConflictSeverity.HIGH
        elif severity_score > 0.4:
            return ConflictSeverity.MEDIUM
        else:
            return ConflictSeverity.LOW

class EnhancedECBSSolver:
    """增强ECBS求解器 - 深度集成混合A*"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # ECBS优化参数
        self.suboptimality_bound = 1.2
        self.max_search_time = 30.0
        self.max_nodes_expanded = 1000
        self.focal_weight = 1.1
        
        # 混合A*集成参数
        self.astar_quality_threshold = 0.7
        self.path_smoothness_weight = 0.3
        self.curvature_penalty = 0.2
        
        # 分层解决策略
        self.resolution_layers = [
            'interface_layer',    # 接口层冲突
            'backbone_layer',     # 骨干路径层冲突
            'spatial_layer',      # 空间层冲突
            'temporal_layer'      # 时间层冲突
        ]
        
        # 性能统计
        self.stats = {
            'nodes_expanded': 0,
            'conflicts_resolved': 0,
            'total_time': 0,
            'cache_hits': 0,
            'layer_resolutions': defaultdict(int),
            'quality_improvements': 0,
            'astar_integrations': 0
        }
        
        # 解决方案缓存
        self.solution_cache = OrderedDict()
        self.cache_size_limit = 300
        
        # 质量评估器
        self.quality_evaluator = PathQualityEvaluator()
    
    def solve_hierarchical(self, initial_paths: Dict[str, List], 
                          conflicts: List[EnhancedConflict],
                          current_time: float = 0) -> Dict[str, List]:
        """分层ECBS求解"""
        start_time = time.time()
        
        if not conflicts:
            return initial_paths
        
        # 按层次分类冲突
        layered_conflicts = self._classify_conflicts_by_layer(conflicts)
        
        resolved_paths = initial_paths.copy()
        total_resolved = 0
        
        # 逐层解决冲突
        for layer in self.resolution_layers:
            if layer not in layered_conflicts:
                continue
            
            layer_conflicts = layered_conflicts[layer]
            if not layer_conflicts:
                continue
            
            print(f"解决 {layer} 层冲突: {len(layer_conflicts)} 个")
            
            layer_result = self._solve_layer_conflicts(
                resolved_paths, layer_conflicts, layer, current_time
            )
            
            if layer_result:
                resolved_paths = layer_result
                total_resolved += len(layer_conflicts)
                self.stats['layer_resolutions'][layer] += len(layer_conflicts)
        
        # 整体质量优化
        if self.path_planner and total_resolved > 0:
            optimized_paths = self._optimize_path_quality(resolved_paths)
            if optimized_paths:
                resolved_paths = optimized_paths
                self.stats['quality_improvements'] += 1
        
        # 更新统计
        solving_time = time.time() - start_time
        self.stats['total_time'] += solving_time
        self.stats['conflicts_resolved'] += total_resolved
        
        print(f"分层ECBS求解完成: 解决了 {total_resolved} 个冲突，耗时 {solving_time:.2f}s")
        
        return resolved_paths
    
    def _classify_conflicts_by_layer(self, conflicts: List[EnhancedConflict]) -> Dict[str, List]:
        """按层次分类冲突"""
        layered = {layer: [] for layer in self.resolution_layers}
        
        for conflict in conflicts:
            if conflict.conflict_type == ConflictType.INTERFACE:
                layered['interface_layer'].append(conflict)
            elif conflict.conflict_type == ConflictType.BACKBONE:
                layered['backbone_layer'].append(conflict)
            elif conflict.conflict_type in [ConflictType.VERTEX, ConflictType.EDGE]:
                layered['spatial_layer'].append(conflict)
            else:
                layered['temporal_layer'].append(conflict)
        
        # 按优先级排序每层的冲突
        for layer_conflicts in layered.values():
            layer_conflicts.sort(reverse=True)
        
        return layered
    
    def _solve_layer_conflicts(self, paths: Dict[str, List], 
                              conflicts: List[EnhancedConflict],
                              layer: str, current_time: float) -> Optional[Dict[str, List]]:
        """解决特定层的冲突"""
        if layer == 'interface_layer':
            return self._solve_interface_conflicts(paths, conflicts, current_time)
        elif layer == 'backbone_layer':
            return self._solve_backbone_conflicts(paths, conflicts)
        elif layer == 'spatial_layer':
            return self._solve_spatial_conflicts(paths, conflicts)
        elif layer == 'temporal_layer':
            return self._solve_temporal_conflicts(paths, conflicts, current_time)
        
        return paths
    
    def _solve_interface_conflicts(self, paths: Dict[str, List], 
                                  conflicts: List[EnhancedConflict],
                                  current_time: float) -> Dict[str, List]:
        """解决接口层冲突"""
        resolved_paths = paths.copy()
        
        for conflict in conflicts:
            strategy = self._select_interface_resolution_strategy(conflict)
            
            if strategy == ResolutionStrategy.TIME_SHIFT:
                success = self._apply_time_shift(conflict, current_time)
                if success:
                    continue
            
            elif strategy == ResolutionStrategy.INTERFACE_SWITCH:
                alternative_path = self._find_alternative_interface_path(
                    conflict.agent1, resolved_paths.get(conflict.agent1, [])
                )
                if alternative_path:
                    resolved_paths[conflict.agent1] = alternative_path
                    continue
            
            # 回退到重规划
            new_path = self._replan_with_astar_optimization(
                conflict.agent1, resolved_paths.get(conflict.agent1, [])
            )
            if new_path:
                resolved_paths[conflict.agent1] = new_path
        
        return resolved_paths
    
    def _solve_backbone_conflicts(self, paths: Dict[str, List], 
                                 conflicts: List[EnhancedConflict]) -> Dict[str, List]:
        """解决骨干路径层冲突"""
        resolved_paths = paths.copy()
        
        for conflict in conflicts:
            # 优先尝试时间错开
            if self._try_temporal_separation(conflict, resolved_paths):
                continue
            
            # 寻找替代骨干路径
            for agent in [conflict.agent1, conflict.agent2]:
                current_path = resolved_paths.get(agent, [])
                if not current_path or len(current_path) < 2:
                    continue
                
                alternative_path = self._find_alternative_backbone_path(
                    current_path[0], current_path[-1], agent
                )
                
                if alternative_path:
                    resolved_paths[agent] = alternative_path
                    break
        
        return resolved_paths
    
    def _solve_spatial_conflicts(self, paths: Dict[str, List], 
                                conflicts: List[EnhancedConflict]) -> Dict[str, List]:
        """解决空间层冲突"""
        resolved_paths = paths.copy()
        
        # 按冲突影响度排序
        sorted_conflicts = sorted(conflicts, 
                                key=lambda c: c.path_quality_impact, reverse=True)
        
        for conflict in sorted_conflicts:
            # 选择重规划的智能体
            agent_to_replan = self._select_replanning_agent(conflict, resolved_paths)
            
            current_path = resolved_paths.get(agent_to_replan, [])
            if not current_path or len(current_path) < 2:
                continue
            
            # 生成约束
            constraints = self._generate_spatial_constraints(conflict, resolved_paths)
            
            # 使用混合A*重规划
            new_path = self._replan_with_constraints_and_astar(
                agent_to_replan, current_path[0], current_path[-1], constraints
            )
            
            if new_path:
                # 验证路径质量
                quality = self.quality_evaluator.evaluate_path(new_path)
                if quality >= self.astar_quality_threshold:
                    resolved_paths[agent_to_replan] = new_path
                    self.stats['astar_integrations'] += 1
        
        return resolved_paths
    
    def _solve_temporal_conflicts(self, paths: Dict[str, List], 
                                 conflicts: List[EnhancedConflict],
                                 current_time: float) -> Dict[str, List]:
        """解决时间层冲突"""
        resolved_paths = paths.copy()
        
        for conflict in conflicts:
            # 尝试动态速度调整
            if self._try_speed_adjustment(conflict, resolved_paths):
                continue
            
            # 尝试路径重时序
            if self._try_path_retiming(conflict, resolved_paths, current_time):
                continue
        
        return resolved_paths
    
    def _replan_with_astar_optimization(self, agent: str, 
                                      current_path: List) -> Optional[List]:
        """使用混合A*优化重规划"""
        if not current_path or len(current_path) < 2:
            return None
        
        if not self.path_planner:
            return None
        
        start, goal = current_path[0], current_path[-1]
        
        try:
            # 使用混合A*进行高质量重规划
            result = self.path_planner.plan_path(
                agent, start, goal,
                use_backbone=True,
                check_conflicts=False,  # 由ECBS处理冲突
                planner_type="hybrid_astar"
            )
            
            if result:
                if isinstance(result, tuple):
                    path, structure = result
                    quality = structure.get('final_quality', 0)
                    
                    # 只接受高质量路径
                    if quality >= self.astar_quality_threshold:
                        return path
                else:
                    return result
        
        except Exception as e:
            print(f"混合A*重规划失败: {e}")
        
        return None
    
    def _replan_with_constraints_and_astar(self, agent: str, start: Tuple, goal: Tuple,
                                         constraints: List[SmartConstraint]) -> Optional[List]:
        """带约束的混合A*重规划"""
        if not self.path_planner:
            return None
        
        try:
            # 将约束转换为规划提示
            planning_hints = self._constraints_to_astar_hints(constraints)
            
            # 多次尝试，逐步放松约束
            for attempt in range(3):
                result = self.path_planner.plan_path(
                    agent, start, goal,
                    use_backbone=True,
                    check_conflicts=False,
                    planner_type="hybrid_astar"
                )
                
                if result:
                    if isinstance(result, tuple):
                        path, structure = result
                        
                        # 验证约束满足
                        if self._validate_constraints(path, constraints):
                            quality = structure.get('final_quality', 0)
                            if quality >= max(0.4, self.astar_quality_threshold - attempt * 0.1):
                                return path
                    else:
                        if self._validate_constraints(result, constraints):
                            return result
                
                # 放松约束
                constraints = [c.get_relaxed_constraint() for c in constraints]
        
        except Exception as e:
            print(f"约束混合A*重规划失败: {e}")
        
        return None
    
    def _constraints_to_astar_hints(self, constraints: List[SmartConstraint]) -> Dict:
        """将约束转换为混合A*提示"""
        hints = {
            'forbidden_regions': [],
            'quality_requirements': {},
            'smoothness_requirements': {}
        }
        
        for constraint in constraints:
            if constraint.location:
                hints['forbidden_regions'].append({
                    'location': constraint.location,
                    'time': constraint.time_step,
                    'radius': 5.0
                })
            
            hints['quality_requirements'][constraint.agent_id] = {
                'min_quality': constraint.path_quality_requirement,
                'max_curvature': constraint.curvature_limit,
                'min_smoothness': constraint.smoothness_requirement
            }
        
        return hints
    
    def _validate_constraints(self, path: List, 
                            constraints: List[SmartConstraint]) -> bool:
        """验证路径是否满足约束"""
        if not path:
            return False
        
        for constraint in constraints:
            if not constraint.is_valid():
                continue
            
            if constraint.constraint_type == ConflictType.VERTEX:
                if not self._validate_vertex_constraint(path, constraint):
                    return False
            elif constraint.constraint_type == ConflictType.EDGE:
                if not self._validate_edge_constraint(path, constraint):
                    return False
        
        return True
    
    def _validate_vertex_constraint(self, path: List, 
                                   constraint: SmartConstraint) -> bool:
        """验证顶点约束"""
        if not constraint.location or constraint.time_step is None:
            return True
        
        time_index = int(constraint.time_step)
        if 0 <= time_index < len(path):
            point = path[time_index]
            distance = math.sqrt(
                (point[0] - constraint.location[0])**2 + 
                (point[1] - constraint.location[1])**2
            )
            
            # 考虑灵活性
            allowed_distance = 5.0 * (1 + constraint.flexibility)
            return distance >= allowed_distance
        
        return True
    
    def _validate_edge_constraint(self, path: List, 
                                 constraint: SmartConstraint) -> bool:
        """验证边约束"""
        # 简化实现
        return True
    
    def _optimize_path_quality(self, paths: Dict[str, List]) -> Optional[Dict[str, List]]:
        """优化路径质量"""
        if not self.path_planner:
            return None
        
        optimized_paths = {}
        improvement_count = 0
        
        for agent, path in paths.items():
            if not path or len(path) < 2:
                optimized_paths[agent] = path
                continue
            
            # 评估当前路径质量
            current_quality = self.quality_evaluator.evaluate_path(path)
            
            if current_quality < 0.8:  # 只优化低质量路径
                try:
                    result = self.path_planner.plan_path(
                        agent, path[0], path[-1],
                        use_backbone=True,
                        check_conflicts=False,
                        planner_type="hybrid_astar"
                    )
                    
                    if result:
                        if isinstance(result, tuple):
                            new_path, structure = result
                            new_quality = structure.get('final_quality', 0)
                        else:
                            new_path = result
                            new_quality = self.quality_evaluator.evaluate_path(new_path)
                        
                        if new_quality > current_quality + 0.1:
                            optimized_paths[agent] = new_path
                            improvement_count += 1
                        else:
                            optimized_paths[agent] = path
                    else:
                        optimized_paths[agent] = path
                
                except Exception:
                    optimized_paths[agent] = path
            else:
                optimized_paths[agent] = path
        
        if improvement_count > 0:
            print(f"质量优化完成: {improvement_count} 条路径得到改善")
            return optimized_paths
        
        return None
    
    def _select_interface_resolution_strategy(self, conflict: EnhancedConflict) -> ResolutionStrategy:
        """选择接口冲突解决策略"""
        if conflict.severity.value <= 2:
            return ResolutionStrategy.TIME_SHIFT
        elif conflict.interface_id:
            return ResolutionStrategy.INTERFACE_SWITCH
        else:
            return ResolutionStrategy.REPLANNING
    
    def _select_replanning_agent(self, conflict: EnhancedConflict, 
                               paths: Dict[str, List]) -> str:
        """选择重规划的智能体"""
        # 选择路径质量较低的智能体
        agent1_path = paths.get(conflict.agent1, [])
        agent2_path = paths.get(conflict.agent2, [])
        
        if not agent1_path:
            return conflict.agent2
        if not agent2_path:
            return conflict.agent1
        
        quality1 = self.quality_evaluator.evaluate_path(agent1_path)
        quality2 = self.quality_evaluator.evaluate_path(agent2_path)
        
        return conflict.agent1 if quality1 <= quality2 else conflict.agent2
    
    # 其他辅助方法的简化实现
    def _apply_time_shift(self, conflict: EnhancedConflict, current_time: float) -> bool:
        """应用时间错开策略"""
        # 简化实现
        return True
    
    def _find_alternative_interface_path(self, agent: str, current_path: List) -> Optional[List]:
        """寻找替代接口路径"""
        # 简化实现
        return None
    
    def _try_temporal_separation(self, conflict: EnhancedConflict, paths: Dict) -> bool:
        """尝试时间分离"""
        # 简化实现
        return False
    
    def _find_alternative_backbone_path(self, start: Tuple, goal: Tuple, agent: str) -> Optional[List]:
        """寻找替代骨干路径"""
        # 简化实现
        return None
    
    def _generate_spatial_constraints(self, conflict: EnhancedConflict, paths: Dict) -> List[SmartConstraint]:
        """生成空间约束"""
        constraints = []
        
        constraint = SmartConstraint(
            agent_id=conflict.agent1,
            constraint_type=conflict.conflict_type,
            location=conflict.location,
            time_step=conflict.time_step,
            path_quality_requirement=self.astar_quality_threshold
        )
        constraints.append(constraint)
        
        return constraints
    
    def _try_speed_adjustment(self, conflict: EnhancedConflict, paths: Dict) -> bool:
        """尝试速度调整"""
        # 简化实现
        return False
    
    def _try_path_retiming(self, conflict: EnhancedConflict, paths: Dict, current_time: float) -> bool:
        """尝试路径重时序"""
        # 简化实现
        return False

class PathQualityEvaluator:
    """路径质量评估器"""
    
    def __init__(self):
        self.weights = {
            'length_efficiency': 0.25,
            'smoothness': 0.25,
            'curvature': 0.25,
            'safety': 0.25
        }
    
    def evaluate_path(self, path: List) -> float:
        """评估路径质量"""
        if not path or len(path) < 2:
            return 0.0
        
        scores = {}
        scores['length_efficiency'] = self._evaluate_length_efficiency(path)
        scores['smoothness'] = self._evaluate_smoothness(path)
        scores['curvature'] = self._evaluate_curvature(path)
        scores['safety'] = self._evaluate_safety(path)
        
        total_score = sum(
            scores[metric] * self.weights[metric] 
            for metric in scores
        )
        
        return min(1.0, max(0.0, total_score))
    
    def _evaluate_length_efficiency(self, path: List) -> float:
        """评估长度效率"""
        path_length = sum(
            math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
            for i in range(len(path) - 1)
        )
        
        direct_distance = math.sqrt(
            (path[-1][0] - path[0][0])**2 + (path[-1][1] - path[0][1])**2
        )
        
        if path_length < 0.1:
            return 1.0
        
        efficiency = direct_distance / path_length
        return min(1.0, efficiency)
    
    def _evaluate_smoothness(self, path: List) -> float:
        """评估平滑度"""
        if len(path) < 3:
            return 1.0
        
        total_angle_change = 0.0
        for i in range(1, len(path) - 1):
            angle_change = self._calculate_angle_change(path[i-1], path[i], path[i+1])
            total_angle_change += angle_change
        
        avg_angle_change = total_angle_change / max(1, len(path) - 2)
        smoothness = math.exp(-avg_angle_change * 2)
        
        return min(1.0, smoothness)
    
    def _evaluate_curvature(self, path: List) -> float:
        """评估曲率"""
        if len(path) < 3:
            return 1.0
        
        max_curvature = 0.0
        for i in range(1, len(path) - 1):
            curvature = self._calculate_curvature(path[i-1], path[i], path[i+1])
            max_curvature = max(max_curvature, curvature)
        
        # 曲率越小越好
        curvature_score = max(0, 1.0 - max_curvature / 2.0)
        return curvature_score
    
    def _evaluate_safety(self, path: List) -> float:
        """评估安全性"""
        # 简化实现，可以基于与障碍物的距离
        return 0.8  # 默认安全评分
    
    def _calculate_angle_change(self, p1: Tuple, p2: Tuple, p3: Tuple) -> float:
        """计算角度变化"""
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 0.001 or len2 < 0.001:
            return 0.0
        
        cos_angle = np.dot(v1, v2) / (len1 * len2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        return math.acos(cos_angle)
    
    def _calculate_curvature(self, p1: Tuple, p2: Tuple, p3: Tuple) -> float:
        """计算曲率"""
        angle_change = self._calculate_angle_change(p1, p2, p3)
        
        dist1 = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        dist2 = math.sqrt((p3[0] - p2[0])**2 + (p3[1] - p2[1])**2)
        avg_dist = (dist1 + dist2) / 2
        
        if avg_dist < 0.001:
            return 0.0
        
        return angle_change / avg_dist

class IntegratedTrafficManager:
    """深度集成智能交通管理器"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # 高级组件
        self.ecbs_solver = EnhancedECBSSolver(env, backbone_network, path_planner)
        self.conflict_predictor = PredictiveConflictDetector(prediction_horizon=180.0)
        self.quality_evaluator = PathQualityEvaluator()
        
        # 路径管理
        self.active_paths = OrderedDict()  # {vehicle_id: path_info}
        self.path_reservations = {}  # 时空预留表
        self.path_history = defaultdict(deque)  # 路径历史
        
        # 冲突管理
        self.active_conflicts = OrderedDict()  # {conflict_id: EnhancedConflict}
        self.resolved_conflicts = deque(maxlen=200)  # 已解决冲突历史
        self.conflict_resolution_cache = OrderedDict()
        
        # 接口管理（从原代码移植）
        self.interface_reservations = {}  # {interface_id: [(vehicle_id, start_time, end_time)]}
        self.interface_lock = threading.RLock()
        
        # 性能配置
        self.performance_config = {
            'max_detection_threads': 6,
            'conflict_detection_interval': 10.0,
            'prediction_update_interval': 15.0,
            'quality_optimization_interval': 30.0,
            'cache_cleanup_interval': 60.0,
            'max_concurrent_resolutions': 4
        }
        
        # 检测参数优化
        self.detection_params = {
            'safety_distance': 7.0,
            'time_discretization': 1.0,
            'prediction_horizon': 180.0,
            'confidence_threshold': 0.7,
            'severity_escalation_factor': 1.2
        }
        
        # 线程管理
        self.executor = ThreadPoolExecutor(
            max_workers=self.performance_config['max_detection_threads']
        )
        self.state_lock = threading.RLock()
        
        # 定时器
        self.last_detection_time = 0
        self.last_prediction_time = 0
        self.last_optimization_time = 0
        self.last_cache_cleanup = 0
        
        # 综合统计
        self.stats = {
            'total_conflicts_detected': 0,
            'conflicts_resolved': 0,
            'predictive_conflicts': 0,
            'quality_improvements': 0,
            'cache_hit_rate': 0.0,
            'average_resolution_time': 0.0,
            'layer_performance': defaultdict(dict),
            'planner_integration_stats': defaultdict(int),
            'vehicle_conflict_rates': defaultdict(float),
            'performance_trends': defaultdict(deque)
        }
        
        print("初始化深度集成智能交通管理器")
        self._initialize_performance_monitoring()
    
    def _initialize_performance_monitoring(self):
        """初始化性能监控"""
        # 性能指标
        self.performance_metrics = [
            'detection_time', 'resolution_time', 'path_quality',
            'conflict_rate', 'prediction_accuracy', 'cache_efficiency'
        ]
        
        # 初始化趋势追踪
        for metric in self.performance_metrics:
            self.stats['performance_trends'][metric] = deque(maxlen=100)
    
    def register_vehicle_path_enhanced(self, vehicle_id: str, path: List, 
                                     path_structure: Dict = None, 
                                     start_time: float = 0, speed: float = 1.0,
                                     quality_score: float = 0.5) -> bool:
        """增强版车辆路径注册"""
        if not path or len(path) < 2:
            return False
        
        with self.state_lock:
            # 移除旧路径
            self.release_vehicle_path(vehicle_id)
            
            # 评估路径质量
            if quality_score <= 0:
                quality_score = self.quality_evaluator.evaluate_path(path)
            
            # 准备增强路径信息
            path_info = {
                'path': path,
                'structure': path_structure or {},
                'start_time': start_time,
                'speed': speed,
                'registered_time': time.time(),
                'quality_score': quality_score,
                'planner_used': path_structure.get('planner_used', 'unknown') if path_structure else 'unknown',
                'backbone_utilization': path_structure.get('backbone_utilization', 0.0) if path_structure else 0.0,
                'current_position': path[0],
                'target_position': path[-1],
                'progress': 0.0,
                'estimated_completion_time': start_time + len(path) * 2.0 / speed
            }
            
            # 处理接口预约
            self._handle_interface_reservations(vehicle_id, path_structure)
            
            # 注册路径
            self.active_paths[vehicle_id] = path_info
            
            # 添加到时空预留表
            self._add_to_spacetime_reservation(vehicle_id, path, start_time, speed)
            
            # 记录历史
            self.path_history[vehicle_id].append({
                'timestamp': time.time(),
                'path_length': len(path),
                'quality': quality_score,
                'planner': path_info['planner_used']
            })
            
            # 限制历史长度
            if len(self.path_history[vehicle_id]) > 20:
                self.path_history[vehicle_id].popleft()
            
            print(f"增强路径注册: 车辆{vehicle_id}, 长度{len(path)}, "
                  f"质量{quality_score:.2f}, 规划器{path_info['planner_used']}")
            
            return True
    
    def _handle_interface_reservations(self, vehicle_id: str, path_structure: Dict):
        """处理接口预约"""
        if not path_structure:
            return
        
        interface_id = path_structure.get('interface_id')
        if not interface_id:
            return
        
        with self.interface_lock:
            if interface_id not in self.interface_reservations:
                self.interface_reservations[interface_id] = []
            
            # 估算到达时间
            access_length = path_structure.get('access_length', 0)
            arrival_time = time.time() + access_length * 2.0  # 估算
            duration = 120  # 2分钟预约
            
            # 添加预约
            reservation = (vehicle_id, arrival_time, arrival_time + duration)
            self.interface_reservations[interface_id].append(reservation)
            
            # 按时间排序
            self.interface_reservations[interface_id].sort(key=lambda x: x[1])
    
    def detect_all_conflicts_enhanced(self, current_time: float = 0) -> List[EnhancedConflict]:
        """增强版综合冲突检测"""
        detection_start = time.time()
        all_conflicts = []
        
        with self.state_lock:
            # 1. 实时空间冲突检测
            spatial_conflicts = self._detect_spatial_conflicts_parallel()
            all_conflicts.extend(spatial_conflicts)
            
            # 2. 预测性冲突检测
            if current_time - self.last_prediction_time > self.performance_config['prediction_update_interval']:
                predictive_conflicts = self._detect_predictive_conflicts(current_time)
                all_conflicts.extend(predictive_conflicts)
                self.last_prediction_time = current_time
                self.stats['predictive_conflicts'] += len(predictive_conflicts)
            
            # 3. 接口冲突检测
            interface_conflicts = self._detect_interface_conflicts(current_time)
            all_conflicts.extend(interface_conflicts)
            
            # 4. 骨干路径冲突检测
            backbone_conflicts = self._detect_backbone_conflicts_enhanced()
            all_conflicts.extend(backbone_conflicts)
            
            # 5. 路径质量冲突检测
            quality_conflicts = self._detect_quality_conflicts()
            all_conflicts.extend(quality_conflicts)
        
        # 去重和分类
        unique_conflicts = self._deduplicate_and_classify_conflicts(all_conflicts)
        
        # 更新活跃冲突
        self._update_active_conflicts(unique_conflicts)
        
        # 记录性能指标
        detection_time = time.time() - detection_start
        self.stats['performance_trends']['detection_time'].append(detection_time)
        self.stats['total_conflicts_detected'] += len(unique_conflicts)
        
        if len(unique_conflicts) > 0:
            print(f"增强冲突检测完成: 发现 {len(unique_conflicts)} 个冲突，"
                  f"耗时 {detection_time:.3f}s")
        
        return unique_conflicts
    
    def _detect_spatial_conflicts_parallel(self) -> List[EnhancedConflict]:
        """并行空间冲突检测"""
        if len(self.active_paths) < 2:
            return []
        
        conflicts = []
        vehicle_pairs = []
        
        # 生成车辆对
        vehicle_ids = list(self.active_paths.keys())
        for i in range(len(vehicle_ids)):
            for j in range(i + 1, len(vehicle_ids)):
                vehicle_pairs.append((vehicle_ids[i], vehicle_ids[j]))
        
        # 并行检测
        if len(vehicle_pairs) > 10:  # 大量车辆时使用并行
            batch_size = max(1, len(vehicle_pairs) // self.performance_config['max_detection_threads'])
            batches = [vehicle_pairs[i:i + batch_size] 
                      for i in range(0, len(vehicle_pairs), batch_size)]
            
            futures = []
            for batch in batches:
                future = self.executor.submit(self._detect_batch_spatial_conflicts, batch)
                futures.append(future)
            
            for future in as_completed(futures, timeout=5.0):
                try:
                    batch_conflicts = future.result()
                    conflicts.extend(batch_conflicts)
                except Exception as e:
                    print(f"并行空间冲突检测失败: {e}")
        else:
            # 顺序检测
            conflicts = self._detect_batch_spatial_conflicts(vehicle_pairs)
        
        return conflicts
    
    def _detect_batch_spatial_conflicts(self, vehicle_pairs: List[Tuple[str, str]]) -> List[EnhancedConflict]:
        """批量检测空间冲突"""
        conflicts = []
        
        for vehicle1, vehicle2 in vehicle_pairs:
            if vehicle1 not in self.active_paths or vehicle2 not in self.active_paths:
                continue
            
            path_info1 = self.active_paths[vehicle1]
            path_info2 = self.active_paths[vehicle2]
            
            path1 = path_info1['path']
            path2 = path_info2['path']
            
            # 顶点冲突检测
            vertex_conflicts = self._detect_vertex_conflicts_enhanced(
                vehicle1, path1, path_info1,
                vehicle2, path2, path_info2
            )
            conflicts.extend(vertex_conflicts)
            
            # 边冲突检测
            edge_conflicts = self._detect_edge_conflicts_enhanced(
                vehicle1, path1, path_info1,
                vehicle2, path2, path_info2
            )
            conflicts.extend(edge_conflicts)
        
        return conflicts
    
    def _detect_vertex_conflicts_enhanced(self, agent1: str, path1: List, info1: Dict,
                                        agent2: str, path2: List, info2: Dict) -> List[EnhancedConflict]:
        """增强版顶点冲突检测"""
        conflicts = []
        min_len = min(len(path1), len(path2))
        
        for t in range(min_len):
            p1, p2 = path1[t], path2[t]
            distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
            if distance < self.detection_params['safety_distance']:
                # 计算增强冲突属性
                severity = self._calculate_enhanced_severity(distance, p1, p2, info1, info2)
                confidence = self._calculate_conflict_confidence(distance, info1, info2)
                
                # 计算路径质量影响
                quality_impact = self._calculate_quality_impact(t, path1, path2, info1, info2)
                
                # 检查车辆动力学
                dynamics_violation = self._check_vehicle_dynamics_violation(
                    t, path1, path2, info1, info2
                )
                
                conflict = EnhancedConflict(
                    conflict_id=f"vertex_{agent1}_{agent2}_{t}_{int(time.time())}",
                    agent1=agent1,
                    agent2=agent2,
                    location=((p1[0] + p2[0])/2, (p1[1] + p2[1])/2),
                    time_step=float(t),
                    conflict_type=ConflictType.VERTEX,
                    severity=severity,
                    confidence=confidence,
                    path_quality_impact=quality_impact,
                    vehicle_dynamics_violation=dynamics_violation,
                    recommended_strategy=self._recommend_resolution_strategy(
                        ConflictType.VERTEX, severity, info1, info2
                    )
                )
                
                conflicts.append(conflict)
        
        return conflicts
    
    def _detect_edge_conflicts_enhanced(self, agent1: str, path1: List, info1: Dict,
                                      agent2: str, path2: List, info2: Dict) -> List[EnhancedConflict]:
        """增强版边冲突检测"""
        conflicts = []
        min_len = min(len(path1), len(path2)) - 1
        
        for t in range(min_len):
            # 检查位置交换
            if (self._points_close(path1[t], path2[t+1]) and 
                self._points_close(path1[t+1], path2[t])):
                
                # 计算交换点的中心
                center_x = (path1[t][0] + path2[t][0] + path1[t+1][0] + path2[t+1][0]) / 4
                center_y = (path1[t][1] + path2[t][1] + path1[t+1][1] + path2[t+1][1]) / 4
                
                conflict = EnhancedConflict(
                    conflict_id=f"edge_{agent1}_{agent2}_{t}_{int(time.time())}",
                    agent1=agent1,
                    agent2=agent2,
                    location=(center_x, center_y),
                    time_step=float(t + 0.5),
                    conflict_type=ConflictType.EDGE,
                    severity=ConflictSeverity.HIGH,  # 边冲突较严重
                    confidence=0.95,
                    path_quality_impact=0.8,  # 边冲突对质量影响较大
                    recommended_strategy=ResolutionStrategy.REPLANNING
                )
                
                conflicts.append(conflict)
        
        return conflicts
    
    def _detect_predictive_conflicts(self, current_time: float) -> List[EnhancedConflict]:
        """检测预测性冲突"""
        try:
            # 准备车辆路径数据
            vehicle_paths = {}
            for vehicle_id, path_info in self.active_paths.items():
                vehicle_paths[vehicle_id] = {
                    'path': path_info['path'],
                    'speed': path_info['speed'],
                    'current_position': path_info['current_position'],
                    'progress': path_info.get('progress', 0.0)
                }
            
            # 使用预测器检测冲突
            predicted_conflicts = self.conflict_predictor.predict_conflicts(
                vehicle_paths, current_time
            )
            
            return predicted_conflicts
        
        except Exception as e:
            print(f"预测性冲突检测失败: {e}")
            return []
    
    def _detect_interface_conflicts(self, current_time: float) -> List[EnhancedConflict]:
        """检测接口冲突"""
        conflicts = []
        
        with self.interface_lock:
            for interface_id, reservations in self.interface_reservations.items():
                # 清理过期预约
                valid_reservations = [
                    res for res in reservations if res[2] > current_time
                ]
                self.interface_reservations[interface_id] = valid_reservations
                
                # 检查重叠预约
                for i in range(len(valid_reservations)):
                    for j in range(i + 1, len(valid_reservations)):
                        res1, res2 = valid_reservations[i], valid_reservations[j]
                        
                        # 检查时间重叠
                        if not (res1[2] <= res2[1] or res1[1] >= res2[2]):
                            overlap_time = min(res1[2], res2[2]) - max(res1[1], res2[1])
                            
                            conflict = EnhancedConflict(
                                conflict_id=f"interface_{interface_id}_{res1[0]}_{res2[0]}",
                                agent1=res1[0],
                                agent2=res2[0],
                                location=(0, 0),  # 接口位置
                                time_step=max(res1[1], res2[1]),
                                conflict_type=ConflictType.INTERFACE,
                                severity=ConflictSeverity.HIGH,
                                confidence=1.0,
                                interface_id=interface_id,
                                resource_type='interface',
                                recommended_strategy=ResolutionStrategy.TIME_SHIFT
                            )
                            conflicts.append(conflict)
        
        return conflicts
    
    def _detect_backbone_conflicts_enhanced(self) -> List[EnhancedConflict]:
        """增强版骨干路径冲突检测"""
        conflicts = []
        
        if not self.backbone_network:
            return conflicts
        
        # 按骨干路径分组
        backbone_usage = defaultdict(list)
        
        for vehicle_id, path_info in self.active_paths.items():
            structure = path_info.get('structure', {})
            backbone_path_id = structure.get('backbone_path_id')
            
            if backbone_path_id:
                backbone_usage[backbone_path_id].append({
                    'vehicle_id': vehicle_id,
                    'start_time': path_info['start_time'],
                    'access_length': structure.get('access_length', 0),
                    'backbone_length': structure.get('backbone_length', 0),
                    'quality': path_info.get('quality_score', 0.5)
                })
        
        # 检查每条骨干路径的冲突
        for backbone_path_id, users in backbone_usage.items():
            if len(users) <= 1:
                continue
            
            # 按时间排序
            users.sort(key=lambda u: u['start_time'] + u['access_length'])
            
            for i in range(len(users) - 1):
                current_user = users[i]
                next_user = users[i + 1]
                
                # 计算时间重叠
                current_end = (current_user['start_time'] + current_user['access_length'] + 
                              current_user['backbone_length'])
                next_start = next_user['start_time'] + next_user['access_length']
                
                if next_start < current_end:
                    overlap = current_end - next_start
                    
                    # 根据重叠程度确定严重性
                    if overlap > 60:
                        severity = ConflictSeverity.CRITICAL
                    elif overlap > 30:
                        severity = ConflictSeverity.HIGH
                    else:
                        severity = ConflictSeverity.MEDIUM
                    
                    conflict = EnhancedConflict(
                        conflict_id=f"backbone_{backbone_path_id}_{current_user['vehicle_id']}_{next_user['vehicle_id']}",
                        agent1=current_user['vehicle_id'],
                        agent2=next_user['vehicle_id'],
                        location=(0, 0),
                        time_step=next_start,
                        conflict_type=ConflictType.BACKBONE,
                        severity=severity,
                        confidence=0.9,
                        backbone_path_id=backbone_path_id,
                        recommended_strategy=ResolutionStrategy.TIME_SHIFT
                    )
                    conflicts.append(conflict)
        
        return conflicts
    
    def _detect_quality_conflicts(self) -> List[EnhancedConflict]:
        """检测路径质量冲突"""
        conflicts = []
        quality_threshold = 0.6
        
        for vehicle_id, path_info in self.active_paths.items():
            quality = path_info.get('quality_score', 0.5)
            
            if quality < quality_threshold:
                # 低质量路径标记为需要优化的"冲突"
                conflict = EnhancedConflict(
                    conflict_id=f"quality_{vehicle_id}_{int(time.time())}",
                    agent1=vehicle_id,
                    agent2="system",
                    location=(0, 0),
                    time_step=0,
                    conflict_type=ConflictType.DYNAMIC,
                    severity=ConflictSeverity.LOW,
                    confidence=1.0,
                    path_quality_impact=1.0 - quality,
                    recommended_strategy=ResolutionStrategy.REPLANNING
                )
                conflicts.append(conflict)
        
        return conflicts
    
    def resolve_conflicts_enhanced(self, conflicts: List[EnhancedConflict], 
                                  current_time: float = 0) -> Dict[str, List]:
        """增强版冲突解决"""
        if not conflicts:
            current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
            return current_paths
        
        resolution_start = time.time()
        
        # 使用分层ECBS求解器
        current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
        resolved_paths = self.ecbs_solver.solve_hierarchical(
            current_paths, conflicts, current_time
        )
        
        # 验证解决方案质量
        if resolved_paths != current_paths:
            quality_improved = self._validate_resolution_quality(
                current_paths, resolved_paths
            )
            
            if quality_improved:
                self.stats['quality_improvements'] += 1
            
            # 更新路径信息
            self._update_resolved_paths(resolved_paths)
        
        # 记录解决统计
        resolution_time = time.time() - resolution_start
        self.stats['performance_trends']['resolution_time'].append(resolution_time)
        self.stats['conflicts_resolved'] += len(conflicts)
        
        # 更新平均解决时间
        if self.stats['conflicts_resolved'] > 0:
            total_time = sum(self.stats['performance_trends']['resolution_time'])
            self.stats['average_resolution_time'] = total_time / len(self.stats['performance_trends']['resolution_time'])
        
        print(f"增强冲突解决完成: 处理了 {len(conflicts)} 个冲突，"
              f"耗时 {resolution_time:.3f}s")
        
        return resolved_paths
    
    def _validate_resolution_quality(self, original_paths: Dict[str, List], 
                                   resolved_paths: Dict[str, List]) -> bool:
        """验证解决方案质量"""
        quality_improvements = 0
        total_comparisons = 0
        
        for vehicle_id in resolved_paths:
            if vehicle_id in original_paths:
                original_quality = self.quality_evaluator.evaluate_path(
                    original_paths[vehicle_id]
                )
                resolved_quality = self.quality_evaluator.evaluate_path(
                    resolved_paths[vehicle_id]
                )
                
                if resolved_quality > original_quality + 0.05:  # 5%改善阈值
                    quality_improvements += 1
                
                total_comparisons += 1
        
        return quality_improvements >= total_comparisons * 0.3  # 30%路径改善
    
    def _update_resolved_paths(self, resolved_paths: Dict[str, List]):
        """更新解决后的路径"""
        for vehicle_id, new_path in resolved_paths.items():
            if vehicle_id in self.active_paths:
                path_info = self.active_paths[vehicle_id]
                
                if path_info['path'] != new_path:
                    # 更新路径信息
                    old_quality = path_info.get('quality_score', 0.5)
                    new_quality = self.quality_evaluator.evaluate_path(new_path)
                    
                    path_info['path'] = new_path
                    path_info['quality_score'] = new_quality
                    path_info['last_update'] = time.time()
                    
                    print(f"车辆 {vehicle_id} 路径已更新，质量: {old_quality:.2f} -> {new_quality:.2f}")
    
    def update_performance_monitoring(self, time_delta: float):
        """更新性能监控"""
        current_time = time.time()
        
        # 定期检测
        if current_time - self.last_detection_time > self.performance_config['conflict_detection_interval']:
            conflicts = self.detect_all_conflicts_enhanced(current_time)
            
            if conflicts:
                self.resolve_conflicts_enhanced(conflicts, current_time)
            
            self.last_detection_time = current_time
        
        # 质量优化
        if current_time - self.last_optimization_time > self.performance_config['quality_optimization_interval']:
            self._optimize_low_quality_paths()
            self.last_optimization_time = current_time
        
        # 缓存清理
        if current_time - self.last_cache_cleanup > self.performance_config['cache_cleanup_interval']:
            self._cleanup_caches()
            self.last_cache_cleanup = current_time
        
        # 更新车辆位置
        self._update_vehicle_positions(time_delta)
    
    def _optimize_low_quality_paths(self):
        """优化低质量路径"""
        if not self.path_planner:
            return
        
        optimization_count = 0
        
        for vehicle_id, path_info in self.active_paths.items():
            quality = path_info.get('quality_score', 0.5)
            
            if quality < 0.6:  # 低质量阈值
                try:
                    path = path_info['path']
                    if len(path) >= 2:
                        result = self.path_planner.plan_path(
                            vehicle_id, path[0], path[-1],
                            use_backbone=True,
                            check_conflicts=False,
                            planner_type="hybrid_astar"
                        )
                        
                        if result:
                            if isinstance(result, tuple):
                                new_path, structure = result
                                new_quality = structure.get('final_quality', quality)
                            else:
                                new_path = result
                                new_quality = self.quality_evaluator.evaluate_path(new_path)
                            
                            if new_quality > quality + 0.1:  # 显著改善
                                path_info['path'] = new_path
                                path_info['quality_score'] = new_quality
                                optimization_count += 1
                                
                                print(f"路径质量优化: 车辆{vehicle_id}, "
                                      f"{quality:.2f} -> {new_quality:.2f}")
                
                except Exception as e:
                    print(f"路径优化失败 {vehicle_id}: {e}")
        
        if optimization_count > 0:
            self.stats['quality_improvements'] += optimization_count
    
    def _update_vehicle_positions(self, time_delta: float):
        """更新车辆位置信息"""
        for vehicle_id, path_info in self.active_paths.items():
            if vehicle_id in self.env.vehicles:
                env_vehicle = self.env.vehicles[vehicle_id]
                
                # 更新当前位置
                current_pos = env_vehicle.get('position', path_info['current_position'])
                path_info['current_position'] = current_pos
                
                # 更新进度
                progress = env_vehicle.get('progress', path_info.get('progress', 0.0))
                path_info['progress'] = progress
                
                # 估算完成时间
                if progress > 0:
                    elapsed_time = time.time() - path_info['start_time']
                    if progress > 0.1:  # 有足够进度来估算
                        estimated_total_time = elapsed_time / progress
                        path_info['estimated_completion_time'] = (
                            path_info['start_time'] + estimated_total_time
                        )
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息"""
        stats = self.stats.copy()
        
        # 添加实时状态
        stats['real_time'] = {
            'active_vehicles': len(self.active_paths),
            'active_conflicts': len(self.active_conflicts),
            'interface_reservations': sum(len(reservations) for reservations in self.interface_reservations.values()),
            'average_path_quality': self._calculate_average_path_quality(),
            'system_load': len(self.active_paths) / max(1, len(self.env.vehicles))
        }
        
        # 计算缓存命中率
        if hasattr(self.ecbs_solver, 'stats'):
            solver_stats = self.ecbs_solver.stats
            total_cache_operations = solver_stats.get('cache_hits', 0) + solver_stats.get('nodes_expanded', 0)
            if total_cache_operations > 0:
                stats['cache_hit_rate'] = solver_stats.get('cache_hits', 0) / total_cache_operations
        
        # 性能趋势摘要
        stats['performance_summary'] = {}
        for metric, values in self.stats['performance_trends'].items():
            if values:
                stats['performance_summary'][metric] = {
                    'average': sum(values) / len(values),
                    'recent': values[-1] if values else 0,
                    'trend': 'improving' if len(values) >= 2 and values[-1] < values[0] else 'stable'
                }
        
        # ECBS求解器统计
        if hasattr(self.ecbs_solver, 'stats'):
            stats['ecbs_solver'] = self.ecbs_solver.stats.copy()
        
        return stats
    
    def _calculate_average_path_quality(self) -> float:
        """计算平均路径质量"""
        if not self.active_paths:
            return 0.0
        
        total_quality = sum(info.get('quality_score', 0.5) for info in self.active_paths.values())
        return total_quality / len(self.active_paths)
    
    # 辅助方法
    def _calculate_enhanced_severity(self, distance: float, p1: Tuple, p2: Tuple, 
                                   info1: Dict, info2: Dict) -> ConflictSeverity:
        """计算增强严重性"""
        base_severity = self.detection_params['safety_distance'] / (distance + 0.1)
        
        # 质量因子
        quality_factor = 2.0 - (info1.get('quality_score', 0.5) + info2.get('quality_score', 0.5))
        
        # 速度因子
        speed_factor = (info1.get('speed', 1.0) + info2.get('speed', 1.0)) / 2.0
        
        severity_score = base_severity * quality_factor * speed_factor
        
        if severity_score > 8:
            return ConflictSeverity.EMERGENCY
        elif severity_score > 6:
            return ConflictSeverity.CRITICAL
        elif severity_score > 4:
            return ConflictSeverity.HIGH
        elif severity_score > 2:
            return ConflictSeverity.MEDIUM
        else:
            return ConflictSeverity.LOW
    
    def _calculate_conflict_confidence(self, distance: float, info1: Dict, info2: Dict) -> float:
        """计算冲突置信度"""
        distance_confidence = max(0, 1.0 - distance / self.detection_params['safety_distance'])
        quality_confidence = (info1.get('quality_score', 0.5) + info2.get('quality_score', 0.5)) / 2.0
        
        return (distance_confidence + quality_confidence) / 2.0
    
    def _calculate_quality_impact(self, time_step: int, path1: List, path2: List, 
                                info1: Dict, info2: Dict) -> float:
        """计算路径质量影响"""
        # 基于路径长度和位置的影响评估
        impact = 0.5  # 基础影响
        
        # 路径前端冲突影响更大
        if time_step < min(len(path1), len(path2)) * 0.3:
            impact += 0.3
        
        # 低质量路径的冲突影响更大
        avg_quality = (info1.get('quality_score', 0.5) + info2.get('quality_score', 0.5)) / 2.0
        impact += (1.0 - avg_quality) * 0.4
        
        return min(1.0, impact)
    
    def _check_vehicle_dynamics_violation(self, time_step: int, path1: List, path2: List,
                                        info1: Dict, info2: Dict) -> bool:
        """检查车辆动力学违规"""
        # 简化检查：基于路径曲率
        for path in [path1, path2]:
            if time_step >= 1 and time_step < len(path) - 1:
                curvature = self._calculate_path_curvature(
                    path[time_step-1], path[time_step], path[time_step+1]
                )
                if curvature > 0.5:  # 曲率阈值
                    return True
        
        return False
    
    def _calculate_path_curvature(self, p1: Tuple, p2: Tuple, p3: Tuple) -> float:
        """计算路径曲率"""
        # 使用角度变化计算曲率
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 0.001 or len2 < 0.001:
            return 0.0
        
        cos_angle = np.dot(v1, v2) / (len1 * len2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = math.acos(cos_angle)
        
        avg_length = (len1 + len2) / 2
        return angle / (avg_length + 0.001)
    
    def _recommend_resolution_strategy(self, conflict_type: ConflictType, 
                                     severity: ConflictSeverity, 
                                     info1: Dict, info2: Dict) -> ResolutionStrategy:
        """推荐解决策略"""
        if conflict_type == ConflictType.INTERFACE:
            return ResolutionStrategy.INTERFACE_SWITCH
        elif conflict_type == ConflictType.BACKBONE:
            return ResolutionStrategy.ALTERNATIVE_PATH
        elif severity.value >= 4:
            return ResolutionStrategy.REPLANNING
        elif severity.value >= 3:
            return ResolutionStrategy.TIME_SHIFT
        else:
            return ResolutionStrategy.COOPERATIVE
    
    def _points_close(self, p1: Tuple, p2: Tuple, threshold: float = None) -> bool:
        """检查两点是否接近"""
        if threshold is None:
            threshold = self.detection_params['safety_distance']
        
        distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return distance < threshold
    
    def _deduplicate_and_classify_conflicts(self, conflicts: List[EnhancedConflict]) -> List[EnhancedConflict]:
        """去重和分类冲突"""
        unique_conflicts = {}
        
        for conflict in conflicts:
            # 创建唯一键
            agent_pair = tuple(sorted([conflict.agent1, conflict.agent2]))
            location_key = (round(conflict.location[0]), round(conflict.location[1]))
            time_key = round(conflict.time_step)
            
            key = (agent_pair, conflict.conflict_type, location_key, time_key)
            
            if key not in unique_conflicts:
                unique_conflicts[key] = conflict
            else:
                # 保留严重性更高的冲突
                existing = unique_conflicts[key]
                if conflict.severity.value > existing.severity.value:
                    unique_conflicts[key] = conflict
        
        return sorted(unique_conflicts.values(), reverse=True)
    
    def _update_active_conflicts(self, conflicts: List[EnhancedConflict]):
        """更新活跃冲突"""
        # 清理旧冲突
        current_time = time.time()
        expired_conflicts = []
        
        for conflict_id, conflict in self.active_conflicts.items():
            if current_time - conflict.creation_time > 300:  # 5分钟超时
                expired_conflicts.append(conflict_id)
        
        for conflict_id in expired_conflicts:
            self.resolved_conflicts.append(self.active_conflicts[conflict_id])
            del self.active_conflicts[conflict_id]
        
        # 添加新冲突
        for conflict in conflicts:
            self.active_conflicts[conflict.conflict_id] = conflict
    
    def _add_to_spacetime_reservation(self, vehicle_id: str, path: List, 
                                    start_time: float, speed: float):
        """添加到时空预留表"""
        current_time = start_time
        
        for i in range(len(path) - 1):
            segment_length = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            segment_time = segment_length / speed
            
            # 离散化时间和空间
            time_slot = int(current_time / self.detection_params['time_discretization'])
            space_key = (int(path[i][0] / 5), int(path[i][1] / 5))  # 5x5网格
            
            reservation_key = (space_key, time_slot)
            
            if reservation_key not in self.path_reservations:
                self.path_reservations[reservation_key] = []
            
            if vehicle_id not in self.path_reservations[reservation_key]:
                self.path_reservations[reservation_key].append(vehicle_id)
            
            current_time += segment_time
    
    def _cleanup_caches(self):
        """清理缓存"""
        current_time = time.time()
        
        # 清理解决方案缓存
        expired_keys = []
        for key, (solution, timestamp) in self.conflict_resolution_cache.items():
            if current_time - timestamp > 300:  # 5分钟过期
                expired_keys.append(key)
        
        for key in expired_keys:
            del self.conflict_resolution_cache[key]
        
        # 清理时空预留表
        expired_reservations = []
        for key, vehicles in self.path_reservations.items():
            space_key, time_slot = key
            reservation_time = time_slot * self.detection_params['time_discretization']
            
            if current_time - reservation_time > 600:  # 10分钟过期
                expired_reservations.append(key)
        
        for key in expired_reservations:
            del self.path_reservations[key]
        
        print(f"缓存清理完成: 移除了 {len(expired_keys)} 个解决方案缓存项，"
              f"{len(expired_reservations)} 个时空预留项")
    
    def release_vehicle_path(self, vehicle_id: str) -> bool:
        """释放车辆路径"""
        with self.state_lock:
            if vehicle_id not in self.active_paths:
                return False
            
            # 释放接口预约
            with self.interface_lock:
                for interface_id, reservations in self.interface_reservations.items():
                    self.interface_reservations[interface_id] = [
                        res for res in reservations if res[0] != vehicle_id
                    ]
            
            # 移除时空预留
            expired_reservations = []
            for key, vehicles in self.path_reservations.items():
                if vehicle_id in vehicles:
                    vehicles.remove(vehicle_id)
                    if not vehicles:
                        expired_reservations.append(key)
            
            for key in expired_reservations:
                del self.path_reservations[key]
            
            # 移除活跃路径
            del self.active_paths[vehicle_id]
            
            return True
    
    def set_backbone_network(self, backbone_network):
        """设置骨干路径网络"""
        self.backbone_network = backbone_network
        self.ecbs_solver.backbone_network = backbone_network
        print("已更新交通管理器的骨干路径网络引用")
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        self.ecbs_solver.path_planner = path_planner
        print("已更新交通管理器的路径规划器引用")
    
    def shutdown(self):
        """关闭交通管理器"""
        print("关闭智能交通管理器...")
        
        # 关闭线程池
        self.executor.shutdown(wait=True)
        
        # 清理资源
        with self.state_lock:
            self.active_paths.clear()
            self.active_conflicts.clear()
            self.path_reservations.clear()
            
        with self.interface_lock:
            self.interface_reservations.clear()
        
        print("智能交通管理器已关闭")

# 向后兼容性
OptimizedTrafficManager = IntegratedTrafficManager
TrafficManager = IntegratedTrafficManager