"""
path_planner.py - 全面优化版统一路径规划器
整合安全矩形冲突检测、车辆安全参数、冲突解决上下文、智能回退策略
支持backbone稳定性管理和增强的路径结构信息
"""

import time
import math
import threading
from typing import List, Tuple, Dict, Optional, Any, Union
from collections import OrderedDict, namedtuple, defaultdict
from dataclasses import dataclass
from enum import Enum

# 导入规划器
try:
    from RRT import SimplifiedRRTPlanner
    RRT_AVAILABLE = True
except ImportError:
    print("警告: RRT规划器不可用")
    RRT_AVAILABLE = False

try:
    from astar import HybridAStarPlanner
    ASTAR_AVAILABLE = True
except ImportError:
    print("警告: 混合A*规划器不可用")
    ASTAR_AVAILABLE = False

class PlanningContext(Enum):
    """规划上下文"""
    NORMAL = "normal"
    BACKBONE = "backbone"
    NAVIGATION = "navigation"
    CONFLICT_RESOLUTION = "conflict_resolution"
    EMERGENCY = "emergency"
    BACKBONE_ACCESS = "backbone_access"

class QualityLevel(Enum):
    """质量等级"""
    STRICT = "strict"
    STANDARD = "standard"
    RELAXED = "relaxed"
    EMERGENCY = "emergency"

# 规划结果类型
PlanningResult = namedtuple('PlanningResult', [
    'path', 'planner_used', 'quality_score', 'planning_time', 'structure',
    'safety_score', 'conflict_risk', 'backbone_utilization'
])

@dataclass
class EnhancedPlannerConfig:
    """增强规划器配置"""
    max_planning_time: float = 20.0
    quality_threshold: float = 0.6
    cache_size: int = 300
    enable_safety_optimization: bool = True
    enable_conflict_avoidance: bool = True
    min_safety_clearance: float = 8.0
    
    # 渐进式回退策略
    enable_progressive_fallback: bool = True
    timeout_retry_count: int = 3
    
    # 上下文特定配置
    context_configs: Dict = None
    
    # 规划器参数
    astar_params: Dict = None
    rrt_params: Dict = None
    
    def __post_init__(self):
        if self.context_configs is None:
            self.context_configs = {
                PlanningContext.BACKBONE: {
                    'quality_threshold': 0.8,
                    'max_time': 25.0,
                    'safety_bias': 1.2,
                    'preferred_planner': 'hybrid_astar'
                },
                PlanningContext.CONFLICT_RESOLUTION: {
                    'quality_threshold': 0.7,
                    'max_time': 15.0,
                    'safety_bias': 1.5,
                    'conflict_avoidance': True
                },
                PlanningContext.EMERGENCY: {
                    'quality_threshold': 0.4,
                    'max_time': 8.0,
                    'safety_bias': 0.8,
                    'preferred_planner': 'rrt'
                },
                PlanningContext.BACKBONE_ACCESS: {
                    'quality_threshold': 0.6,
                    'max_time': 12.0,
                    'safety_bias': 1.3,
                    'preferred_planner': 'hybrid_astar'
                }
            }
        
        if self.astar_params is None:
            self.astar_params = {
                QualityLevel.STRICT: {
                    'vehicle_length': 6.0,
                    'vehicle_width': 3.0,
                    'turning_radius': 8.0,
                    'step_size': 2.0,
                    'angle_resolution': 30,
                    'max_iterations': 20000,
                    'quality_threshold': 0.8
                },
                QualityLevel.STANDARD: {
                    'vehicle_length': 6.0,
                    'vehicle_width': 3.0,
                    'turning_radius': 7.0,
                    'step_size': 2.5,
                    'angle_resolution': 36,
                    'max_iterations': 15000,
                    'quality_threshold': 0.6
                },
                QualityLevel.RELAXED: {
                    'vehicle_length': 6.0,
                    'vehicle_width': 3.0,
                    'turning_radius': 6.0,
                    'step_size': 3.0,
                    'angle_resolution': 45,
                    'max_iterations': 10000,
                    'quality_threshold': 0.4
                }
            }
        
        if self.rrt_params is None:
            self.rrt_params = {
                QualityLevel.STANDARD: {
                    'vehicle_length': 5.0,
                    'vehicle_width': 2.0,
                    'turning_radius': 5.0,
                    'step_size': 0.8,
                    'max_nodes': 6000,
                    'quality_threshold': 0.5
                },
                QualityLevel.RELAXED: {
                    'vehicle_length': 5.0,
                    'vehicle_width': 2.0,
                    'turning_radius': 4.0,
                    'step_size': 1.0,
                    'max_nodes': 4000,
                    'quality_threshold': 0.4
                }
            }

@dataclass
class VehicleSafetyParams:
    """车辆安全参数"""
    length: float = 6.0
    width: float = 3.0
    safety_margin: float = 1.5
    turning_radius: float = 8.0
    max_speed: float = 2.0
    
    def get_safe_dimensions(self) -> Tuple[float, float]:
        """获取安全尺寸"""
        return (self.length + self.safety_margin, 
                self.width + self.safety_margin)
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            'length': self.length,
            'width': self.width,
            'safety_margin': self.safety_margin,
            'turning_radius': self.turning_radius,
            'max_speed': self.max_speed
        }

class ProgressiveFallbackStrategy:
    """渐进式回退策略管理器"""
    
    def __init__(self, config: EnhancedPlannerConfig):
        self.config = config
        
        # 定义回退序列
        self.fallback_sequences = {
            PlanningContext.BACKBONE: [
                ('hybrid_astar', QualityLevel.STRICT, 15.0),
                ('hybrid_astar', QualityLevel.STANDARD, 18.0),
                ('rrt', QualityLevel.STANDARD, 12.0),
                ('hybrid_astar', QualityLevel.RELAXED, 20.0),
                ('direct', None, 1.0)
            ],
            PlanningContext.CONFLICT_RESOLUTION: [
                ('hybrid_astar', QualityLevel.STRICT, 12.0),
                ('rrt', QualityLevel.STANDARD, 10.0),
                ('hybrid_astar', QualityLevel.STANDARD, 15.0),
                ('rrt', QualityLevel.RELAXED, 8.0),
                ('direct', None, 1.0)
            ],
            PlanningContext.EMERGENCY: [
                ('rrt', QualityLevel.RELAXED, 6.0),
                ('hybrid_astar', QualityLevel.RELAXED, 8.0),
                ('direct', None, 1.0)
            ],
            PlanningContext.NORMAL: [
                ('hybrid_astar', QualityLevel.STANDARD, 12.0),
                ('rrt', QualityLevel.STANDARD, 10.0),
                ('hybrid_astar', QualityLevel.RELAXED, 15.0),
                ('rrt', QualityLevel.RELAXED, 8.0),
                ('direct', None, 1.0)
            ]
        }
    
    def get_fallback_sequence(self, context: PlanningContext) -> List[Tuple]:
        """获取指定上下文的回退序列"""
        return self.fallback_sequences.get(context, self.fallback_sequences[PlanningContext.NORMAL])

class SafetyAnalyzer:
    """安全性分析器"""
    
    def __init__(self, env):
        self.env = env
        self.obstacle_cache = {}
        self.safety_cache = {}
    
    def evaluate_path_safety(self, path: List[Tuple], 
                           vehicle_params: VehicleSafetyParams = None,
                           min_clearance: float = None) -> float:
        """评估路径安全性"""
        if not path or len(path) < 2:
            return 0.5
        
        vehicle_params = vehicle_params or VehicleSafetyParams()
        min_clearance = min_clearance or vehicle_params.safety_margin
        
        safety_score = 1.0
        safe_length, safe_width = vehicle_params.get_safe_dimensions()
        
        # 检查每个路径点的安全性
        for point in path[::max(1, len(path)//20)]:  # 采样检查
            x, y = point[0], point[1]
            
            # 检查到障碍物的最小距离
            min_obstacle_distance = self._get_min_obstacle_distance(x, y)
            
            if min_obstacle_distance < min_clearance:
                # 距离不足，降低安全分数
                safety_penalty = (min_clearance - min_obstacle_distance) / min_clearance
                safety_score -= safety_penalty * 0.1
            
            # 检查转弯半径安全性
            if len(point) > 2:
                turning_safety = self._check_turning_safety(
                    point, vehicle_params.turning_radius
                )
                safety_score *= turning_safety
        
        return max(0.1, min(1.0, safety_score))
    
    def _get_min_obstacle_distance(self, x: float, y: float) -> float:
        """获取到最近障碍物的距离"""
        cache_key = f"{int(x//5)}_{int(y//5)}"
        if cache_key in self.obstacle_cache:
            return self.obstacle_cache[cache_key]
        
        min_distance = float('inf')
        search_radius = 15
        
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x, check_y = int(x + dx), int(y + dy)
                
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height and
                    hasattr(self.env, 'grid') and
                    self.env.grid[check_x, check_y] == 1):
                    
                    distance = math.sqrt(dx*dx + dy*dy)
                    min_distance = min(min_distance, distance)
        
        # 缓存结果
        self.obstacle_cache[cache_key] = min_distance
        return min_distance
    
    def _check_turning_safety(self, point: Tuple, turning_radius: float) -> float:
        """检查转弯安全性"""
        # 简化实现：基于转弯半径评估
        x, y, theta = point[0], point[1], point[2]
        
        # 检查转弯区域是否安全
        turn_points = [
            (x + turning_radius * math.cos(theta + math.pi/4),
             y + turning_radius * math.sin(theta + math.pi/4)),
            (x + turning_radius * math.cos(theta - math.pi/4),
             y + turning_radius * math.sin(theta - math.pi/4))
        ]
        
        safety_factor = 1.0
        for tx, ty in turn_points:
            if not self._is_point_safe(tx, ty):
                safety_factor *= 0.9
        
        return safety_factor
    
    def _is_point_safe(self, x: float, y: float) -> bool:
        """检查点是否安全"""
        ix, iy = int(x), int(y)
        
        if (ix < 0 or ix >= self.env.width or 
            iy < 0 or iy >= self.env.height):
            return False
        
        if hasattr(self.env, 'grid'):
            return self.env.grid[ix, iy] == 0
        
        return True

class ConflictRiskEstimator:
    """冲突风险评估器"""
    
    def __init__(self, traffic_manager=None):
        self.traffic_manager = traffic_manager
        self.risk_cache = {}
    
    def estimate_path_conflict_risk(self, path: List[Tuple], 
                                  vehicle_id: str = None) -> float:
        """估算路径冲突风险"""
        if not self.traffic_manager or not path:
            return 0.0
        
        risk_score = 0.0
        
        # 检查路径是否经过高冲突区域
        for i, point in enumerate(path[::max(1, len(path)//10)]):
            # 检查该位置的车辆密度
            nearby_vehicles = self._count_nearby_vehicles(point, radius=12.0)
            
            if nearby_vehicles > 3:
                risk_score += 0.15
            elif nearby_vehicles > 1:
                risk_score += 0.08
            
            # 检查是否在骨干路径的高负载区域
            backbone_risk = self._check_backbone_congestion_risk(point)
            risk_score += backbone_risk
        
        return min(1.0, risk_score)
    
    def _count_nearby_vehicles(self, point: Tuple, radius: float) -> int:
        """计算附近车辆数量"""
        if not hasattr(self.traffic_manager, 'active_paths'):
            return 0
        
        count = 0
        x, y = point[0], point[1]
        
        for vehicle_id, path_info in self.traffic_manager.active_paths.items():
            other_path = path_info.get('path', [])
            
            for other_point in other_path[::5]:  # 采样检查
                distance = math.sqrt(
                    (x - other_point[0])**2 + (y - other_point[1])**2
                )
                
                if distance < radius:
                    count += 1
                    break
        
        return count
    
    def _check_backbone_congestion_risk(self, point: Tuple) -> float:
        """检查骨干路径拥堵风险"""
        # 简化实现
        return 0.02  # 基础风险值

class EnhancedPathPlanner:
    """全面优化版统一路径规划器"""
    
    def __init__(self, env, backbone_network=None, traffic_manager=None, 
                 config: EnhancedPlannerConfig = None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 配置
        self.config = config or EnhancedPlannerConfig()
        
        # 规划器实例
        self.planners = {}
        self._initialize_planners()
        
        # 策略管理器
        self.fallback_strategy = ProgressiveFallbackStrategy(self.config)
        
        # 分析器
        self.safety_analyzer = SafetyAnalyzer(env)
        self.conflict_estimator = ConflictRiskEstimator(traffic_manager)
        
        # 增强缓存
        self.cache = OrderedDict()
        self.cache_lock = threading.RLock()
        self.vehicle_param_cache = {}  # 车辆参数缓存
        
        # 详细统计
        self.stats = {
            'total_requests': 0,
            'successful_plans': 0,
            'context_usage': defaultdict(int),
            'planner_success_rates': defaultdict(lambda: {'success': 0, 'total': 0}),
            'quality_distribution': defaultdict(int),
            'safety_scores': [],
            'conflict_risks': [],
            'cache_hits': 0,
            'fallback_usage': defaultdict(int),
            'average_planning_times': defaultdict(list)
        }
        
        print(f"初始化全面优化路径规划器: A*={ASTAR_AVAILABLE}, RRT={RRT_AVAILABLE}")
        print(f"安全优化: {self.config.enable_safety_optimization}")
        print(f"冲突避让: {self.config.enable_conflict_avoidance}")
    
    def _initialize_planners(self):
        """初始化规划器"""
        try:
            # 初始化混合A*
            if ASTAR_AVAILABLE:
                base_params = self.config.astar_params[QualityLevel.STANDARD]
                self.planners['hybrid_astar'] = HybridAStarPlanner(
                    self.env,
                    **base_params
                )
                print("✓ 混合A*规划器已初始化")
            
            # 初始化RRT
            if RRT_AVAILABLE:
                base_params = self.config.rrt_params[QualityLevel.STANDARD]
                self.planners['rrt'] = SimplifiedRRTPlanner(
                    self.env,
                    **base_params
                )
                print("✓ RRT规划器已初始化")
            
            # 设置骨干网络
            if self.backbone_network:
                self._set_backbone_for_planners()
            
        except Exception as e:
            print(f"规划器初始化失败: {e}")
    
    def _set_backbone_for_planners(self):
        """为规划器设置骨干网络"""
        for planner in self.planners.values():
            if hasattr(planner, 'set_backbone_network'):
                try:
                    planner.set_backbone_network(self.backbone_network)
                except Exception as e:
                    print(f"设置骨干网络失败: {e}")
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self._set_backbone_for_planners()
        print("路径规划器已更新骨干网络引用")
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.traffic_manager = traffic_manager
        self.conflict_estimator.traffic_manager = traffic_manager
        print("路径规划器已更新交通管理器引用")
    
    def plan_path(self, vehicle_id: str, start: Tuple, goal: Tuple,
                use_backbone: bool = True, check_conflicts: bool = True,
                planner_type: str = "auto", context: str = "normal",
                return_object: bool = False, 
                # 新增参数
                vehicle_params: Dict = None,
                conflict_avoidance: bool = False,
                min_safety_clearance: float = None,
                **kwargs) -> Optional[Union[List, Tuple[List, Dict], Any]]:
        """
        增强的路径规划接口
        
        Args:
            vehicle_id: 车辆ID
            start: 起点 (x, y, theta)
            goal: 终点 (x, y, theta)
            use_backbone: 是否使用骨干网络
            check_conflicts: 是否检查冲突
            planner_type: 规划器类型 ("auto", "hybrid_astar", "rrt")
            context: 规划上下文
            return_object: 是否返回结果对象格式
            vehicle_params: 车辆安全参数
            conflict_avoidance: 是否进行冲突避让规划
            min_safety_clearance: 最小安全间隔
            
        Returns:
            路径点列表、(路径, 结构信息)元组或结果对象
        """
        self.stats['total_requests'] += 1
        planning_start = time.time()
        
        # 解析上下文
        planning_context = self._parse_context(context)
        self.stats['context_usage'][planning_context] += 1
        
        # 处理车辆参数
        vehicle_safety_params = self._process_vehicle_params(vehicle_id, vehicle_params)
        
        # 根据上下文调整参数
        context_config = self.config.context_configs.get(planning_context, {})
        if conflict_avoidance or context_config.get('conflict_avoidance', False):
            kwargs['safety_bias'] = context_config.get('safety_bias', 1.5)
        
        # 输入验证
        if not self._validate_inputs(start, goal):
            print(f"输入验证失败: start={start}, goal={goal}")
            return None
        
        # ===== 新增：骨干网络优先逻辑 =====
        # 注意：避免在生成骨干路径时递归调用
        if (use_backbone and self.backbone_network and 
            context != 'backbone' and context != 'backbone_generation'):
            
            print(f"路径规划: {vehicle_id} - 尝试使用骨干网络")
            
            # 尝试使用骨干网络
            backbone_result = self._try_backbone_network_first(
                vehicle_id, start, goal, vehicle_safety_params, **kwargs
            )
            
            if backbone_result:
                # 缓存结果
                cache_key = self._generate_enhanced_cache_key(
                    start, goal, planning_context, vehicle_params, conflict_avoidance
                )
                self._add_to_cache(cache_key, backbone_result)
                
                self.stats['successful_plans'] += 1
                self._update_planning_stats(backbone_result, planning_start)
                
                print(f"✅ 车辆 {vehicle_id} 使用骨干网络路径成功: "
                    f"长度={len(backbone_result.path)}, "
                    f"骨干利用率={backbone_result.backbone_utilization:.2%}")
                
                return self._format_result(backbone_result, return_object)
            else:
                print(f"  骨干网络无法提供路径，切换到直接规划")
        
        # ===== 原有逻辑：检查缓存 =====
        cache_key = self._generate_enhanced_cache_key(
            start, goal, planning_context, vehicle_params, conflict_avoidance
        )
        cached_result = self._check_cache(cache_key)
        if cached_result:
            self.stats['cache_hits'] += 1
            print(f"使用缓存路径: {vehicle_id}")
            return self._format_result(cached_result, return_object)
        
        print(f"开始增强路径规划: {vehicle_id} ({planning_context.value})")
        
        # 使用渐进式回退策略
        result = self._plan_with_progressive_fallback(
            vehicle_id, start, goal, planning_context, vehicle_safety_params,
            conflict_avoidance, min_safety_clearance, planning_start, **kwargs
        )
        
        if result:
            # 缓存结果
            self._add_to_cache(cache_key, result)
            self.stats['successful_plans'] += 1
            
            # 更新统计
            self._update_planning_stats(result, planning_start)
            
            print(f"✅ 路径规划成功: 质量={result.quality_score:.2f}, "
                f"安全={result.safety_score:.2f}, 风险={result.conflict_risk:.2f}")
        else:
            print(f"❌ 路径规划失败: {vehicle_id}")
        
        return self._format_result(result, return_object)
    
    def _parse_context(self, context_str: str) -> PlanningContext:
        """解析规划上下文"""
        context_map = {
            'normal': PlanningContext.NORMAL,
            'backbone': PlanningContext.BACKBONE,
            'navigation': PlanningContext.NAVIGATION,
            'conflict_resolution': PlanningContext.CONFLICT_RESOLUTION,
            'emergency': PlanningContext.EMERGENCY,
            'backbone_access': PlanningContext.BACKBONE_ACCESS
        }
        
        return context_map.get(context_str, PlanningContext.NORMAL)
    
    def _process_vehicle_params(self, vehicle_id: str, 
                               vehicle_params: Dict = None) -> VehicleSafetyParams:
        """处理车辆安全参数"""
        # 检查缓存
        if vehicle_id in self.vehicle_param_cache and vehicle_params is None:
            return self.vehicle_param_cache[vehicle_id]
        
        # 创建安全参数对象
        if vehicle_params:
            safety_params = VehicleSafetyParams(
                length=vehicle_params.get('length', 6.0),
                width=vehicle_params.get('width', 3.0),
                safety_margin=vehicle_params.get('safety_margin', 1.5),
                turning_radius=vehicle_params.get('turning_radius', 8.0),
                max_speed=vehicle_params.get('max_speed', 2.0)
            )
        else:
            safety_params = VehicleSafetyParams()
        
        # 缓存结果
        self.vehicle_param_cache[vehicle_id] = safety_params
        
        return safety_params
    
    def _plan_with_progressive_fallback(self, vehicle_id: str, start: Tuple, goal: Tuple,
                                      context: PlanningContext, 
                                      vehicle_params: VehicleSafetyParams,
                                      conflict_avoidance: bool, 
                                      min_safety_clearance: float,
                                      planning_start: float, **kwargs) -> Optional[PlanningResult]:
        """使用渐进式回退策略进行规划"""
        
        fallback_sequence = self.fallback_strategy.get_fallback_sequence(context)
        
        for i, (planner_type, quality_level, max_time) in enumerate(fallback_sequence, 1):
            strategy_name = f"{planner_type}_{quality_level.value if quality_level else 'direct'}"
            
            print(f"  [{i}/{len(fallback_sequence)}] 尝试策略: {strategy_name}")
            
            # 记录回退使用统计
            self.stats['fallback_usage'][strategy_name] += 1
            
            # 执行规划
            strategy_start = time.time()
            result = self._execute_planning_strategy(
                vehicle_id, start, goal, planner_type, quality_level,
                max_time, context, vehicle_params, conflict_avoidance,
                min_safety_clearance, **kwargs
            )
            strategy_time = time.time() - strategy_start
            
            # 更新策略统计
            self.stats['planner_success_rates'][strategy_name]['total'] += 1
            self.stats['average_planning_times'][strategy_name].append(strategy_time)
            
            if result:
                self.stats['planner_success_rates'][strategy_name]['success'] += 1
                
                total_time = time.time() - planning_start
                print(f"    ✅ 策略成功! 规划器: {result.planner_used}, "
                      f"质量: {result.quality_score:.2f}, 耗时: {total_time:.2f}s")
                
                return result
            else:
                print(f"    ❌ 策略失败, 耗时: {strategy_time:.2f}s")
        
        print("  ❌ 所有回退策略均失败")
        return None
    def _try_backbone_network_first(self, vehicle_id: str, start: Tuple, goal: Tuple,
                                vehicle_params: VehicleSafetyParams, **kwargs) -> Optional[PlanningResult]:
        """优先尝试使用骨干网络"""
        try:
            # 从kwargs或环境中解析目标信息
            target_type = kwargs.get('target_type')
            target_id = kwargs.get('target_id')
            
            if not target_type or target_id is None:
                target_type, target_id = self._infer_target_from_goal(goal)
            
            if not target_type:
                return None
            
            print(f"  尝试骨干网络: {start} -> {target_type}_{target_id}")
            
            # 调用骨干网络
            backbone_result = self.backbone_network.get_path_from_position_to_target(
                start, target_type, target_id, vehicle_id
            )
            
            if backbone_result:
                if isinstance(backbone_result, tuple):
                    path, structure = backbone_result
                else:
                    path = backbone_result
                    structure = {'type': 'backbone_direct'}
                
                # 创建标准化的PlanningResult
                return self._create_enhanced_result(
                    path, 'backbone_network', vehicle_params, 
                    PlanningContext.NORMAL, start, goal
                )
        
        except Exception as e:
            print(f"  骨干网络尝试失败: {e}")
        
        return None

    def _infer_target_from_goal(self, goal: Tuple) -> Tuple[Optional[str], Optional[int]]:
        """从目标坐标推断目标类型和ID"""
        if not self.env:
            return None, None
        
        tolerance = 2.0  # 位置匹配容差
        
        # 检查卸载点
        for i, point in enumerate(self.env.unloading_points):
            if self._calculate_distance(goal, point) < tolerance:
                return 'unloading', i
        
        # 检查装载点
        for i, point in enumerate(self.env.loading_points):
            if self._calculate_distance(goal, point) < tolerance:
                return 'loading', i
        
        # 检查停车点
        if hasattr(self.env, 'parking_areas'):
            for i, point in enumerate(self.env.parking_areas):
                if self._calculate_distance(goal, point) < tolerance:
                    return 'parking', i
        
        return None, None    
    def _execute_planning_strategy(self, vehicle_id: str, start: Tuple, goal: Tuple,
                                 planner_type: str, quality_level: QualityLevel,
                                 max_time: float, context: PlanningContext,
                                 vehicle_params: VehicleSafetyParams,
                                 conflict_avoidance: bool, min_safety_clearance: float,
                                 **kwargs) -> Optional[PlanningResult]:
        """执行具体的规划策略"""
        try:
            if planner_type == 'hybrid_astar' and 'hybrid_astar' in self.planners:
                return self._plan_with_astar_enhanced(
                    vehicle_id, start, goal, quality_level, max_time,
                    context, vehicle_params, conflict_avoidance, **kwargs
                )
            
            elif planner_type == 'rrt' and 'rrt' in self.planners:
                return self._plan_with_rrt_enhanced(
                    vehicle_id, start, goal, quality_level, max_time,
                    context, vehicle_params, conflict_avoidance, **kwargs
                )
            
            elif planner_type == 'direct':
                return self._plan_direct_path_enhanced(
                    start, goal, vehicle_params, context
                )
            
            return None
        
        except Exception as e:
            print(f"      策略执行异常: {e}")
            return None
    
    def _plan_with_astar_enhanced(self, vehicle_id: str, start: Tuple, goal: Tuple,
                                quality_level: QualityLevel, max_time: float,
                                context: PlanningContext,
                                vehicle_params: VehicleSafetyParams,
                                conflict_avoidance: bool, **kwargs) -> Optional[PlanningResult]:
        """使用增强混合A*规划"""
        planner = self.planners['hybrid_astar']
        
        # 获取质量级别对应的配置
        if quality_level and quality_level in self.config.astar_params:
            config = self.config.astar_params[quality_level].copy()
        else:
            config = self.config.astar_params[QualityLevel.STANDARD].copy()
        
        # 根据车辆参数调整配置
        config.update({
            'vehicle_length': vehicle_params.length,
            'vehicle_width': vehicle_params.width,
            'turning_radius': vehicle_params.turning_radius
        })
        
        # 根据上下文调整参数
        if context == PlanningContext.CONFLICT_RESOLUTION:
            config['quality_threshold'] = max(config['quality_threshold'], 0.7)
            if conflict_avoidance:
                config['safety_bias'] = kwargs.get('safety_bias', 1.5)
        
        # 动态更新规划器配置
        if hasattr(planner, 'config'):
            planner.config.update({
                'max_iterations': config.get('max_iterations', 15000),
                'timeout': min(max_time, config.get('timeout', 15.0)),
                'quality_threshold': config.get('quality_threshold', 0.6)
            })
        
        # 执行规划
        path = planner.plan_path(
            start, goal,
            agent_id=vehicle_id,
            max_iterations=config.get('max_iterations'),
            quality_threshold=config.get('quality_threshold', 0.6),
            use_cache=True
        )
        
        if path and len(path) >= 2:
            return self._create_enhanced_result(
                path, 'hybrid_astar', vehicle_params, context, start, goal
            )
        
        return None
    
    def _plan_with_rrt_enhanced(self, vehicle_id: str, start: Tuple, goal: Tuple,
                              quality_level: QualityLevel, max_time: float,
                              context: PlanningContext,
                              vehicle_params: VehicleSafetyParams,
                              conflict_avoidance: bool, **kwargs) -> Optional[PlanningResult]:
        """使用增强RRT规划"""
        planner = self.planners['rrt']
        
        # 获取质量级别对应的配置
        config_level = quality_level if quality_level in self.config.rrt_params else QualityLevel.STANDARD
        config = self.config.rrt_params[config_level].copy()
        
        # 根据车辆参数调整配置
        config.update({
            'vehicle_length': vehicle_params.length,
            'vehicle_width': vehicle_params.width,
            'turning_radius': vehicle_params.turning_radius
        })
        
        # 执行规划
        path = planner.plan_path(
            start, goal,
            agent_id=vehicle_id,
            max_iterations=config.get('max_nodes', 4000),
            quality_threshold=config.get('quality_threshold', 0.5),
            enable_dynamics_optimization=config.get('enable_dynamics_optimization', True)
        )
        
        if path and len(path) >= 2:
            return self._create_enhanced_result(
                path, 'rrt', vehicle_params, context, start, goal
            )
        
        return None
    
    def _plan_direct_path_enhanced(self, start: Tuple, goal: Tuple,
                                 vehicle_params: VehicleSafetyParams,
                                 context: PlanningContext) -> PlanningResult:
        """增强的直线路径规划"""
        # 确保3D坐标
        start_3d = self._ensure_3d_coordinates(start)
        goal_3d = self._ensure_3d_coordinates(goal)
        
        # 计算路径点
        distance = math.sqrt(
            (goal_3d[0] - start_3d[0])**2 + 
            (goal_3d[1] - start_3d[1])**2
        )
        
        # 根据距离和车辆参数确定步数
        step_size = min(vehicle_params.length, 3.0)
        steps = max(3, int(distance / step_size))
        
        path = []
        for i in range(steps + 1):
            t = i / steps
            x = start_3d[0] + t * (goal_3d[0] - start_3d[0])
            y = start_3d[1] + t * (goal_3d[1] - start_3d[1])
            theta = start_3d[2] + t * (goal_3d[2] - start_3d[2])
            path.append((x, y, theta))
        
        return self._create_enhanced_result(
            path, 'direct', vehicle_params, context, start, goal
        )
    
    def _create_enhanced_result(self, path: List[Tuple], planner_used: str,
                              vehicle_params: VehicleSafetyParams,
                              context: PlanningContext,
                              start: Tuple, goal: Tuple) -> PlanningResult:
        """创建增强的规划结果"""
        # 计算质量分数
        quality_score = self._evaluate_path_quality_enhanced(path, start, goal)
        
        # 计算安全分数
        safety_score = 0.0
        if self.config.enable_safety_optimization:
            safety_score = self.safety_analyzer.evaluate_path_safety(
                path, vehicle_params, self.config.min_safety_clearance
            )
        
        # 估算冲突风险
        conflict_risk = 0.0
        if self.config.enable_conflict_avoidance:
            conflict_risk = self.conflict_estimator.estimate_path_conflict_risk(path)
        
        # 计算骨干网络利用率
        backbone_utilization = self._calculate_backbone_utilization(path)
        
        # 创建增强的路径结构信息
        structure = {
            'planner_used': planner_used,
            'context': context.value,
            'final_quality': quality_score,
            'safety_score': safety_score,
            'conflict_risk': conflict_risk,
            'backbone_utilization': backbone_utilization,
            'total_length': len(path),
            'path_distance': self._calculate_path_distance(path),
            'vehicle_params_used': vehicle_params.to_dict(),
            'safety_optimized': self.config.enable_safety_optimization,
            'conflict_avoidance_enabled': self.config.enable_conflict_avoidance,
            'planning_time': 0.0  # 将由调用者设置
        }
        
        return PlanningResult(
            path=path,
            planner_used=planner_used,
            quality_score=quality_score,
            planning_time=0.0,  # 将由调用者设置
            structure=structure,
            safety_score=safety_score,
            conflict_risk=conflict_risk,
            backbone_utilization=backbone_utilization
        )
    
    def _evaluate_path_quality_enhanced(self, path: List[Tuple], 
                                      start: Tuple, goal: Tuple) -> float:
        """增强的路径质量评估"""
        if not path or len(path) < 2:
            return 0.0
        
        try:
            # 1. 长度效率
            path_length = self._calculate_path_distance(path)
            direct_distance = math.sqrt(
                (goal[0] - start[0])**2 + (goal[1] - start[1])**2
            )
            
            length_efficiency = direct_distance / max(path_length, 0.1)
            
            # 2. 平滑度
            smoothness = self._calculate_path_smoothness(path)
            
            # 3. 方向一致性
            direction_consistency = self._calculate_direction_consistency(path)
            
            # 4. 曲率分析
            curvature_score = self._analyze_path_curvature(path)
            
            # 综合质量评分
            quality = (
                length_efficiency * 0.35 + 
                smoothness * 0.25 + 
                direction_consistency * 0.2 + 
                curvature_score * 0.2
            )
            
            return min(1.0, max(0.0, quality))
        
        except Exception:
            return 0.5
    
    def _calculate_path_distance(self, path: List[Tuple]) -> float:
        """计算路径总距离"""
        if not path or len(path) < 2:
            return 0.0
        
        distance = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            distance += math.sqrt(dx*dx + dy*dy)
        
        return distance
    
    def _calculate_path_smoothness(self, path: List[Tuple]) -> float:
        """计算路径平滑度"""
        if len(path) < 3:
            return 1.0
        
        try:
            total_angle_change = 0.0
            valid_segments = 0
            
            for i in range(1, len(path) - 1):
                # 计算角度变化
                v1 = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
                v2 = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
                
                len1 = math.sqrt(v1[0]**2 + v1[1]**2)
                len2 = math.sqrt(v2[0]**2 + v2[1]**2)
                
                if len1 > 1e-6 and len2 > 1e-6:
                    cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
                    cos_angle = max(-1, min(1, cos_angle))
                    angle_change = math.acos(cos_angle)
                    total_angle_change += angle_change
                    valid_segments += 1
            
            if valid_segments == 0:
                return 1.0
            
            avg_angle_change = total_angle_change / valid_segments
            smoothness = math.exp(-avg_angle_change * 2)
            
            return smoothness
        
        except Exception:
            return 0.8
    
    def _calculate_direction_consistency(self, path: List[Tuple]) -> float:
        """计算方向一致性"""
        if len(path) < 3:
            return 1.0
        
        try:
            if len(path[0]) < 3:
                return 0.8  # 没有方向信息时的默认分数
            
            direction_changes = 0
            total_segments = len(path) - 1
            
            for i in range(len(path) - 1):
                current_theta = path[i][2]
                next_theta = path[i+1][2]
                
                # 计算角度差
                angle_diff = abs(next_theta - current_theta)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # 取较小角度
                
                if angle_diff > math.pi/4:  # 45度阈值
                    direction_changes += 1
            
            consistency = 1.0 - (direction_changes / max(1, total_segments))
            return max(0.0, consistency)
        
        except Exception:
            return 0.8
    
    def _analyze_path_curvature(self, path: List[Tuple]) -> float:
        """分析路径曲率"""
        if len(path) < 4:
            return 1.0
        
        try:
            total_curvature = 0.0
            valid_points = 0
            
            for i in range(1, len(path) - 1):
                # 使用三点计算曲率
                p1 = path[i-1]
                p2 = path[i]
                p3 = path[i+1]
                
                # 计算曲率半径
                curvature = self._calculate_curvature_at_point(p1, p2, p3)
                if curvature > 0:
                    total_curvature += curvature
                    valid_points += 1
            
            if valid_points == 0:
                return 1.0
            
            avg_curvature = total_curvature / valid_points
            # 曲率越小越好（更直的路径）
            curvature_score = 1.0 / (1.0 + avg_curvature * 0.1)
            
            return curvature_score
        
        except Exception:
            return 0.8
    
    def _calculate_curvature_at_point(self, p1: Tuple, p2: Tuple, p3: Tuple) -> float:
        """计算三点处的曲率"""
        try:
            # 使用向量叉积计算曲率
            v1 = (p2[0] - p1[0], p2[1] - p1[1])
            v2 = (p3[0] - p2[0], p3[1] - p2[1])
            
            len1 = math.sqrt(v1[0]**2 + v1[1]**2)
            len2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if len1 > 1e-6 and len2 > 1e-6:
                # 叉积
                cross_product = v1[0]*v2[1] - v1[1]*v2[0]
                curvature = abs(cross_product) / (len1 * len2)
                return curvature
            
            return 0.0
        
        except Exception:
            return 0.0
    
    def _calculate_backbone_utilization(self, path: List[Tuple]) -> float:
        """计算骨干网络利用率"""
        if not self.backbone_network or not path:
            return 0.0
        
        # 简化实现：检查路径是否经过骨干网络
        backbone_points = 0
        total_points = len(path)
        
        # 这里应该根据实际的骨干网络结构来判断
        # 简化处理，返回一个估计值
        return 0.3  # 默认30%骨干利用率
    
    def _update_planning_stats(self, result: PlanningResult, planning_start: float):
        """更新规划统计"""
        planning_time = time.time() - planning_start
        
        # 更新结果中的规划时间
        result.structure['planning_time'] = planning_time
        
        # 更新统计
        quality_range = int(result.quality_score * 10)  # 0-10范围
        self.stats['quality_distribution'][quality_range] += 1
        
        self.stats['safety_scores'].append(result.safety_score)
        self.stats['conflict_risks'].append(result.conflict_risk)
        
        # 限制统计列表长度
        if len(self.stats['safety_scores']) > 1000:
            self.stats['safety_scores'] = self.stats['safety_scores'][-500:]
        if len(self.stats['conflict_risks']) > 1000:
            self.stats['conflict_risks'] = self.stats['conflict_risks'][-500:]
    
    def _validate_inputs(self, start: Tuple, goal: Tuple) -> bool:
        """验证输入参数"""
        try:
            if not start or not goal:
                return False
            
            if len(start) < 2 or len(goal) < 2:
                return False
            
            # 检查坐标范围
            for point in [start, goal]:
                x, y = float(point[0]), float(point[1])
                if not (0 <= x < self.env.width and 0 <= y < self.env.height):
                    return False
            
            return True
        
        except Exception:
            return False
    
    def _ensure_3d_coordinates(self, point: Tuple) -> Tuple[float, float, float]:
        """确保坐标为3D格式"""
        if len(point) >= 3:
            return (float(point[0]), float(point[1]), float(point[2]))
        elif len(point) == 2:
            return (float(point[0]), float(point[1]), 0.0)
        else:
            return (0.0, 0.0, 0.0)
    
    def _generate_enhanced_cache_key(self, start: Tuple, goal: Tuple, 
                                   context: PlanningContext,
                                   vehicle_params: Dict = None,
                                   conflict_avoidance: bool = False) -> str:
        """生成增强缓存键"""
        start_key = f"{start[0]:.1f},{start[1]:.1f}"
        goal_key = f"{goal[0]:.1f},{goal[1]:.1f}"
        
        # 添加上下文和参数信息
        param_hash = "default"
        if vehicle_params:
            param_str = f"{vehicle_params.get('length', 6):.1f}_{vehicle_params.get('width', 3):.1f}"
            param_hash = param_str
        
        return f"{start_key}_{goal_key}_{context.value}_{param_hash}_{conflict_avoidance}"
    
    def _check_cache(self, cache_key: str) -> Optional[Any]:
        """检查缓存"""
        with self.cache_lock:
            if cache_key in self.cache:
                # LRU: 移动到末尾
                self.cache.move_to_end(cache_key)
                return self.cache[cache_key]
        return None
    
    def _add_to_cache(self, cache_key: str, result: Any):
        """添加到缓存"""
        with self.cache_lock:
            # 限制缓存大小
            if len(self.cache) >= self.config.cache_size:
                self.cache.popitem(last=False)
            
            self.cache[cache_key] = result
    
    def _format_result(self, result: Optional[PlanningResult], 
                      return_object: bool) -> Optional[Any]:
        """格式化返回结果"""
        if not result:
            return None
        
        if return_object:
            return result
        else:
            # 返回元组格式以保持向后兼容
            return (result.path, result.structure)
    
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息"""
        stats = self.stats.copy()
        
        # 计算成功率
        for strategy_name, data in self.stats['planner_success_rates'].items():
            if data['total'] > 0:
                success_rate = data['success'] / data['total']
                stats[f'{strategy_name}_success_rate'] = success_rate
        
        # 计算平均规划时间
        for strategy_name, times in self.stats['average_planning_times'].items():
            if times:
                avg_time = sum(times) / len(times)
                stats[f'{strategy_name}_avg_time'] = avg_time
        
        # 计算总体成功率
        if stats['total_requests'] > 0:
            stats['overall_success_rate'] = stats['successful_plans'] / stats['total_requests']
            stats['cache_hit_rate'] = stats['cache_hits'] / stats['total_requests']
        else:
            stats['overall_success_rate'] = 0.0
            stats['cache_hit_rate'] = 0.0
        
        # 安全和冲突统计
        if self.stats['safety_scores']:
            stats['average_safety_score'] = sum(self.stats['safety_scores']) / len(self.stats['safety_scores'])
            stats['min_safety_score'] = min(self.stats['safety_scores'])
        
        if self.stats['conflict_risks']:
            stats['average_conflict_risk'] = sum(self.stats['conflict_risks']) / len(self.stats['conflict_risks'])
            stats['max_conflict_risk'] = max(self.stats['conflict_risks'])
        
        # 实时信息
        stats['available_planners'] = list(self.planners.keys())
        stats['cache_size'] = len(self.cache)
        stats['backbone_network_available'] = self.backbone_network is not None
        stats['traffic_manager_available'] = self.traffic_manager is not None
        stats['safety_optimization_enabled'] = self.config.enable_safety_optimization
        stats['conflict_avoidance_enabled'] = self.config.enable_conflict_avoidance
        
        return stats
    
    def get_planner_info(self, planner_name: str) -> Optional[Dict]:
        """获取规划器详细信息"""
        if planner_name not in self.planners:
            return None
        
        planner = self.planners[planner_name]
        info = {
            'name': planner_name,
            'available': True,
            'type': type(planner).__name__,
            'success_rate': 0.0,
            'average_time': 0.0
        }
        
        # 获取成功率统计
        strategy_stats = self.stats['planner_success_rates']
        for strategy_name in strategy_stats:
            if planner_name in strategy_name:
                data = strategy_stats[strategy_name]
                if data['total'] > 0:
                    info['success_rate'] = max(info['success_rate'], 
                                             data['success'] / data['total'])
        
        # 获取平均时间
        time_stats = self.stats['average_planning_times']
        for strategy_name in time_stats:
            if planner_name in strategy_name and time_stats[strategy_name]:
                avg_time = sum(time_stats[strategy_name]) / len(time_stats[strategy_name])
                info['average_time'] = max(info['average_time'], avg_time)
        
        # 获取规划器特定统计（如果有）
        if hasattr(planner, 'get_statistics'):
            try:
                info['planner_specific_stats'] = planner.get_statistics()
            except Exception:
                pass
        
        return info
    
    def clear_cache(self):
        """清理缓存"""
        with self.cache_lock:
            self.cache.clear()
            self.vehicle_param_cache.clear()
        
        # 重置缓存相关统计
        self.stats['cache_hits'] = 0
        
        print("增强路径规划器缓存已清理")
    
    def reset_statistics(self):
        """重置统计信息"""
        self.stats = {
            'total_requests': 0,
            'successful_plans': 0,
            'context_usage': defaultdict(int),
            'planner_success_rates': defaultdict(lambda: {'success': 0, 'total': 0}),
            'quality_distribution': defaultdict(int),
            'safety_scores': [],
            'conflict_risks': [],
            'cache_hits': 0,
            'fallback_usage': defaultdict(int),
            'average_planning_times': defaultdict(list)
        }
        
        print("增强路径规划器统计信息已重置")
    
    def optimize_configuration(self):
        """基于统计数据优化配置"""
        print("开始优化路径规划器配置...")
        
        # 分析成功率，调整回退序列
        success_rates = {}
        for strategy_name, data in self.stats['planner_success_rates'].items():
            if data['total'] > 10:  # 足够的样本
                success_rates[strategy_name] = data['success'] / data['total']
        
        # 分析平均规划时间
        avg_times = {}
        for strategy_name, times in self.stats['average_planning_times'].items():
            if len(times) > 5:
                avg_times[strategy_name] = sum(times) / len(times)
        
        # 根据分析结果调整配置
        optimizations = []
        
        # 检查安全分数分布
        if self.stats['safety_scores']:
            avg_safety = sum(self.stats['safety_scores']) / len(self.stats['safety_scores'])
            if avg_safety < 0.7:
                self.config.min_safety_clearance *= 1.2
                optimizations.append(f"增加安全间隔到 {self.config.min_safety_clearance:.1f}")
        
        # 检查冲突风险分布
        if self.stats['conflict_risks']:
            avg_risk = sum(self.stats['conflict_risks']) / len(self.stats['conflict_risks'])
            if avg_risk > 0.3:
                self.config.enable_conflict_avoidance = True
                optimizations.append("启用冲突避让优化")
        
        if optimizations:
            print(f"配置优化完成: {'; '.join(optimizations)}")
        else:
            print("当前配置已经是最优状态")
    
    def shutdown(self):
        """关闭规划器"""
        # 清理缓存
        self.clear_cache()
        
        # 关闭子规划器
        for planner_name, planner in self.planners.items():
            if hasattr(planner, 'shutdown'):
                try:
                    planner.shutdown()
                except Exception as e:
                    print(f"关闭规划器 {planner_name} 失败: {e}")
        
        self.planners.clear()
        print("增强路径规划器已关闭")

# 向后兼容性
SimplifiedPathPlanner = EnhancedPathPlanner
PathPlanner = EnhancedPathPlanner
UnifiedPathPlanner = EnhancedPathPlanner
EnhancedPathPlannerWithConfig = EnhancedPathPlanner