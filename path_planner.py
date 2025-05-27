import math
import numpy as np
import time
from collections import defaultdict, OrderedDict, deque
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from RRT import OptimizedRRTPlanner
from astar import HybridAStarPlanner  # 导入新的混合A*规划器

@dataclass
class PlanningRequest:
    """路径规划请求"""
    vehicle_id: str
    start: Tuple[float, float, float]
    goal: Tuple[float, float, float]
    priority: int = 1
    deadline: float = 0.0
    quality_requirement: float = 0.6
    use_cache: bool = True
    strategy_hints: Dict = None
    planner_type: str = "hybrid_astar"  # 新增：规划器类型选择

@dataclass
class PlanningResult:
    """路径规划结果"""
    path: List[Tuple[float, float, float]]
    structure: Dict
    quality_score: float
    planning_time: float
    cache_hit: bool
    planner_used: str = "unknown"  # 新增：使用的规划器类型
    rrt_stats: Dict = None
    astar_stats: Dict = None  # 新增：混合A*统计

class EnhancedPathPlanner:
    """
    增强版路径规划器 - 集成混合A*和RRT
    
    设计理念：
    1. 支持多种规划器：混合A*、RRT、直接规划
    2. 智能选择最适合的规划器
    3. 保持骨干网络优先策略
    4. 统一的接口和缓存系统
    """
    
    def __init__(self, env, backbone_network=None, traffic_manager=None, 
                 primary_planner="hybrid_astar"):
        # 初始化代码
        self.env = env
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        self.primary_planner = primary_planner  # 主要规划器类型
        
        # 规划器实例
        self.planners = {}
        self._initialize_planners()
        
        # 智能缓存系统
        self.cache_config = {
            'max_size': 500,
            'ttl': 300,  # 5分钟过期
            'quality_threshold': 0.6
        }
        self.route_cache = OrderedDict()  # LRU缓存
        self.cache_metadata = {}
        
        # 路径质量评估器
        self.quality_assessor = PathQualityAssessor(env)
        
        # 性能统计
        self.performance_stats = {
            'total_requests': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'backbone_usage': 0,
            'direct_planning': 0,
            'planning_times': [],
            'quality_scores': [],
            'planner_usage': defaultdict(int),  # 各规划器使用次数
            'planner_success_rate': defaultdict(float),  # 各规划器成功率
            'backbone_success_rate': 0.0
        }
        
        # 规划配置
        self.planning_config = {
            'max_attempts': 3,
            'enable_post_smoothing': True,
            'enable_shortcut_optimization': True,
            'enable_quality_check': True,
            'auto_fallback': True,  # 自动回退到其他规划器
            'quality_threshold_adaptive': True  # 自适应质量阈值
        }
        
        # 调试选项
        self.debug = False
        self.verbose_logging = False
        
        print(f"初始化增强版路径规划器 - 主规划器: {primary_planner}")
    
    def _initialize_planners(self):
        """初始化所有可用的规划器"""
        # 1. 混合A*规划器
        try:
            self.planners['hybrid_astar'] = HybridAStarPlanner(
                self.env,
                vehicle_length=6.0,
                vehicle_width=3.0,
                turning_radius=8.0,
                step_size=2.0,
                angle_resolution=30
            )
            print("✓ 混合A*规划器初始化成功")
        except Exception as e:
            print(f"⚠️ 混合A*规划器初始化失败: {e}")
        
        # 2. RRT规划器
        try:
            self.planners['rrt'] = OptimizedRRTPlanner(
                self.env,
                vehicle_length=6.0,
                vehicle_width=3.0,
                turning_radius=8.0,
                step_size=0.6
            )
            print("✓ RRT规划器初始化成功")
        except Exception as e:
            print(f"⚠️ RRT规划器初始化失败: {e}")
            try:
                from RRT import RRTPlanner
                self.planners['rrt'] = RRTPlanner(
                    self.env,
                    vehicle_length=6.0,
                    vehicle_width=3.0,
                    turning_radius=8.0,
                    step_size=0.6
                )
                print("✓ 回退RRT规划器初始化成功")
            except Exception as e2:
                print(f"⚠️ 所有RRT规划器初始化失败: {e2}")
        
        # 确保至少有一个可用的规划器
        if not self.planners:
            raise RuntimeError("没有可用的路径规划器")
        
        # 设置默认主规划器
        if self.primary_planner not in self.planners:
            self.primary_planner = list(self.planners.keys())[0]
            print(f"主规划器回退到: {self.primary_planner}")
    
    def set_backbone_network(self, backbone_network):
        """设置骨干路径网络"""
        self.backbone_network = backbone_network
        
        # 为所有规划器设置骨干网络
        for planner_name, planner in self.planners.items():
            if hasattr(planner, 'set_backbone_network'):
                planner.set_backbone_network(backbone_network)
        
        # 清空缓存
        self._clear_cache()
        print("已为所有规划器设置骨干路径网络")
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.traffic_manager = traffic_manager
    
    def plan_path(self, vehicle_id, start, goal, use_backbone=True, check_conflicts=True, 
                  planner_type=None, max_attempts=None):
        """
        主要路径规划接口 - 增强版
        
        Args:
            vehicle_id: 车辆ID
            start: 起点坐标 (x, y, theta)
            goal: 终点坐标 (x, y, theta)
            use_backbone: 是否使用骨干网络
            check_conflicts: 是否检查冲突
            planner_type: 指定规划器类型
            max_attempts: 最大尝试次数
            
        Returns:
            tuple: (路径点列表, 路径结构信息) 或 (None, None)
        """
        start_time = time.time()
        self.performance_stats['total_requests'] += 1
        
        # 输入验证
        if not self._validate_inputs(start, goal):
            return None, None
        
        # 检查起点和终点是否相同
        if self._is_same_position(start, goal):
            return [start], {'type': 'direct', 'segments': 1}
        
        # 生成缓存键
        cache_key = self._generate_cache_key(vehicle_id, start, goal)
        
        # 检查缓存
        cached_result = self._check_cache(cache_key)
        if cached_result:
            self.performance_stats['cache_hits'] += 1
            return cached_result
        
        self.performance_stats['cache_misses'] += 1
        
        # 选择规划器
        selected_planner = planner_type or self.primary_planner
        if selected_planner not in self.planners:
            selected_planner = self.primary_planner
        
        # 多次尝试规划
        max_attempts = max_attempts or self.planning_config['max_attempts']
        best_path = None
        best_structure = None
        best_quality = 0
        planner_used = None
        
        # 规划器尝试顺序
        planner_sequence = self._get_planner_sequence(selected_planner)
        
        for attempt in range(max_attempts):
            for planner_name in planner_sequence:
                if planner_name not in self.planners:
                    continue
                
                try:
                    # 核心规划逻辑
                    path, structure = self._plan_path_with_planner(
                        planner_name, vehicle_id, start, goal, attempt, use_backbone
                    )
                    
                    if path:
                        # 验证路径
                        if self._validate_path(path):
                            # 评估质量
                            quality = self.quality_assessor.evaluate_path(path)
                            
                            if quality > best_quality:
                                best_path = path
                                best_structure = structure
                                best_quality = quality
                                planner_used = planner_name
                                
                                # 记录规划器使用
                                self.performance_stats['planner_usage'][planner_name] += 1
                                
                                # 质量足够高，提前结束
                                if quality >= 0.9:
                                    break
                
                except Exception as e:
                    if self.debug:
                        print(f"规划器 {planner_name} 尝试 {attempt + 1} 失败: {e}")
                    continue
            
            if best_path and best_quality >= 0.7:
                break
        
        # 后处理优化
        if best_path:
            best_path = self._post_process_path(best_path)
            if best_structure:
                best_structure['final_quality'] = self.quality_assessor.evaluate_path(best_path)
                best_structure['planner_used'] = planner_used
        
        # 冲突检查
        if best_path and check_conflicts and self.traffic_manager:
            if self.traffic_manager.check_path_conflicts(vehicle_id, best_path):
                # 尝试调整路径
                adjusted_path = self._resolve_path_conflicts(vehicle_id, start, goal, best_path)
                if adjusted_path:
                    best_path = adjusted_path
                    best_structure = self._analyze_path_structure(best_path)
                    best_structure['planner_used'] = planner_used
        
        # 缓存高质量结果
        if best_path and best_quality >= self.cache_config['quality_threshold']:
            self._add_to_cache(cache_key, (best_path, best_structure), best_quality)
        
        # 更新统计信息
        planning_time = time.time() - start_time
        self.performance_stats['planning_times'].append(planning_time)
        if best_quality > 0:
            self.performance_stats['quality_scores'].append(best_quality)
        
        if self.verbose_logging:
            print(f"路径规划完成: 车辆{vehicle_id}, 规划器{planner_used}, "
                  f"质量={best_quality:.2f}, 耗时={planning_time:.3f}s")
        
        return best_path, best_structure
    
    def _get_planner_sequence(self, primary_planner: str) -> List[str]:
        """获取规划器尝试序列"""
        available_planners = list(self.planners.keys())
        
        # 主规划器优先
        sequence = [primary_planner] if primary_planner in available_planners else []
        
        # 添加其他规划器作为备选
        for planner in available_planners:
            if planner not in sequence:
                sequence.append(planner)
        
        return sequence
    
    def _plan_path_with_planner(self, planner_name: str, vehicle_id: str, 
                               start: Tuple, goal: Tuple, attempt: int, 
                               use_backbone: bool) -> Tuple:
        """使用指定规划器进行路径规划"""
        if self.debug:
            print(f"使用 {planner_name} 规划路径: {start} -> {goal} (尝试 {attempt + 1})")
        
        # 1. 尝试使用骨干网络
        if use_backbone and self.backbone_network:
            backbone_result = self._try_backbone_planning(start, goal, attempt)
            if backbone_result:
                path, structure = backbone_result
                self.performance_stats['backbone_usage'] += 1
                structure['planner_used'] = f"{planner_name}_backbone"
                if self.debug:
                    print(f"骨干网络规划成功，路径长度: {len(path)}")
                return path, structure
        
        # 2. 直接规划
        if self.debug:
            print(f"使用 {planner_name} 直接规划")
        
        direct_result = self._direct_planning_with_planner(planner_name, start, goal, attempt)
        if direct_result:
            self.performance_stats['direct_planning'] += 1
            return direct_result, {
                'type': 'direct', 
                'method': planner_name,
                'planner_used': planner_name
            }
        
        return None, None
    
    def _direct_planning_with_planner(self, planner_name: str, start: Tuple, 
                                     goal: Tuple, attempt: int):
        """使用指定规划器进行直接规划"""
        if planner_name not in self.planners:
            return None
        
        planner = self.planners[planner_name]
        
        try:
            # 根据尝试次数调整参数
            if planner_name == 'hybrid_astar':
                # 混合A*参数调整
                max_iterations = 6000 + attempt * 2000
                quality_threshold = 0.6 - attempt * 0.1
                
                if self.debug:
                    print(f"混合A*规划: 最大迭代次数 {max_iterations}")
                
                path = planner.plan_path(
                    start=start,
                    goal=goal,
                    agent_id=vehicle_id if 'vehicle_id' in locals() else 'temp',
                    max_iterations=max_iterations,
                    quality_threshold=max(0.3, quality_threshold)
                )
                
                if path and len(path) >= 2:
                    if self.debug:
                        print(f"混合A*规划成功，路径长度: {len(path)}")
                    return path
            
            elif planner_name == 'rrt':
                # RRT参数调整
                max_iterations = 4000 + attempt * 1000
                
                if self.debug:
                    print(f"RRT规划: 最大迭代次数 {max_iterations}")
                
                if hasattr(planner, 'plan_path'):
                    path = planner.plan_path(
                        start=start,
                        goal=goal,
                        agent_id=vehicle_id if 'vehicle_id' in locals() else 'temp',
                        max_iterations=max_iterations,
                        quality_threshold=0.6
                    )
                else:
                    # 回退到原始接口
                    path = planner.plan_path(start, goal, max_iterations=max_iterations)
                
                if path and len(path) >= 2:
                    if self.debug:
                        print(f"RRT规划成功，路径长度: {len(path)}")
                    return path
            
            return None
            
        except Exception as e:
            if self.debug:
                print(f"{planner_name} 规划失败: {e}")
            return None
    
    def _try_backbone_planning(self, start, goal, attempt):
        """尝试使用骨干网络规划路径"""
        try:
            # 1. 识别目标点类型
            target_type, target_id = self.backbone_network.identify_target_point(goal)
            
            if not target_type:
                if self.debug:
                    print("目标不是特殊点，无法使用骨干路径")
                return None
            
            if self.debug:
                print(f"目标识别为: {target_type}_{target_id}")
            
            # 2. 优先使用增强的接口系统
            if hasattr(self.backbone_network, 'get_complete_path_via_interface_enhanced'):
                # 获取RRT引导信息
                guidance = None
                if hasattr(self.backbone_network, 'get_sampling_guidance_for_rrt'):
                    guidance = self.backbone_network.get_sampling_guidance_for_rrt(start, goal)
                
                # 准备提示
                hints = self._prepare_planning_hints(guidance, start, goal)
                
                complete_path, structure = self.backbone_network.get_complete_path_via_interface_enhanced(
                    start, target_type, target_id, hints
                )
                
                if complete_path and structure:
                    if self.debug:
                        print(f"增强接口系统规划成功: 总长度{len(complete_path)}")
                    return complete_path, structure
            
            # 3. 回退到标准接口系统
            elif hasattr(self.backbone_network, 'get_path_from_position_to_target_via_interface'):
                complete_path, structure = self.backbone_network.get_path_from_position_to_target_via_interface(
                    start, target_type, target_id
                )
                
                if complete_path and structure:
                    if self.debug:
                        print(f"标准接口系统规划成功: 总长度{len(complete_path)}")
                    return complete_path, structure
            
            return None
            
        except Exception as e:
            if self.debug:
                print(f"骨干网络规划失败: {e}")
            return None
    
    def _prepare_planning_hints(self, guidance, start, goal):
        """准备规划提示信息"""
        hints = {}
        
        if guidance:
            # 采样区域提示
            if guidance.get('priority_regions'):
                hints['priority_sampling_regions'] = guidance['priority_regions']
            
            # 骨干路径提示
            if guidance.get('backbone_hints'):
                hints['backbone_alignment_targets'] = guidance['backbone_hints']
        
        # 距离自适应
        distance = self._calculate_distance(start, goal)
        if distance > 100:
            hints['performance_mode'] = 'exploration'
        elif distance < 20:
            hints['performance_mode'] = 'precision'
        
        return hints
    
    def get_planner_stats(self, planner_name: str = None) -> Dict:
        """获取规划器统计信息"""
        if planner_name and planner_name in self.planners:
            planner = self.planners[planner_name]
            if hasattr(planner, 'get_statistics'):
                return planner.get_statistics()
            return {}
        
        # 返回所有规划器的统计
        all_stats = {}
        for name, planner in self.planners.items():
            if hasattr(planner, 'get_statistics'):
                all_stats[name] = planner.get_statistics()
        
        return all_stats
    
    def get_performance_stats(self):
        """获取性能统计信息"""
        stats = self.performance_stats.copy()
        
        # 计算平均值
        if stats['planning_times']:
            stats['avg_planning_time'] = sum(stats['planning_times']) / len(stats['planning_times'])
            stats['max_planning_time'] = max(stats['planning_times'])
            stats['min_planning_time'] = min(stats['planning_times'])
        
        if stats['quality_scores']:
            stats['avg_quality_score'] = sum(stats['quality_scores']) / len(stats['quality_scores'])
            stats['max_quality_score'] = max(stats['quality_scores'])
            stats['min_quality_score'] = min(stats['quality_scores'])
        
        # 缓存统计
        total_requests = stats['cache_hits'] + stats['cache_misses']
        if total_requests > 0:
            stats['cache_hit_rate'] = stats['cache_hits'] / total_requests
        
        # 骨干网络使用率
        total_planning = stats['backbone_usage'] + stats['direct_planning']
        if total_planning > 0:
            stats['backbone_success_rate'] = stats['backbone_usage'] / total_planning
        
        # 规划器统计
        stats['available_planners'] = list(self.planners.keys())
        stats['primary_planner'] = self.primary_planner
        
        # 各规划器详细统计
        planner_stats = self.get_planner_stats()
        stats['planner_details'] = planner_stats
        
        return stats
    
    def switch_primary_planner(self, planner_name: str) -> bool:
        """切换主要规划器"""
        if planner_name in self.planners:
            old_planner = self.primary_planner
            self.primary_planner = planner_name
            print(f"主规划器从 {old_planner} 切换到 {planner_name}")
            return True
        else:
            print(f"规划器 {planner_name} 不可用")
            return False
    
    def set_planner_config(self, planner_name: str, config: Dict) -> bool:
        """设置规划器配置"""
        if planner_name in self.planners:
            planner = self.planners[planner_name]
            
            # 为不同规划器设置不同的配置
            try:
                if planner_name == 'hybrid_astar' and hasattr(planner, 'config'):
                    planner.config.update(config)
                elif planner_name == 'rrt' and hasattr(planner, 'planning_config'):
                    planner.planning_config.update(config)
                
                print(f"已更新 {planner_name} 规划器配置")
                return True
            except Exception as e:
                print(f"设置 {planner_name} 配置失败: {e}")
                return False
        
        return False
    
    # 以下方法保持不变，从原始代码复制
    def _resolve_path_conflicts(self, vehicle_id, start, goal, current_path):
        """解决路径冲突"""
        if not self.traffic_manager:
            return current_path
        
        try:
            adjusted_path = self.traffic_manager.suggest_path_adjustment(vehicle_id, start, goal)
            if adjusted_path:
                if self.debug:
                    print(f"交通管理器建议路径调整，新路径长度: {len(adjusted_path)}")
                return adjusted_path
            return current_path
        except Exception as e:
            if self.debug:
                print(f"冲突解决失败: {e}")
            return current_path
    
    def _post_process_path(self, path):
        """路径后处理优化"""
        if not path or len(path) < 3:
            return path
        
        optimized = path
        
        if self.planning_config['enable_shortcut_optimization']:
            optimized = self._shortcut_optimization(optimized)
        
        if self.planning_config['enable_post_smoothing']:
            optimized = self._smooth_path(optimized)
        
        return optimized
    
    def _shortcut_optimization(self, path):
        """捷径优化"""
        if len(path) < 3:
            return path
        
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            found_shortcut = False
            
            while j > i + 1:
                if self._is_line_collision_free(path[i], path[j]):
                    optimized.append(path[j])
                    i = j
                    found_shortcut = True
                    break
                j -= 1
            
            if not found_shortcut:
                optimized.append(path[i + 1])
                i += 1
        
        return optimized
    
    def _smooth_path(self, path, iterations=2):
        """路径平滑"""
        if len(path) <= 2:
            return path
        
        smoothed = list(path)
        
        for _ in range(iterations):
            new_smoothed = [smoothed[0]]
            
            for i in range(1, len(smoothed) - 1):
                prev = smoothed[i-1]
                curr = smoothed[i]
                next_p = smoothed[i+1]
                
                x = (prev[0] + curr[0] + next_p[0]) / 3
                y = (prev[1] + curr[1] + next_p[1]) / 3
                theta = curr[2] if len(curr) > 2 else 0
                
                if self._is_valid_position(int(x), int(y)):
                    new_smoothed.append((x, y, theta))
                else:
                    new_smoothed.append(curr)
            
            new_smoothed.append(smoothed[-1])
            smoothed = new_smoothed
        
        return smoothed
    
    def _analyze_path_structure(self, path):
        """分析路径结构"""
        if not path or len(path) < 2:
            return {'type': 'empty'}
        
        return {
            'type': 'analyzed',
            'length': self._calculate_path_length(path),
            'segments': len(path) - 1,
            'complexity': self._calculate_path_complexity(path),
            'quality': self.quality_assessor.evaluate_path(path)
        }
    
    def _calculate_path_complexity(self, path):
        """计算路径复杂度"""
        if len(path) < 3:
            return 0
        
        total_turning = 0
        for i in range(1, len(path) - 1):
            angle = self._calculate_turning_angle(path[i-1], path[i], path[i+1])
            total_turning += angle
        
        length = self._calculate_path_length(path)
        return total_turning / max(1, length) if length > 0 else 0
    
    def _calculate_turning_angle(self, p1, p2, p3):
        """计算转弯角度"""
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        len_v1 = np.linalg.norm(v1)
        len_v2 = np.linalg.norm(v2)
        
        if len_v1 < 0.001 or len_v2 < 0.001:
            return 0
        
        cos_angle = np.dot(v1, v2) / (len_v1 * len_v2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        return math.acos(cos_angle)
    
    # 缓存管理方法
    def _generate_cache_key(self, vehicle_id, start, goal):
        """生成缓存键"""
        start_rounded = (round(start[0], 1), round(start[1], 1))
        goal_rounded = (round(goal[0], 1), round(goal[1], 1))
        return f"{vehicle_id}:{start_rounded}:{goal_rounded}"
    
    def _check_cache(self, cache_key):
        """检查缓存"""
        if cache_key in self.route_cache:
            metadata = self.cache_metadata.get(cache_key, {})
            current_time = time.time()
            
            if (current_time - metadata.get('timestamp', 0)) > self.cache_config['ttl']:
                del self.route_cache[cache_key]
                del self.cache_metadata[cache_key]
                return None
            
            self.route_cache.move_to_end(cache_key)
            metadata['hit_count'] = metadata.get('hit_count', 0) + 1
            return self.route_cache[cache_key]
        
        return None
    
    def _add_to_cache(self, cache_key, result, quality):
        """添加到缓存"""
        if len(self.route_cache) >= self.cache_config['max_size']:
            oldest_key = next(iter(self.route_cache))
            del self.route_cache[oldest_key]
            del self.cache_metadata[oldest_key]
        
        self.route_cache[cache_key] = result
        self.cache_metadata[cache_key] = {
            'timestamp': time.time(),
            'quality': quality,
            'hit_count': 0
        }
    
    def _clear_cache(self):
        """清空缓存"""
        self.route_cache.clear()
        self.cache_metadata.clear()
    
    # 工具方法
    def _validate_inputs(self, start, goal):
        """验证输入参数"""
        if not start or not goal:
            return False
        
        if len(start) < 2 or len(goal) < 2:
            return False
        
        for pos in [start, goal]:
            x, y = pos[0], pos[1]
            if (x < 0 or x >= self.env.width or 
                y < 0 or y >= self.env.height):
                return False
        
        return True
    
    def _is_same_position(self, pos1, pos2, tolerance=0.1):
        """判断两个位置是否相同"""
        return self._calculate_distance(pos1, pos2) < tolerance
    
    def _calculate_distance(self, pos1, pos2):
        """计算两点间距离"""
        x1 = pos1[0] if len(pos1) > 0 else 0
        y1 = pos1[1] if len(pos1) > 1 else 0
        x2 = pos2[0] if len(pos2) > 0 else 0
        y2 = pos2[1] if len(pos2) > 1 else 0
        
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def _calculate_path_length(self, path):
        """计算路径总长度"""
        if not path or len(path) < 2:
            return 0
        
        length = 0
        for i in range(len(path) - 1):
            length += self._calculate_distance(path[i], path[i + 1])
        
        return length
    
    def _is_valid_position(self, x, y):
        """检查位置是否有效"""
        if not hasattr(self.env, 'grid'):
            return True
        
        if x < 0 or x >= self.env.width or y < 0 or y >= self.env.height:
            return False
        
        return self.env.grid[x, y] == 0
    
    def _is_line_collision_free(self, p1, p2):
        """检查直线是否无碰撞"""
        distance = self._calculate_distance(p1, p2)
        steps = max(10, int(distance))
        
        for i in range(steps + 1):
            t = i / max(1, steps)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            
            if not self._is_valid_position(int(x), int(y)):
                return False
        
        return True
    
    def _validate_path(self, path):
        """验证路径有效性"""
        if not path or len(path) < 2:
            return False
        
        for i in range(len(path) - 1):
            if not self._is_line_collision_free(path[i], path[i+1]):
                return False
        
        return True
    
    def reset_stats(self):
        """重置统计信息"""
        self.performance_stats = {
            'total_requests': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'backbone_usage': 0,
            'direct_planning': 0,
            'planning_times': [],
            'quality_scores': [],
            'planner_usage': defaultdict(int),
            'planner_success_rate': defaultdict(float),
            'backbone_success_rate': 0.0
        }
    
    def set_debug(self, enable):
        """设置调试模式"""
        self.debug = enable
        self.verbose_logging = enable
        
        # 为所有规划器设置调试模式
        for planner in self.planners.values():
            if hasattr(planner, 'debug'):
                planner.debug = enable
        
        print(f"调试模式: {'开启' if enable else '关闭'}")

# 路径质量评估器保持不变
class PathQualityAssessor:
    """路径质量评估器"""
    
    def __init__(self, env):
        self.env = env
        self.weights = {
            'length_efficiency': 0.3,
            'smoothness': 0.3,
            'safety': 0.2,
            'complexity': 0.2
        }
    
    def evaluate_path(self, path):
        """综合评估路径质量"""
        if not path or len(path) < 2:
            return 0
        
        scores = {}
        scores['length_efficiency'] = self._evaluate_length_efficiency(path)
        scores['smoothness'] = self._evaluate_smoothness(path)
        scores['safety'] = self._evaluate_safety(path)
        scores['complexity'] = self._evaluate_complexity(path)
        
        total_score = sum(
            scores[metric] * self.weights[metric] 
            for metric in scores
        )
        
        return min(1.0, max(0.0, total_score))
    
    def _evaluate_length_efficiency(self, path):
        """评估长度效率"""
        actual_length = self._calculate_path_length(path)
        direct_distance = math.sqrt(
            (path[-1][0] - path[0][0])**2 + 
            (path[-1][1] - path[0][1])**2
        )
        
        if direct_distance < 0.1:
            return 1.0
        
        efficiency = direct_distance / (actual_length + 0.1)
        return min(1.0, efficiency)
    
    def _evaluate_smoothness(self, path):
        """评估路径平滑度"""
        if len(path) < 3:
            return 1.0
        
        total_curvature = 0
        segments = 0
        
        for i in range(1, len(path) - 1):
            curvature = self._calculate_curvature(path[i-1], path[i], path[i+1])
            total_curvature += curvature
            segments += 1
        
        if segments == 0:
            return 1.0
        
        avg_curvature = total_curvature / segments
        smoothness = math.exp(-avg_curvature * 2)
        
        return min(1.0, smoothness)
    
    def _evaluate_safety(self, path):
        """评估路径安全性"""
        min_safety = 1.0
        
        for point in path[::max(1, len(path)//10)]:
            safety = self._calculate_point_safety(point)
            min_safety = min(min_safety, safety)
        
        return min_safety
    
    def _evaluate_complexity(self, path):
        """评估路径复杂度"""
        if len(path) < 3:
            return 1.0
        
        sharp_turns = 0
        total_segments = len(path) - 2
        
        for i in range(1, len(path) - 1):
            angle = self._calculate_turning_angle(path[i-1], path[i], path[i+1])
            if angle > math.pi / 6:
                sharp_turns += 1
        
        complexity_score = 1.0 - (sharp_turns / max(1, total_segments))
        return max(0, complexity_score)
    
    def _calculate_curvature(self, p1, p2, p3):
        """计算曲率"""
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        len_v1 = np.linalg.norm(v1)
        len_v2 = np.linalg.norm(v2)
        
        if len_v1 < 0.001 or len_v2 < 0.001:
            return 0
        
        v1_norm = v1 / len_v1
        v2_norm = v2 / len_v2
        
        dot_product = np.dot(v1_norm, v2_norm)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        
        angle_change = math.acos(dot_product)
        avg_segment_length = (len_v1 + len_v2) / 2
        
        return angle_change / (avg_segment_length + 0.001)
    
    def _calculate_turning_angle(self, p1, p2, p3):
        """计算转弯角度"""
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        len_v1 = np.linalg.norm(v1)
        len_v2 = np.linalg.norm(v2)
        
        if len_v1 < 0.001 or len_v2 < 0.001:
            return 0
        
        cos_angle = np.dot(v1, v2) / (len_v1 * len_v2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        return math.acos(cos_angle)
    
    def _calculate_point_safety(self, point):
        """计算点的安全性"""
        clearance = self._calculate_clearance(point)
        
        if clearance >= 5:
            return 1.0
        elif clearance >= 2:
            return 0.8
        elif clearance >= 1:
            return 0.5
        else:
            return 0.2
    
    def _calculate_clearance(self, point):
        """计算到最近障碍物的距离"""
        min_distance = float('inf')
        
        x, y = int(point[0]), int(point[1])
        search_radius = 10
        
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x, check_y = x + dx, y + dy
                
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height and 
                    hasattr(self.env, 'grid') and
                    self.env.grid[check_x, check_y] == 1):
                    
                    distance = math.sqrt(dx*dx + dy*dy)
                    min_distance = min(min_distance, distance)
        
        return min_distance if min_distance != float('inf') else 10
    
    def _calculate_path_length(self, path):
        """计算路径总长度"""
        if not path or len(path) < 2:
            return 0
        
        length = 0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.sqrt(dx*dx + dy*dy)
        
        return length

# 向后兼容性
SimplifiedPathPlanner = EnhancedPathPlanner
PathPlanner = EnhancedPathPlanner
OptimizedPathPlanner = EnhancedPathPlanner
AdvancedPathPlanner = EnhancedPathPlanner