"""
path_planner.py - 精简优化版统一路径规划器
专注核心规划功能，移除过度复杂的选择逻辑
"""

import time
import math
import threading
from typing import List, Tuple, Dict, Optional, Any, Union
from collections import OrderedDict, namedtuple
from dataclasses import dataclass

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

# 规划结果类型
PlanningResult = namedtuple('PlanningResult', [
    'path', 'planner_used', 'quality_score', 'planning_time', 'structure'
])

@dataclass
class PlannerConfig:
    """规划器配置"""
    max_planning_time: float = 15.0
    quality_threshold: float = 0.6
    cache_size: int = 200
    enable_fallback: bool = True
    default_planner: str = "hybrid_astar"
    timeout_retry_count: int = 2
    
    # 规划器参数
    astar_params: Dict = None
    rrt_params: Dict = None
    
    def __post_init__(self):
        if self.astar_params is None:
            self.astar_params = {
                'vehicle_length': 6.0,
                'vehicle_width': 3.0,
                'turning_radius': 8.0,
                'step_size': 2.0,
                'angle_resolution': 30
            }
        
        if self.rrt_params is None:
            self.rrt_params = {
                'vehicle_length': 5.0,
                'vehicle_width': 2.0,
                'turning_radius': 5.0,
                'step_size': 0.8
            }

class SimplifiedPathPlanner:
    """精简版统一路径规划器 - 专注核心功能"""
    
    def __init__(self, env, backbone_network=None, traffic_manager=None, config=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.traffic_manager = traffic_manager
        
        # 配置
        self.config = config or PlannerConfig()
        
        # 规划器实例
        self.planners = {}
        self._initialize_planners()
        
        # 简化缓存
        self.cache = OrderedDict()
        self.cache_lock = threading.RLock()
        
        # 简化统计
        self.stats = {
            'total_requests': 0,
            'successful_plans': 0,
            'astar_success': 0,
            'astar_failed': 0,
            'rrt_success': 0,
            'rrt_failed': 0,
            'cache_hits': 0,
            'fallback_used': 0
        }
        
        print(f"初始化精简路径规划器: A*={ASTAR_AVAILABLE}, RRT={RRT_AVAILABLE}")
    
    def _initialize_planners(self):
        """初始化规划器"""
        try:
            # 初始化混合A*
            if ASTAR_AVAILABLE:
                self.planners['hybrid_astar'] = HybridAStarPlanner(
                    self.env,
                    **self.config.astar_params
                )
                print("✓ 混合A*规划器已初始化")
            
            # 初始化RRT
            if RRT_AVAILABLE:
                self.planners['rrt'] = SimplifiedRRTPlanner(
                    self.env,
                    **self.config.rrt_params
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
    
    def plan_path(self, vehicle_id: str, start: Tuple, goal: Tuple,
                  use_backbone: bool = True, check_conflicts: bool = True,
                  planner_type: str = "auto", context: str = "normal",
                  return_object: bool = False, **kwargs) -> Optional[Union[List, Tuple[List, Dict], Any]]:
        """
        统一路径规划接口
        
        Args:
            vehicle_id: 车辆ID
            start: 起点 (x, y, theta)
            goal: 终点 (x, y, theta)
            use_backbone: 是否使用骨干网络
            check_conflicts: 是否检查冲突
            planner_type: 规划器类型 ("auto", "hybrid_astar", "rrt")
            context: 规划上下文 ("normal", "backbone", "navigation")
            return_object: 是否返回结果对象格式（用于backbone_network）
            
        Returns:
            路径点列表、(路径, 结构信息)元组或结果对象
        """
        self.stats['total_requests'] += 1
        planning_start = time.time()
        
        # 输入验证
        if not self._validate_inputs(start, goal):
            return None
        
        # 检查缓存
        cache_key = self._generate_cache_key(start, goal, use_backbone, planner_type)
        cached_result = self._check_cache(cache_key)
        if cached_result:
            self.stats['cache_hits'] += 1
            # 根据需求返回不同格式
            if return_object and isinstance(cached_result, tuple):
                return self._tuple_to_object(cached_result)
            return cached_result
        
        # 骨干路径生成时优先使用混合A*
        if context == "backbone" and planner_type == "auto":
            planner_type = "hybrid_astar"
        
        # 根据上下文选择策略
        result = self._plan_with_strategy(
            vehicle_id, start, goal, planner_type, context, 
            use_backbone, planning_start, **kwargs
        )
        
        if result:
            # 缓存结果
            self._add_to_cache(cache_key, result)
            self.stats['successful_plans'] += 1
            
            # 根据需求返回不同格式
            if return_object and isinstance(result, tuple):
                return self._tuple_to_object(result)
        
        return result
    
    def _plan_with_strategy(self, vehicle_id: str, start: Tuple, goal: Tuple,
                           planner_type: str, context: str, use_backbone: bool,
                           planning_start: float, **kwargs) -> Optional[Any]:
        """根据策略进行规划"""
        if planner_type != "auto":
            # 指定规划器
            return self._plan_with_specific_planner(
                planner_type, vehicle_id, start, goal, use_backbone, planning_start, **kwargs
            )
        
        # 自动选择策略
        if context == "backbone":
            # 骨干路径优先混合A*
            return self._try_astar_then_rrt(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
        elif context == "navigation":
            # 导航段优先RRT
            return self._try_rrt_then_astar(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
        else:
            # 默认策略：混合A* -> RRT
            return self._try_astar_then_rrt(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
    
    def _try_astar_then_rrt(self, vehicle_id: str, start: Tuple, goal: Tuple,
                           use_backbone: bool, planning_start: float, **kwargs) -> Optional[Any]:
        """混合A* -> RRT回退策略"""
        # 尝试混合A*
        result = self._plan_with_astar(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
        if result:
            self.stats['astar_success'] += 1
            return result
        
        self.stats['astar_failed'] += 1
        
        # 回退到RRT
        result = self._plan_with_rrt(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
        if result:
            self.stats['rrt_success'] += 1
            return result
        
        self.stats['rrt_failed'] += 1
        
        # 最后回退到直线路径
        if self.config.enable_fallback:
            return self._fallback_direct_path(start, goal, planning_start)
        
        return None
    
    def _try_rrt_then_astar(self, vehicle_id: str, start: Tuple, goal: Tuple,
                           use_backbone: bool, planning_start: float, **kwargs) -> Optional[Any]:
        """RRT -> 混合A*回退策略"""
        # 尝试RRT
        result = self._plan_with_rrt(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
        if result:
            self.stats['rrt_success'] += 1
            return result
        
        self.stats['rrt_failed'] += 1
        
        # 回退到混合A*
        result = self._plan_with_astar(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
        if result:
            self.stats['astar_success'] += 1
            return result
        
        self.stats['astar_failed'] += 1
        
        # 最后回退到直线路径
        if self.config.enable_fallback:
            return self._fallback_direct_path(start, goal, planning_start)
        
        return None
    
    def _plan_with_specific_planner(self, planner_type: str, vehicle_id: str,
                                  start: Tuple, goal: Tuple, use_backbone: bool,
                                  planning_start: float, **kwargs) -> Optional[Any]:
        """使用指定规划器"""
        if planner_type == "hybrid_astar":
            result = self._plan_with_astar(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
            if result:
                self.stats['astar_success'] += 1
            else:
                self.stats['astar_failed'] += 1
            return result
        
        elif planner_type == "rrt":
            result = self._plan_with_rrt(vehicle_id, start, goal, use_backbone, planning_start, **kwargs)
            if result:
                self.stats['rrt_success'] += 1
            else:
                self.stats['rrt_failed'] += 1
            return result
        
        elif planner_type == "direct":
            return self._fallback_direct_path(start, goal, planning_start)
        
        return None
    
    def _plan_with_astar(self, vehicle_id: str, start: Tuple, goal: Tuple,
                        use_backbone: bool, planning_start: float, **kwargs) -> Optional[Any]:
        """使用混合A*规划"""
        if 'hybrid_astar' not in self.planners:
            return None
        
        try:
            planner = self.planners['hybrid_astar']
            
            # 调用规划器
            path = planner.plan_path(
                start, goal, 
                agent_id=vehicle_id,
                quality_threshold=kwargs.get('quality_threshold', self.config.quality_threshold)
            )
            
            if path and len(path) >= 2:
                planning_time = time.time() - planning_start
                quality = self._evaluate_path_quality(path)
                
                structure = {
                    'planner_used': 'hybrid_astar',
                    'final_quality': quality,
                    'planning_time': planning_time,
                    'type': 'astar',
                    'backbone_utilization': 0.0,  # 简化，不详细计算
                    'total_length': len(path)
                }
                
                return (path, structure)
        
        except Exception as e:
            print(f"混合A*规划失败: {e}")
        
        return None
    
    def _plan_with_rrt(self, vehicle_id: str, start: Tuple, goal: Tuple,
                      use_backbone: bool, planning_start: float, **kwargs) -> Optional[Any]:
        """使用RRT规划"""
        if 'rrt' not in self.planners:
            return None
        
        try:
            planner = self.planners['rrt']
            
            # 调用规划器
            path = planner.plan_path(
                start, goal,
                agent_id=vehicle_id,
                quality_threshold=kwargs.get('quality_threshold', self.config.quality_threshold)
            )
            
            if path and len(path) >= 2:
                planning_time = time.time() - planning_start
                quality = self._evaluate_path_quality(path)
                
                structure = {
                    'planner_used': 'rrt',
                    'final_quality': quality,
                    'planning_time': planning_time,
                    'type': 'rrt',
                    'backbone_utilization': 0.0,  # 简化，不详细计算
                    'total_length': len(path)
                }
                
                return (path, structure)
        
        except Exception as e:
            print(f"RRT规划失败: {e}")
        
        return None
    
    def _fallback_direct_path(self, start: Tuple, goal: Tuple, 
                             planning_start: float) -> Tuple[List, Dict]:
        """回退直线路径"""
        try:
            self.stats['fallback_used'] += 1
            
            # 确保3D坐标
            start_3d = self._ensure_3d_coordinates(start)
            goal_3d = self._ensure_3d_coordinates(goal)
            
            # 计算路径点
            distance = math.sqrt(
                (goal_3d[0] - start_3d[0])**2 + 
                (goal_3d[1] - start_3d[1])**2
            )
            
            # 根据距离确定步数
            steps = max(3, int(distance / 2.0))
            path = []
            
            for i in range(steps + 1):
                t = i / steps
                x = start_3d[0] + t * (goal_3d[0] - start_3d[0])
                y = start_3d[1] + t * (goal_3d[1] - start_3d[1])
                theta = start_3d[2] + t * (goal_3d[2] - start_3d[2])
                path.append((x, y, theta))
            
            planning_time = time.time() - planning_start
            
            structure = {
                'planner_used': 'direct',
                'final_quality': 0.6,  # 直线路径的基准质量
                'planning_time': planning_time,
                'type': 'direct',
                'backbone_utilization': 0.0,
                'total_length': len(path)
            }
            
            return (path, structure)
        
        except Exception as e:
            print(f"直线路径生成失败: {e}")
            return None
    
    def _evaluate_path_quality(self, path: List) -> float:
        """评估路径质量"""
        if not path or len(path) < 2:
            return 0.0
        
        try:
            # 计算路径长度
            path_length = 0.0
            for i in range(len(path) - 1):
                dx = path[i+1][0] - path[i][0]
                dy = path[i+1][1] - path[i][1]
                path_length += math.sqrt(dx*dx + dy*dy)
            
            # 计算直线距离
            start, end = path[0], path[-1]
            direct_distance = math.sqrt(
                (end[0] - start[0])**2 + (end[1] - start[1])**2
            )
            
            # 长度效率
            if path_length > 0:
                length_efficiency = direct_distance / path_length
            else:
                length_efficiency = 1.0
            
            # 平滑度评估（简化）
            smoothness = self._calculate_smoothness(path)
            
            # 综合质量评分
            quality = length_efficiency * 0.6 + smoothness * 0.4
            return min(1.0, max(0.0, quality))
        
        except Exception:
            return 0.5  # 默认质量
    
    def _calculate_smoothness(self, path: List) -> float:
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
            return 0.8  # 默认平滑度
    
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
    
    def _tuple_to_object(self, result_tuple: Tuple[List, Dict]) -> Any:
        """将tuple格式转换为对象格式（用于backbone_network兼容性）"""
        if not isinstance(result_tuple, tuple) or len(result_tuple) != 2:
            return result_tuple
        
        path, structure = result_tuple
        
        # 创建具有所需属性的结果对象
        class PlanningResultObject:
            def __init__(self, path, structure):
                self.path = path
                self.quality_score = structure.get('final_quality', 0.5)
                self.planner_used = structure.get('planner_used', 'unknown')
                self.planning_time = structure.get('planning_time', 0.0)
                self.structure = structure
        
        return PlanningResultObject(path, structure)
    
    def _ensure_3d_coordinates(self, point: Tuple) -> Tuple[float, float, float]:
        """确保坐标为3D格式"""
        if len(point) >= 3:
            return (float(point[0]), float(point[1]), float(point[2]))
        elif len(point) == 2:
            return (float(point[0]), float(point[1]), 0.0)
        else:
            return (0.0, 0.0, 0.0)
    
    def _generate_cache_key(self, start: Tuple, goal: Tuple, 
                           use_backbone: bool, planner_type: str) -> str:
        """生成缓存键"""
        start_key = f"{start[0]:.1f},{start[1]:.1f}"
        goal_key = f"{goal[0]:.1f},{goal[1]:.1f}"
        return f"{start_key}_{goal_key}_{use_backbone}_{planner_type}"
    
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
    
    def get_performance_stats(self) -> Dict:
        """获取性能统计"""
        stats = self.stats.copy()
        
        # 计算成功率
        total_astar = stats['astar_success'] + stats['astar_failed']
        total_rrt = stats['rrt_success'] + stats['rrt_failed']
        
        if total_astar > 0:
            stats['astar_success_rate'] = stats['astar_success'] / total_astar
        else:
            stats['astar_success_rate'] = 0.0
        
        if total_rrt > 0:
            stats['rrt_success_rate'] = stats['rrt_success'] / total_rrt
        else:
            stats['rrt_success_rate'] = 0.0
        
        if stats['total_requests'] > 0:
            stats['overall_success_rate'] = stats['successful_plans'] / stats['total_requests']
            stats['cache_hit_rate'] = stats['cache_hits'] / stats['total_requests']
        else:
            stats['overall_success_rate'] = 0.0
            stats['cache_hit_rate'] = 0.0
        
        # 添加实时信息
        stats['available_planners'] = list(self.planners.keys())
        stats['cache_size'] = len(self.cache)
        stats['backbone_network_available'] = self.backbone_network is not None
        
        return stats
    
    def clear_cache(self):
        """清理缓存"""
        with self.cache_lock:
            self.cache.clear()
        
        # 重置缓存相关统计
        self.stats['cache_hits'] = 0
        
        print("路径规划器缓存已清理")
    
    def get_planner_info(self, planner_name: str) -> Optional[Dict]:
        """获取规划器信息"""
        if planner_name not in self.planners:
            return None
        
        planner = self.planners[planner_name]
        info = {
            'name': planner_name,
            'available': True,
            'type': type(planner).__name__
        }
        
        # 获取规划器统计（如果有）
        if hasattr(planner, 'get_statistics'):
            try:
                info['statistics'] = planner.get_statistics()
            except Exception:
                pass
        
        return info
    
    def reset_statistics(self):
        """重置统计信息"""
        self.stats = {
            'total_requests': 0,
            'successful_plans': 0,
            'astar_success': 0,
            'astar_failed': 0,
            'rrt_success': 0,
            'rrt_failed': 0,
            'cache_hits': 0,
            'fallback_used': 0
        }
        
        print("路径规划器统计信息已重置")
    
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
        print("路径规划器已关闭")

# 向后兼容性
EnhancedPathPlanner = SimplifiedPathPlanner
UnifiedPathPlanner = SimplifiedPathPlanner
PathPlanner = SimplifiedPathPlanner