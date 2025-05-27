"""
traffic_manager.py - 增强版交通管理器
实现停车避让机制、骨干路径动态切换、智能冲突解决策略
"""

import math
import time
import threading
from typing import List, Dict, Tuple, Optional, Any
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass
from enum import Enum

class ConflictType(Enum):
    """冲突类型"""
    VERTEX = "vertex"           # 顶点冲突
    EDGE = "edge"               # 边冲突
    INTERFACE = "interface"     # 接口冲突
    BACKBONE_CONGESTION = "backbone_congestion"  # 骨干路径拥堵

class ResolutionStrategy(Enum):
    """冲突解决策略"""
    REPLAN = "replan"          # 重新规划
    WAIT = "wait"              # 停车等待
    REROUTE = "reroute"        # 改道
    BACKBONE_SWITCH = "backbone_switch"  # 骨干路径切换
    PRIORITY_OVERRIDE = "priority_override"  # 优先级覆盖

@dataclass
class EnhancedConflict:
    """增强冲突表示"""
    conflict_id: str
    agent1: str
    agent2: str
    location: Tuple[float, float]
    time_step: int
    conflict_type: ConflictType = ConflictType.VERTEX
    severity: float = 1.0  # 冲突严重程度 (0-1)
    suggested_strategy: ResolutionStrategy = ResolutionStrategy.REPLAN
    
    def __lt__(self, other):
        """优先级比较 - 严重程度高的优先"""
        if abs(self.severity - other.severity) > 0.1:
            return self.severity > other.severity
        return self.time_step < other.time_step

@dataclass
class ParkingManeuver:
    """停车操作"""
    vehicle_id: str
    parking_location: Tuple[float, float]
    wait_duration: float
    start_time: float
    reason: str = "conflict_avoidance"

class ConflictResolutionStrategy:
    """冲突解决策略管理器"""
    
    def __init__(self, env, path_planner=None, backbone_network=None):
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        
        # 策略权重配置
        self.strategy_weights = {
            ResolutionStrategy.WAIT: 0.3,
            ResolutionStrategy.REPLAN: 0.4,
            ResolutionStrategy.REROUTE: 0.2,
            ResolutionStrategy.BACKBONE_SWITCH: 0.1
        }
        
        # 车辆优先级
        self.vehicle_priorities = {}
        
        # 停车点管理
        self.available_parking_spots = self._initialize_parking_spots()
        self.active_parking_maneuvers = {}  # {vehicle_id: ParkingManeuver}
        
        # 统计
        self.resolution_stats = {
            'strategies_used': defaultdict(int),
            'success_rates': defaultdict(list),
            'total_conflicts_resolved': 0
        }
    
    def _initialize_parking_spots(self) -> List[Tuple]:
        """初始化可用停车点"""
        parking_spots = []
        
        # 使用环境中的停车区域
        if hasattr(self.env, 'parking_areas'):
            for area in self.env.parking_areas:
                # 在停车区域周围生成多个停车位
                x, y = area[0], area[1]
                for dx in [-5, 0, 5]:
                    for dy in [-5, 0, 5]:
                        if dx != 0 or dy != 0:  # 不在中心
                            parking_spots.append((x + dx, y + dy))
        
        # 如果没有明确的停车区域，在地图边缘创建停车点
        if not parking_spots:
            for x in range(10, self.env.width - 10, 20):
                for y in [10, self.env.height - 10]:
                    if self._is_point_safe_for_parking((x, y)):
                        parking_spots.append((x, y))
        
        return parking_spots
    
    def _is_point_safe_for_parking(self, point: Tuple) -> bool:
        """检查点是否适合停车"""
        x, y = point
        
        # 检查是否在地图范围内
        if x < 5 or x >= self.env.width - 5 or y < 5 or y >= self.env.height - 5:
            return False
        
        # 检查是否远离障碍物
        if hasattr(self.env, 'grid'):
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    check_x, check_y = int(x + dx), int(y + dy)
                    if (0 <= check_x < self.env.width and 
                        0 <= check_y < self.env.height and
                        self.env.grid[check_x, check_y] == 1):
                        return False
        
        return True
    
    def select_resolution_strategy(self, conflict: EnhancedConflict, 
                                 vehicle_paths: Dict) -> Tuple[ResolutionStrategy, Dict]:
        """选择最优冲突解决策略"""
        
        # 分析冲突特征
        conflict_analysis = self._analyze_conflict(conflict, vehicle_paths)
        
        # 评估各种策略的适用性
        strategy_scores = {}
        
        # 1. 等待策略评分
        strategy_scores[ResolutionStrategy.WAIT] = self._evaluate_wait_strategy(
            conflict, conflict_analysis, vehicle_paths
        )
        
        # 2. 重规划策略评分
        strategy_scores[ResolutionStrategy.REPLAN] = self._evaluate_replan_strategy(
            conflict, conflict_analysis, vehicle_paths
        )
        
        # 3. 改道策略评分
        strategy_scores[ResolutionStrategy.REROUTE] = self._evaluate_reroute_strategy(
            conflict, conflict_analysis, vehicle_paths
        )
        
        # 4. 骨干路径切换策略评分
        if self.backbone_network:
            strategy_scores[ResolutionStrategy.BACKBONE_SWITCH] = self._evaluate_backbone_switch_strategy(
                conflict, conflict_analysis, vehicle_paths
            )
        
        # 选择得分最高的策略
        best_strategy = max(strategy_scores, key=strategy_scores.get)
        best_score = strategy_scores[best_strategy]
        
        strategy_params = {
            'score': best_score,
            'conflict_analysis': conflict_analysis,
            'alternative_scores': strategy_scores
        }
        
        return best_strategy, strategy_params
    
    def _analyze_conflict(self, conflict: EnhancedConflict, 
                         vehicle_paths: Dict) -> Dict:
        """分析冲突特征"""
        agent1_path = vehicle_paths.get(conflict.agent1, [])
        agent2_path = vehicle_paths.get(conflict.agent2, [])
        
        analysis = {
            'conflict_duration': 1,  # 冲突持续时间步数
            'path_lengths': {
                conflict.agent1: len(agent1_path),
                conflict.agent2: len(agent2_path)
            },
            'distance_to_goal': {},
            'vehicle_priorities': {
                conflict.agent1: self.vehicle_priorities.get(conflict.agent1, 0.5),
                conflict.agent2: self.vehicle_priorities.get(conflict.agent2, 0.5)
            },
            'available_alternatives': 0
        }
        
        # 计算到目标的剩余距离
        for agent in [conflict.agent1, conflict.agent2]:
            path = vehicle_paths.get(agent, [])
            if len(path) > conflict.time_step:
                remaining_path = path[conflict.time_step:]
                analysis['distance_to_goal'][agent] = self._calculate_path_length(remaining_path)
            else:
                analysis['distance_to_goal'][agent] = 0
        
        # 检查可用的备选方案数量
        if self.backbone_network:
            analysis['available_alternatives'] = len(
                self.backbone_network.find_alternative_backbone_paths(
                    'unloading', 0  # 简化，实际应该从任务中获取
                )
            )
        
        return analysis
    
    def _evaluate_wait_strategy(self, conflict: EnhancedConflict, 
                               analysis: Dict, vehicle_paths: Dict) -> float:
        """评估等待策略的适用性"""
        base_score = 0.6
        
        # 优先级因子：优先级低的车辆更适合等待
        lower_priority_agent = min(analysis['vehicle_priorities'], 
                                 key=analysis['vehicle_priorities'].get)
        priority_factor = 1.0 - analysis['vehicle_priorities'][lower_priority_agent]
        
        # 距离因子：距离目标近的车辆更适合等待
        min_distance = min(analysis['distance_to_goal'].values())
        max_distance = max(analysis['distance_to_goal'].values())
        if max_distance > 0:
            distance_factor = min_distance / max_distance
        else:
            distance_factor = 0.5
        
        # 停车点可用性
        available_spots = len([spot for spot in self.available_parking_spots 
                              if self._is_parking_spot_available(spot)])
        availability_factor = min(1.0, available_spots / 5.0)
        
        final_score = base_score * (0.4 * priority_factor + 
                                  0.3 * distance_factor + 
                                  0.3 * availability_factor)
        
        return final_score
    
    def _evaluate_replan_strategy(self, conflict: EnhancedConflict, 
                                 analysis: Dict, vehicle_paths: Dict) -> float:
        """评估重规划策略的适用性"""
        base_score = 0.7
        
        # 路径长度因子：短路径更容易重规划
        avg_path_length = sum(analysis['path_lengths'].values()) / len(analysis['path_lengths'])
        length_factor = max(0.1, 1.0 - (avg_path_length / 100.0))
        
        # 规划器可用性
        planner_factor = 1.0 if self.path_planner else 0.0
        
        # 冲突严重程度：严重冲突更需要重规划
        severity_factor = conflict.severity
        
        final_score = base_score * (0.4 * length_factor + 
                                  0.3 * planner_factor + 
                                  0.3 * severity_factor)
        
        return final_score
    
    def _evaluate_reroute_strategy(self, conflict: EnhancedConflict, 
                                  analysis: Dict, vehicle_paths: Dict) -> float:
        """评估改道策略的适用性"""
        base_score = 0.5
        
        # 备选路径因子
        alternative_factor = min(1.0, analysis['available_alternatives'] / 3.0)
        
        # 冲突位置因子：在路径中段的冲突更适合改道
        conflict_position_ratio = conflict.time_step / max(1, max(analysis['path_lengths'].values()))
        position_factor = 1.0 - abs(0.5 - conflict_position_ratio) * 2
        
        final_score = base_score * (0.6 * alternative_factor + 0.4 * position_factor)
        
        return final_score
    
    def _evaluate_backbone_switch_strategy(self, conflict: EnhancedConflict, 
                                          analysis: Dict, vehicle_paths: Dict) -> float:
        """评估骨干路径切换策略的适用性"""
        if not self.backbone_network:
            return 0.0
        
        base_score = 0.8
        
        # 备选骨干路径数量
        alternatives_count = analysis['available_alternatives']
        alternatives_factor = min(1.0, alternatives_count / 2.0)
        
        # 当前骨干路径负载
        # 简化处理，实际应该获取冲突车辆使用的骨干路径
        load_factor = 0.7  # 假设值
        
        final_score = base_score * (0.7 * alternatives_factor + 0.3 * (1.0 - load_factor))
        
        return final_score
    
    def _is_parking_spot_available(self, spot: Tuple) -> bool:
        """检查停车点是否可用"""
        # 检查是否有车辆正在使用此停车点
        for maneuver in self.active_parking_maneuvers.values():
            if self._calculate_distance(spot, maneuver.parking_location) < 8.0:
                return False
        
        return True
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径长度"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += self._calculate_distance(path[i], path[i + 1])
        return length
    
    def execute_wait_strategy(self, conflict: EnhancedConflict, 
                             strategy_params: Dict, vehicle_paths: Dict) -> Dict[str, List]:
        """执行等待策略"""
        analysis = strategy_params['conflict_analysis']
        
        # 选择优先级较低的车辆等待
        lower_priority_agent = min(analysis['vehicle_priorities'], 
                                 key=analysis['vehicle_priorities'].get)
        
        # 计算最优等待时间
        wait_time = self._calculate_optimal_wait_time(conflict, vehicle_paths)
        
        # 找到合适的停车位置
        parking_location = self._find_optimal_parking_spot(conflict.location)
        
        if parking_location:
            # 执行停车操作
            new_path = self._insert_parking_maneuver(
                lower_priority_agent, 
                vehicle_paths[lower_priority_agent],
                parking_location,
                wait_time,
                conflict.time_step
            )
            
            if new_path:
                # 记录停车操作
                maneuver = ParkingManeuver(
                    vehicle_id=lower_priority_agent,
                    parking_location=parking_location,
                    wait_duration=wait_time,
                    start_time=time.time(),
                    reason=f"conflict_avoidance_{conflict.conflict_id}"
                )
                
                self.active_parking_maneuvers[lower_priority_agent] = maneuver
                
                # 更新路径
                updated_paths = vehicle_paths.copy()
                updated_paths[lower_priority_agent] = new_path
                
                # 记录统计
                self.resolution_stats['strategies_used'][ResolutionStrategy.WAIT] += 1
                
                print(f"车辆 {lower_priority_agent} 执行停车等待策略，等待时间: {wait_time:.1f}秒")
                
                return updated_paths
        
        # 如果停车策略失败，回退到重规划
        return self.execute_replan_strategy(conflict, strategy_params, vehicle_paths)
    
    def _calculate_optimal_wait_time(self, conflict: EnhancedConflict, 
                                   vehicle_paths: Dict) -> float:
        """计算最优等待时间"""
        # 基础等待时间
        base_wait_time = 30.0
        
        # 根据冲突严重程度调整
        severity_multiplier = 1.0 + (conflict.severity - 0.5) * 0.5
        
        # 根据路径长度调整
        agent1_path = vehicle_paths.get(conflict.agent1, [])
        agent2_path = vehicle_paths.get(conflict.agent2, [])
        avg_remaining = (len(agent1_path) - conflict.time_step + 
                        len(agent2_path) - conflict.time_step) / 2.0
        
        length_multiplier = min(2.0, max(0.5, avg_remaining / 20.0))
        
        optimal_wait_time = base_wait_time * severity_multiplier * length_multiplier
        
        return max(15.0, min(120.0, optimal_wait_time))  # 限制在15-120秒
    
    def _find_optimal_parking_spot(self, conflict_location: Tuple) -> Optional[Tuple]:
        """找到最优停车位置"""
        available_spots = [spot for spot in self.available_parking_spots 
                          if self._is_parking_spot_available(spot)]
        
        if not available_spots:
            # 动态生成临时停车点
            return self._generate_temporary_parking_spot(conflict_location)
        
        # 选择距离冲突位置适中的停车点
        target_distance = 15.0  # 理想距离
        
        best_spot = None
        best_score = float('inf')
        
        for spot in available_spots:
            distance = self._calculate_distance(spot, conflict_location)
            
            # 评分：距离理想距离越近越好
            distance_score = abs(distance - target_distance)
            
            # 安全性评分
            safety_score = 0.0
            if self._is_point_safe_for_parking(spot):
                safety_score = 0.0
            else:
                safety_score = 100.0
            
            total_score = distance_score + safety_score
            
            if total_score < best_score:
                best_score = total_score
                best_spot = spot
        
        return best_spot
    
    def _generate_temporary_parking_spot(self, conflict_location: Tuple) -> Tuple:
        """生成临时停车点"""
        x, y = conflict_location
        
        # 在冲突位置周围生成停车点
        candidates = [
            (x + 15, y),      # 右侧
            (x - 15, y),      # 左侧
            (x, y + 15),      # 上方
            (x, y - 15),      # 下方
            (x + 10, y + 10), # 右上
            (x - 10, y - 10)  # 左下
        ]
        
        for candidate in candidates:
            if self._is_point_safe_for_parking(candidate):
                return candidate
        
        # 如果都不安全，返回相对安全的位置
        return (max(10, min(self.env.width - 10, x + 20)), 
                max(10, min(self.env.height - 10, y)))
    
    def _insert_parking_maneuver(self, vehicle_id: str, original_path: List,
                                parking_location: Tuple, wait_time: float,
                                conflict_step: int) -> Optional[List]:
        """在路径中插入停车操作"""
        if not original_path or conflict_step >= len(original_path):
            return None
        
        try:
            # 当前位置（冲突前的位置）
            current_position = original_path[max(0, conflict_step - 1)]
            
            # 分割原路径
            path_to_conflict = original_path[:conflict_step]
            path_after_conflict = original_path[conflict_step:]
            
            # 规划到停车点的路径
            if self.path_planner:
                parking_approach = self.path_planner.plan_path(
                    vehicle_id=f"{vehicle_id}_parking",
                    start=current_position,
                    goal=parking_location,
                    use_backbone=False
                )
                
                if parking_approach:
                    if hasattr(parking_approach, 'path'):
                        approach_path = parking_approach.path
                    elif isinstance(parking_approach, tuple):
                        approach_path = parking_approach[0]
                    else:
                        approach_path = parking_approach
                else:
                    # 简单直线到停车点
                    approach_path = [current_position, parking_location]
            else:
                approach_path = [current_position, parking_location]
            
            # 生成等待路径段
            wait_steps = max(3, int(wait_time / 5.0))  # 每5秒一个路径点
            wait_path = [parking_location] * wait_steps
            
            # 从停车点回到原路径
            if path_after_conflict:
                resume_point = path_after_conflict[0]
                if self.path_planner:
                    resume_path = self.path_planner.plan_path(
                        vehicle_id=f"{vehicle_id}_resume",
                        start=parking_location,
                        goal=resume_point,
                        use_backbone=False
                    )
                    
                    if resume_path:
                        if hasattr(resume_path, 'path'):
                            resume_path = resume_path.path
                        elif isinstance(resume_path, tuple):
                            resume_path = resume_path[0]
                    else:
                        resume_path = [parking_location, resume_point]
                else:
                    resume_path = [parking_location, resume_point]
                
                # 合并所有路径段
                new_path = (path_to_conflict + 
                           approach_path[1:] +  # 去除重复点
                           wait_path + 
                           resume_path[1:] +    # 去除重复点
                           path_after_conflict[1:])  # 去除重复点
            else:
                # 如果没有后续路径，只执行停车
                new_path = path_to_conflict + approach_path[1:] + wait_path
            
            return new_path
        
        except Exception as e:
            print(f"插入停车操作失败: {e}")
            return None
    
    def execute_replan_strategy(self, conflict: EnhancedConflict, 
                               strategy_params: Dict, vehicle_paths: Dict) -> Dict[str, List]:
        """执行重规划策略"""
        if not self.path_planner:
            return vehicle_paths
        
        analysis = strategy_params['conflict_analysis']
        
        # 选择重规划的车辆（通常选择优先级较低或路径较短的）
        replan_agent = self._select_replan_agent(conflict, analysis)
        
        current_path = vehicle_paths[replan_agent]
        if len(current_path) < 2:
            return vehicle_paths
        
        try:
            # 重新规划路径
            new_path_result = self.path_planner.plan_path(
                vehicle_id=replan_agent,
                start=current_path[0],
                goal=current_path[-1],
                use_backbone=True,
                context='conflict_resolution'
            )
            
            if new_path_result:
                if hasattr(new_path_result, 'path'):
                    new_path = new_path_result.path
                elif isinstance(new_path_result, tuple):
                    new_path = new_path_result[0]
                else:
                    new_path = new_path_result
                
                if new_path and len(new_path) >= 2:
                    updated_paths = vehicle_paths.copy()
                    updated_paths[replan_agent] = new_path
                    
                    # 记录统计
                    self.resolution_stats['strategies_used'][ResolutionStrategy.REPLAN] += 1
                    
                    print(f"车辆 {replan_agent} 执行重规划策略")
                    
                    return updated_paths
        
        except Exception as e:
            print(f"重规划策略执行失败: {e}")
        
        return vehicle_paths
    
    def execute_backbone_switch_strategy(self, conflict: EnhancedConflict,
                                       strategy_params: Dict, vehicle_paths: Dict) -> Dict[str, List]:
        """执行骨干路径切换策略"""
        if not self.backbone_network:
            return vehicle_paths
        
        analysis = strategy_params['conflict_analysis']
        
        # 选择切换骨干路径的车辆
        switch_agent = self._select_replan_agent(conflict, analysis)
        
        # 获取当前车辆的目标信息（简化处理）
        # 实际应该从车辆的任务信息中获取
        target_type = 'unloading'  # 假设目标类型
        target_id = 0              # 假设目标ID
        
        # 查找备选骨干路径
        current_path = vehicle_paths[switch_agent]
        alternative_paths = self.backbone_network.find_alternative_backbone_paths(
            target_type, target_id
        )
        
        if alternative_paths:
            # 选择最佳备选路径
            best_alternative = self._select_best_alternative_backbone(
                alternative_paths, current_path[0] if current_path else (0, 0, 0)
            )
            
            if best_alternative:
                # 使用备选骨干路径重新规划
                new_path_result = self.backbone_network.get_path_from_position_to_target(
                    current_path[0] if current_path else (0, 0, 0),
                    target_type,
                    target_id,
                    switch_agent
                )
                
                if new_path_result:
                    if isinstance(new_path_result, tuple):
                        new_path = new_path_result[0]
                    else:
                        new_path = new_path_result
                    
                    if new_path and len(new_path) >= 2:
                        updated_paths = vehicle_paths.copy()
                        updated_paths[switch_agent] = new_path
                        
                        # 记录统计
                        self.resolution_stats['strategies_used'][ResolutionStrategy.BACKBONE_SWITCH] += 1
                        
                        print(f"车辆 {switch_agent} 执行骨干路径切换策略")
                        
                        return updated_paths
        
        # 如果骨干路径切换失败，回退到重规划
        return self.execute_replan_strategy(conflict, strategy_params, vehicle_paths)
    
    def _select_replan_agent(self, conflict: EnhancedConflict, analysis: Dict) -> str:
        """选择需要重规划的车辆"""
        # 优先选择优先级较低的车辆
        priorities = analysis['vehicle_priorities']
        
        if priorities[conflict.agent1] < priorities[conflict.agent2]:
            return conflict.agent1
        elif priorities[conflict.agent2] < priorities[conflict.agent1]:
            return conflict.agent2
        else:
            # 优先级相同时，选择路径较短的
            path_lengths = analysis['path_lengths']
            if path_lengths[conflict.agent1] <= path_lengths[conflict.agent2]:
                return conflict.agent1
            else:
                return conflict.agent2
    
    def _select_best_alternative_backbone(self, alternatives: List, 
                                        current_position: Tuple) -> Optional[Any]:
        """选择最佳备选骨干路径"""
        if not alternatives:
            return None
        
        best_path = None
        best_score = float('inf')
        
        for alt_path in alternatives:
            # 评估备选路径的质量
            quality_score = 1.0 / max(0.1, alt_path.quality)
            load_score = alt_path.get_load_factor() * 2.0
            
            # 计算到路径起点的距离
            if alt_path.forward_path:
                distance_score = self._calculate_distance(
                    current_position, alt_path.forward_path[0]
                ) / 100.0
            else:
                distance_score = 1.0
            
            total_score = quality_score + load_score + distance_score
            
            if total_score < best_score:
                best_score = total_score
                best_path = alt_path
        
        return best_path
    
    def cleanup_expired_parking_maneuvers(self, current_time: float):
        """清理过期的停车操作"""
        expired_vehicles = []
        
        for vehicle_id, maneuver in self.active_parking_maneuvers.items():
            if current_time - maneuver.start_time > maneuver.wait_duration:
                expired_vehicles.append(vehicle_id)
        
        for vehicle_id in expired_vehicles:
            del self.active_parking_maneuvers[vehicle_id]
            print(f"车辆 {vehicle_id} 完成停车等待")
    
    def set_vehicle_priority(self, vehicle_id: str, priority: float):
        """设置车辆优先级"""
        self.vehicle_priorities[vehicle_id] = max(0.0, min(1.0, priority))
    
    def get_resolution_statistics(self) -> Dict:
        """获取冲突解决统计"""
        stats = self.resolution_stats.copy()
        
        # 计算成功率
        for strategy in ResolutionStrategy:
            if strategy in stats['strategies_used']:
                usage_count = stats['strategies_used'][strategy]
                if usage_count > 0:
                    success_rates = stats['success_rates'].get(strategy, [])
                    avg_success_rate = sum(success_rates) / len(success_rates) if success_rates else 0.8
                    stats[f'{strategy.value}_success_rate'] = avg_success_rate
        
        stats['active_parking_maneuvers'] = len(self.active_parking_maneuvers)
        stats['available_parking_spots'] = len([spot for spot in self.available_parking_spots 
                                               if self._is_parking_spot_available(spot)])
        
        return stats

class OptimizedTrafficManager:
    """增强交通管理器 - 智能冲突解决与停车避让"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # 增强冲突解决策略
        self.resolution_strategy = ConflictResolutionStrategy(
            env, path_planner, backbone_network
        )
        
        # 简化数据结构
        self.active_paths = {}           # {vehicle_id: path_info}
        self.path_reservations = {}      # 简单时空预留
        
        # 检测参数
        self.safety_distance = 7.0
        self.time_discretization = 1.0
        
        # 线程锁
        self.state_lock = threading.RLock()
        
        # 增强统计
        self.stats = {
            'total_conflicts': 0,
            'resolved_conflicts': 0,
            'detection_time': 0,
            'resolution_time': 0,
            'strategy_effectiveness': defaultdict(float),
            'backbone_switches': 0,
            'parking_maneuvers': 0
        }
        
        print("初始化增强交通管理器（智能冲突解决+停车避让）")
    
    def register_vehicle_path(self, vehicle_id: str, path: List, 
                            start_time: float = 0, speed: float = 1.0) -> bool:
        """注册车辆路径"""
        if not path or len(path) < 2:
            return False
        
        with self.state_lock:
            # 移除旧路径
            self.release_vehicle_path(vehicle_id)
            
            # 注册新路径
            path_info = {
                'path': path,
                'start_time': start_time,
                'speed': speed,
                'registered_time': time.time(),
                'backbone_path_id': None  # 将被骨干网络设置
            }
            
            self.active_paths[vehicle_id] = path_info
            
            # 添加时空预留
            self._add_spacetime_reservation(vehicle_id, path, start_time, speed)
            
            return True
    
    def _add_spacetime_reservation(self, vehicle_id: str, path: List, 
                                 start_time: float, speed: float):
        """添加时空预留"""
        current_time = start_time
        
        for i in range(len(path) - 1):
            # 计算到达时间
            segment_distance = math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            travel_time = segment_distance / speed
            
            # 时空键
            time_slot = int(current_time / self.time_discretization)
            space_key = (int(path[i][0] / 5), int(path[i][1] / 5))
            reservation_key = (space_key, time_slot)
            
            if reservation_key not in self.path_reservations:
                self.path_reservations[reservation_key] = []
            
            if vehicle_id not in self.path_reservations[reservation_key]:
                self.path_reservations[reservation_key].append(vehicle_id)
            
            current_time += travel_time
    
    def detect_all_conflicts(self) -> List[EnhancedConflict]:
        """检测所有冲突（增强版）"""
        detection_start = time.time()
        conflicts = []
        
        with self.state_lock:
            vehicle_ids = list(self.active_paths.keys())
            
            # 两两检测
            for i in range(len(vehicle_ids)):
                for j in range(i + 1, len(vehicle_ids)):
                    vehicle1, vehicle2 = vehicle_ids[i], vehicle_ids[j]
                    path1 = self.active_paths[vehicle1]['path']
                    path2 = self.active_paths[vehicle2]['path']
                    
                    # 顶点冲突
                    vertex_conflicts = self._detect_enhanced_vertex_conflicts(
                        vehicle1, path1, vehicle2, path2
                    )
                    conflicts.extend(vertex_conflicts)
                    
                    # 边冲突
                    edge_conflicts = self._detect_enhanced_edge_conflicts(
                        vehicle1, path1, vehicle2, path2
                    )
                    conflicts.extend(edge_conflicts)
                    
                    # 骨干路径拥堵冲突
                    if self.backbone_network:
                        congestion_conflicts = self._detect_backbone_congestion_conflicts(
                            vehicle1, path1, vehicle2, path2
                        )
                        conflicts.extend(congestion_conflicts)
        
        # 记录统计
        detection_time = time.time() - detection_start
        self.stats['detection_time'] += detection_time
        self.stats['total_conflicts'] += len(conflicts)
        
        return conflicts
    
    def _detect_enhanced_vertex_conflicts(self, agent1: str, path1: List, 
                                        agent2: str, path2: List) -> List[EnhancedConflict]:
        """检测增强顶点冲突"""
        conflicts = []
        min_len = min(len(path1), len(path2))
        
        for t in range(min_len):
            p1, p2 = path1[t], path2[t]
            distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
            if distance < self.safety_distance:
                # 计算冲突严重程度
                severity = 1.0 - (distance / self.safety_distance)
                
                # 分析冲突特征
                suggested_strategy = self._analyze_conflict_for_strategy(
                    agent1, agent2, (p1, p2), t, severity
                )
                
                conflict = EnhancedConflict(
                    conflict_id=f"vertex_{agent1}_{agent2}_{t}",
                    agent1=agent1,
                    agent2=agent2,
                    location=((p1[0] + p2[0])/2, (p1[1] + p2[1])/2),
                    time_step=t,
                    conflict_type=ConflictType.VERTEX,
                    severity=severity,
                    suggested_strategy=suggested_strategy
                )
                conflicts.append(conflict)
        
        return conflicts
    
    def _detect_enhanced_edge_conflicts(self, agent1: str, path1: List, 
                                      agent2: str, path2: List) -> List[EnhancedConflict]:
        """检测增强边冲突"""
        conflicts = []
        min_len = min(len(path1), len(path2)) - 1
        
        for t in range(min_len):
            # 检查位置交换
            if (self._points_close(path1[t], path2[t+1]) and 
                self._points_close(path1[t+1], path2[t])):
                
                center_x = (path1[t][0] + path2[t][0]) / 2
                center_y = (path1[t][1] + path2[t][1]) / 2
                
                # 交换冲突通常比较严重
                severity = 0.8
                
                conflict = EnhancedConflict(
                    conflict_id=f"edge_{agent1}_{agent2}_{t}",
                    agent1=agent1,
                    agent2=agent2,
                    location=(center_x, center_y),
                    time_step=t,
                    conflict_type=ConflictType.EDGE,
                    severity=severity,
                    suggested_strategy=ResolutionStrategy.WAIT
                )
                conflicts.append(conflict)
        
        return conflicts
    
    def _detect_backbone_congestion_conflicts(self, agent1: str, path1: List,
                                            agent2: str, path2: List) -> List[EnhancedConflict]:
        """检测骨干路径拥堵冲突"""
        conflicts = []
        
        # 检查两个车辆是否使用同一骨干路径
        path1_info = self.active_paths.get(agent1, {})
        path2_info = self.active_paths.get(agent2, {})
        
        backbone1_id = path1_info.get('backbone_path_id')
        backbone2_id = path2_info.get('backbone_path_id')
        
        if (backbone1_id and backbone2_id and backbone1_id == backbone2_id):
            # 检查骨干路径负载
            if backbone1_id in self.backbone_network.bidirectional_paths:
                backbone_path = self.backbone_network.bidirectional_paths[backbone1_id]
                load_factor = backbone_path.get_load_factor()
                
                if load_factor > 0.8:  # 高负载阈值
                    conflict = EnhancedConflict(
                        conflict_id=f"backbone_congestion_{agent1}_{agent2}",
                        agent1=agent1,
                        agent2=agent2,
                        location=(0, 0),  # 位置不重要
                        time_step=0,
                        conflict_type=ConflictType.BACKBONE_CONGESTION,
                        severity=load_factor,
                        suggested_strategy=ResolutionStrategy.BACKBONE_SWITCH
                    )
                    conflicts.append(conflict)
        
        return conflicts
    
    def _analyze_conflict_for_strategy(self, agent1: str, agent2: str, 
                                     positions: Tuple, time_step: int, 
                                     severity: float) -> ResolutionStrategy:
        """分析冲突并建议策略"""
        # 简化的策略选择逻辑
        if severity > 0.8:
            return ResolutionStrategy.WAIT
        elif time_step < 5:
            return ResolutionStrategy.REPLAN
        elif self.backbone_network and len(self.backbone_network.bidirectional_paths) > 1:
            return ResolutionStrategy.BACKBONE_SWITCH
        else:
            return ResolutionStrategy.REROUTE
    
    def _points_close(self, p1: Tuple, p2: Tuple) -> bool:
        """检查两点是否接近"""
        distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return distance < self.safety_distance
    
    def resolve_conflicts(self, conflicts: List[EnhancedConflict]) -> Dict[str, List]:
        """解决冲突（增强版）"""
        if not conflicts:
            return {vid: info['path'] for vid, info in self.active_paths.items()}
        
        resolution_start = time.time()
        
        # 获取当前路径
        current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
        
        # 按严重程度和建议策略排序
        sorted_conflicts = sorted(conflicts, reverse=True)
        
        resolved_count = 0
        
        for conflict in sorted_conflicts[:10]:  # 限制处理数量
            print(f"解决冲突: {conflict.conflict_id} (严重程度: {conflict.severity:.2f})")
            
            # 选择解决策略
            strategy, strategy_params = self.resolution_strategy.select_resolution_strategy(
                conflict, current_paths
            )
            
            print(f"  选择策略: {strategy.value}")
            
            # 执行策略
            success = False
            if strategy == ResolutionStrategy.WAIT:
                updated_paths = self.resolution_strategy.execute_wait_strategy(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
                if success:
                    self.stats['parking_maneuvers'] += 1
                
            elif strategy == ResolutionStrategy.REPLAN:
                updated_paths = self.resolution_strategy.execute_replan_strategy(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
                
            elif strategy == ResolutionStrategy.BACKBONE_SWITCH:
                updated_paths = self.resolution_strategy.execute_backbone_switch_strategy(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
                if success:
                    self.stats['backbone_switches'] += 1
                
            else:
                # 默认重规划
                updated_paths = self.resolution_strategy.execute_replan_strategy(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
            
            if success:
                current_paths = updated_paths
                resolved_count += 1
                
                # 更新策略效果统计
                self.stats['strategy_effectiveness'][strategy.value] += 1.0
                
                print(f"  ✅ 冲突解决成功")
            else:
                print(f"  ❌ 冲突解决失败")
        
        # 更新路径
        for vehicle_id, new_path in current_paths.items():
            if vehicle_id in self.active_paths:
                if self.active_paths[vehicle_id]['path'] != new_path:
                    self.active_paths[vehicle_id]['path'] = new_path
                    print(f"车辆 {vehicle_id} 路径已更新")
        
        # 记录统计
        resolution_time = time.time() - resolution_start
        self.stats['resolution_time'] += resolution_time
        self.stats['resolved_conflicts'] += resolved_count
        
        print(f"冲突解决完成: {resolved_count}/{len(conflicts)} 个冲突已解决")
        
        return current_paths
    
    def release_vehicle_path(self, vehicle_id: str) -> bool:
        """释放车辆路径"""
        with self.state_lock:
            if vehicle_id not in self.active_paths:
                return False
            
            # 移除时空预留
            expired_reservations = []
            for key, vehicles in self.path_reservations.items():
                if vehicle_id in vehicles:
                    vehicles.remove(vehicle_id)
                    if not vehicles:
                        expired_reservations.append(key)
            
            for key in expired_reservations:
                del self.path_reservations[key]
            
            # 移除路径
            del self.active_paths[vehicle_id]
            
            # 清理停车操作
            if vehicle_id in self.resolution_strategy.active_parking_maneuvers:
                del self.resolution_strategy.active_parking_maneuvers[vehicle_id]
            
            # 释放骨干网络资源
            if self.backbone_network:
                self.backbone_network.release_vehicle_from_path(vehicle_id)
            
            return True
    
    def update(self, time_delta: float):
        """更新管理器（增强版）"""
        current_time = time.time()
        
        # 清理过期的停车操作
        self.resolution_strategy.cleanup_expired_parking_maneuvers(current_time)
        
        # 定期检测和解决冲突
        conflicts = self.detect_all_conflicts()
        
        if conflicts:
            print(f"检测到 {len(conflicts)} 个冲突，开始智能解决...")
            self.resolve_conflicts(conflicts)
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.resolution_strategy.backbone_network = backbone_network
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        self.resolution_strategy.path_planner = path_planner
    
    def set_vehicle_priority(self, vehicle_id: str, priority: float):
        """设置车辆优先级"""
        self.resolution_strategy.set_vehicle_priority(vehicle_id, priority)
    
    def get_statistics(self) -> Dict:
        """获取统计信息（增强版）"""
        stats = self.stats.copy()
        stats['active_vehicles'] = len(self.active_paths)
        stats['active_reservations'] = len(self.path_reservations)
        
        # 解决策略统计
        resolution_stats = self.resolution_strategy.get_resolution_statistics()
        stats['resolution_strategies'] = resolution_stats
        
        # 计算策略效果率
        total_effectiveness = sum(self.stats['strategy_effectiveness'].values())
        if total_effectiveness > 0:
            for strategy, count in self.stats['strategy_effectiveness'].items():
                stats[f'{strategy}_effectiveness_rate'] = count / total_effectiveness
        
        return stats
    
    def clear_all(self):
        """清理所有数据"""
        with self.state_lock:
            self.active_paths.clear()
            self.path_reservations.clear()
            self.resolution_strategy.active_parking_maneuvers.clear()
    
    def shutdown(self):
        """关闭管理器"""
        self.clear_all()
        print("增强交通管理器已关闭")