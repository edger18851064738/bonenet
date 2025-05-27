"""
traffic_manager.py - 优化版交通管理器
实现安全矩形冲突检测、调整策略优先级、提高骨干路径稳定性
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
    SAFETY_RECTANGLE = "safety_rectangle"  # 安全矩形冲突
    INTERFACE = "interface"                # 接口冲突
    BACKBONE_CONGESTION = "backbone_congestion"  # 骨干路径拥堵

class ResolutionStrategy(Enum):
    """冲突解决策略 - 按优先级排序"""
    REPLAN = "replan"                    # 重新规划 (最高优先级)
    BACKBONE_SWITCH = "backbone_switch"  # 骨干路径切换 (中等优先级)
    WAIT = "wait"                       # 停车等待 (最低优先级)
    REROUTE = "reroute"                 # 改道 (备用)
    PRIORITY_OVERRIDE = "priority_override"  # 优先级覆盖 (特殊情况)

@dataclass
class SafetyRectangle:
    """车辆安全矩形"""
    center_x: float
    center_y: float
    width: float
    height: float
    rotation: float  # 车辆朝向角度
    
    def get_corners(self) -> List[Tuple[float, float]]:
        """获取矩形四个角点"""
        half_width = self.width / 2
        half_height = self.height / 2
        
        # 相对于中心的四个角点
        corners = [
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, half_height),
            (-half_width, half_height)
        ]
        
        # 应用旋转
        cos_r = math.cos(self.rotation)
        sin_r = math.sin(self.rotation)
        
        rotated_corners = []
        for x, y in corners:
            new_x = self.center_x + x * cos_r - y * sin_r
            new_y = self.center_y + x * sin_r + y * cos_r
            rotated_corners.append((new_x, new_y))
        
        return rotated_corners
    
    def intersects(self, other: 'SafetyRectangle') -> bool:
        """检查两个矩形是否相交 - 使用分离轴定理"""
        corners1 = self.get_corners()
        corners2 = other.get_corners()
        
        # 获取两个矩形的轴
        axes = []
        
        # 第一个矩形的轴
        for i in range(4):
            j = (i + 1) % 4
            edge = (corners1[j][0] - corners1[i][0], corners1[j][1] - corners1[i][1])
            # 垂直向量作为分离轴
            axis = (-edge[1], edge[0])
            length = math.sqrt(axis[0]**2 + axis[1]**2)
            if length > 0:
                axes.append((axis[0]/length, axis[1]/length))
        
        # 第二个矩形的轴
        for i in range(4):
            j = (i + 1) % 4
            edge = (corners2[j][0] - corners2[i][0], corners2[j][1] - corners2[i][1])
            axis = (-edge[1], edge[0])
            length = math.sqrt(axis[0]**2 + axis[1]**2)
            if length > 0:
                axes.append((axis[0]/length, axis[1]/length))
        
        # 检查每个轴上的投影
        for axis in axes:
            # 投影第一个矩形
            proj1 = [corner[0] * axis[0] + corner[1] * axis[1] for corner in corners1]
            min1, max1 = min(proj1), max(proj1)
            
            # 投影第二个矩形
            proj2 = [corner[0] * axis[0] + corner[1] * axis[1] for corner in corners2]
            min2, max2 = min(proj2), max(proj2)
            
            # 检查是否分离
            if max1 < min2 or max2 < min1:
                return False
        
        return True

@dataclass
class EnhancedConflict:
    """增强冲突表示"""
    conflict_id: str
    agent1: str
    agent2: str
    location: Tuple[float, float]
    time_step: int
    conflict_type: ConflictType = ConflictType.SAFETY_RECTANGLE
    severity: float = 1.0  # 冲突严重程度 (0-1)
    predicted_duration: int = 1  # 预测冲突持续时间步数
    suggested_strategy: ResolutionStrategy = ResolutionStrategy.REPLAN
    
    def __lt__(self, other):
        """优先级比较 - 严重程度高的优先"""
        if abs(self.severity - other.severity) > 0.1:
            return self.severity > other.severity
        return self.time_step < other.time_step

@dataclass
class VehiclePrediction:
    """车辆预测状态"""
    vehicle_id: str
    future_positions: List[Tuple[float, float, float]]  # 未来30个位置
    safety_rectangles: List[SafetyRectangle]  # 对应的安全矩形
    backbone_path_id: Optional[str] = None
    path_stability_score: float = 1.0  # 路径稳定性分数

@dataclass
class ParkingManeuver:
    """停车操作"""
    vehicle_id: str
    parking_location: Tuple[float, float]
    wait_duration: float
    start_time: float
    reason: str = "conflict_avoidance"

class ConflictResolutionStrategy:
    """优化的冲突解决策略管理器"""
    
    def __init__(self, env, path_planner=None, backbone_network=None):
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        
        # 调整策略权重 - 重规划优先
        self.strategy_weights = {
            ResolutionStrategy.REPLAN: 0.6,         # 最高优先级
            ResolutionStrategy.BACKBONE_SWITCH: 0.3, # 中等优先级
            ResolutionStrategy.WAIT: 0.1,           # 最低优先级
            ResolutionStrategy.REROUTE: 0.05         # 备用
        }
        
        # 车辆优先级
        self.vehicle_priorities = {}
        
        # 停车点管理
        self.available_parking_spots = self._initialize_parking_spots()
        self.active_parking_maneuvers = {}
        
        # 骨干路径稳定性管理
        self.vehicle_backbone_commitment = {}  # {vehicle_id: {path_id, commitment_time, switches_count}}
        self.backbone_switch_penalty = 0.3    # 频繁切换的惩罚
        
        # 统计
        self.resolution_stats = {
            'strategies_used': defaultdict(int),
            'success_rates': defaultdict(list),
            'total_conflicts_resolved': 0,
            'rectangle_conflicts_detected': 0,
            'backbone_switches_avoided': 0
        }
    
    def _initialize_parking_spots(self) -> List[Tuple]:
        """初始化可用停车点"""
        parking_spots = []
        
        # 使用环境中的停车区域
        if hasattr(self.env, 'parking_areas'):
            for area in self.env.parking_areas:
                x, y = area[0], area[1]
                # 在停车区域周围生成多个停车位
                for dx in [-8, 0, 8]:
                    for dy in [-8, 0, 8]:
                        if dx != 0 or dy != 0:
                            if self._is_point_safe_for_parking((x + dx, y + dy)):
                                parking_spots.append((x + dx, y + dy))
        
        # 如果没有停车区域，在地图边缘创建
        if not parking_spots:
            for x in range(20, self.env.width - 20, 30):
                for y in [15, self.env.height - 15]:
                    if self._is_point_safe_for_parking((x, y)):
                        parking_spots.append((x, y))
        
        return parking_spots
    
    def _is_point_safe_for_parking(self, point: Tuple) -> bool:
        """检查点是否适合停车"""
        x, y = point
        
        if x < 10 or x >= self.env.width - 10 or y < 10 or y >= self.env.height - 10:
            return False
        
        # 检查障碍物
        if hasattr(self.env, 'grid'):
            for dx in range(-5, 6):
                for dy in range(-5, 6):
                    check_x, check_y = int(x + dx), int(y + dy)
                    if (0 <= check_x < self.env.width and 
                        0 <= check_y < self.env.height and
                        self.env.grid[check_x, check_y] == 1):
                        return False
        
        return True
    
    def select_resolution_strategy(self, conflict: EnhancedConflict, 
                                 vehicle_predictions: Dict[str, VehiclePrediction]) -> Tuple[ResolutionStrategy, Dict]:
        """选择最优冲突解决策略 - 重规划优先"""
        
        # 分析冲突特征
        conflict_analysis = self._analyze_enhanced_conflict(conflict, vehicle_predictions)
        
        # 评估各种策略的适用性
        strategy_scores = {}
        
        # 1. 重规划策略评分 - 最高优先级
        strategy_scores[ResolutionStrategy.REPLAN] = self._evaluate_replan_strategy_enhanced(
            conflict, conflict_analysis, vehicle_predictions
        )
        
        # 2. 骨干路径切换策略评分 - 考虑稳定性
        if self.backbone_network:
            strategy_scores[ResolutionStrategy.BACKBONE_SWITCH] = self._evaluate_backbone_switch_with_stability(
                conflict, conflict_analysis, vehicle_predictions
            )
        else:
            strategy_scores[ResolutionStrategy.BACKBONE_SWITCH] = 0.0
        
        # 3. 等待策略评分 - 最低优先级
        strategy_scores[ResolutionStrategy.WAIT] = self._evaluate_wait_strategy_conservative(
            conflict, conflict_analysis, vehicle_predictions
        )
        
        # 4. 改道策略评分
        strategy_scores[ResolutionStrategy.REROUTE] = self._evaluate_reroute_strategy(
            conflict, conflict_analysis, vehicle_predictions
        )
        
        # 选择得分最高的策略
        best_strategy = max(strategy_scores, key=strategy_scores.get)
        best_score = strategy_scores[best_strategy]
        
        strategy_params = {
            'score': best_score,
            'conflict_analysis': conflict_analysis,
            'alternative_scores': strategy_scores,
            'vehicle_predictions': vehicle_predictions
        }
        
        return best_strategy, strategy_params
    
    def _analyze_enhanced_conflict(self, conflict: EnhancedConflict, 
                                 vehicle_predictions: Dict[str, VehiclePrediction]) -> Dict:
        """增强冲突特征分析"""
        pred1 = vehicle_predictions.get(conflict.agent1)
        pred2 = vehicle_predictions.get(conflict.agent2)
        
        analysis = {
            'conflict_duration': conflict.predicted_duration,
            'prediction_reliability': 1.0,
            'vehicle_priorities': {
                conflict.agent1: self.vehicle_priorities.get(conflict.agent1, 0.5),
                conflict.agent2: self.vehicle_priorities.get(conflict.agent2, 0.5)
            },
            'path_stability_scores': {},
            'backbone_commitment_status': {},
            'available_alternatives': 0
        }
        
        # 路径稳定性分析
        if pred1:
            analysis['path_stability_scores'][conflict.agent1] = pred1.path_stability_score
            if pred1.backbone_path_id:
                commitment = self.vehicle_backbone_commitment.get(conflict.agent1, {})
                analysis['backbone_commitment_status'][conflict.agent1] = {
                    'path_id': pred1.backbone_path_id,
                    'switches_count': commitment.get('switches_count', 0),
                    'commitment_time': commitment.get('commitment_time', 0)
                }
        
        if pred2:
            analysis['path_stability_scores'][conflict.agent2] = pred2.path_stability_score
            if pred2.backbone_path_id:
                commitment = self.vehicle_backbone_commitment.get(conflict.agent2, {})
                analysis['backbone_commitment_status'][conflict.agent2] = {
                    'path_id': pred2.backbone_path_id,
                    'switches_count': commitment.get('switches_count', 0),
                    'commitment_time': commitment.get('commitment_time', 0)
                }
        
        # 检查可用的备选方案数量
        if self.backbone_network:
            analysis['available_alternatives'] = len(
                self.backbone_network.bidirectional_paths
            )
        
        return analysis
    
    def _evaluate_replan_strategy_enhanced(self, conflict: EnhancedConflict, 
                                         analysis: Dict, vehicle_predictions: Dict) -> float:
        """增强的重规划策略评分 - 最高优先级"""
        base_score = 0.85  # 提高基础分数
        
        # 规划器可用性
        planner_factor = 1.0 if self.path_planner else 0.0
        
        # 冲突严重程度：严重冲突更需要重规划
        severity_factor = conflict.severity
        
        # 预测持续时间：长时间冲突更适合重规划
        duration_factor = min(1.0, conflict.predicted_duration / 10.0)
        
        # 路径稳定性：不稳定的路径更适合重规划
        avg_stability = sum(analysis['path_stability_scores'].values()) / max(1, len(analysis['path_stability_scores']))
        instability_factor = 1.0 - avg_stability
        
        final_score = base_score * (0.3 * planner_factor + 
                                  0.3 * severity_factor + 
                                  0.2 * duration_factor +
                                  0.2 * instability_factor)
        
        return final_score
    
    def _evaluate_backbone_switch_with_stability(self, conflict: EnhancedConflict, 
                                                analysis: Dict, vehicle_predictions: Dict) -> float:
        """考虑稳定性的骨干路径切换评分"""
        base_score = 0.65  # 中等基础分数
        
        # 备选路径因子
        alternatives_count = analysis['available_alternatives']
        alternatives_factor = min(1.0, alternatives_count / 3.0)
        
        # 骨干路径稳定性惩罚
        stability_penalty = 0.0
        for agent in [conflict.agent1, conflict.agent2]:
            commitment_status = analysis['backbone_commitment_status'].get(agent, {})
            switches_count = commitment_status.get('switches_count', 0)
            
            # 频繁切换的惩罚
            if switches_count > 2:
                stability_penalty += (switches_count - 2) * self.backbone_switch_penalty
        
        stability_factor = max(0.1, 1.0 - stability_penalty)
        
        # 当前骨干路径负载
        congestion_factor = 0.7  # 假设值，实际应该从骨干网络获取
        
        final_score = base_score * (0.4 * alternatives_factor + 
                                  0.4 * stability_factor + 
                                  0.2 * (1.0 - congestion_factor))
        
        return final_score
    
    def _evaluate_wait_strategy_conservative(self, conflict: EnhancedConflict, 
                                           analysis: Dict, vehicle_predictions: Dict) -> float:
        """保守的等待策略评分 - 最低优先级"""
        base_score = 0.4  # 降低基础分数
        
        # 优先级因子：只有在明显优先级差异时才考虑等待
        priorities = analysis['vehicle_priorities']
        priority_diff = abs(priorities[conflict.agent1] - priorities[conflict.agent2])
        
        # 只有当优先级差异明显时才考虑等待
        if priority_diff < 0.3:
            return 0.1  # 优先级相近时，等待策略分数很低
        
        priority_factor = priority_diff
        
        # 停车点可用性
        available_spots = len([spot for spot in self.available_parking_spots 
                              if self._is_parking_spot_available(spot)])
        availability_factor = min(1.0, available_spots / 3.0)
        
        # 冲突持续时间：短时冲突更适合等待
        if conflict.predicted_duration > 5:
            duration_penalty = 0.5
        else:
            duration_penalty = 1.0
        
        final_score = base_score * (0.5 * priority_factor + 
                                  0.3 * availability_factor + 
                                  0.2 * duration_penalty)
        
        return final_score
    
    def _evaluate_reroute_strategy(self, conflict: EnhancedConflict, 
                                 analysis: Dict, vehicle_predictions: Dict) -> float:
        """评估改道策略的适用性"""
        base_score = 0.5
        
        # 备选路径因子
        alternative_factor = min(1.0, analysis['available_alternatives'] / 2.0)
        
        # 冲突位置因子：在路径中段的冲突更适合改道
        position_factor = 0.8 if conflict.time_step > 3 else 0.3
        
        final_score = base_score * (0.7 * alternative_factor + 0.3 * position_factor)
        
        return final_score
    
    def _is_parking_spot_available(self, spot: Tuple) -> bool:
        """检查停车点是否可用"""
        for maneuver in self.active_parking_maneuvers.values():
            if self._calculate_distance(spot, maneuver.parking_location) < 10.0:
                return False
        return True
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def execute_replan_strategy_enhanced(self, conflict: EnhancedConflict, 
                                       strategy_params: Dict, vehicle_paths: Dict) -> Dict[str, List]:
        """执行增强的重规划策略"""
        if not self.path_planner:
            return vehicle_paths
        
        analysis = strategy_params['conflict_analysis']
        
        # 智能选择重规划的车辆
        replan_agent = self._select_replan_agent_smart(conflict, analysis)
        
        current_path = vehicle_paths[replan_agent]
        if len(current_path) < 2:
            return vehicle_paths
        
        try:
            # 增强的重规划调用
            new_path_result = self.path_planner.plan_path(
                vehicle_id=replan_agent,
                start=current_path[0],
                goal=current_path[-1],
                use_backbone=True,
                context='conflict_resolution',
                quality_threshold=0.7  # 要求更高质量
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
                    
                    # 更新骨干路径承诺
                    self._update_backbone_commitment(replan_agent, new_path_result)
                    
                    # 记录统计
                    self.resolution_stats['strategies_used'][ResolutionStrategy.REPLAN] += 1
                    
                    print(f"车辆 {replan_agent} 执行增强重规划策略")
                    
                    return updated_paths
        
        except Exception as e:
            print(f"增强重规划策略执行失败: {e}")
        
        return vehicle_paths
    
    def execute_backbone_switch_with_stability(self, conflict: EnhancedConflict,
                                             strategy_params: Dict, vehicle_paths: Dict) -> Dict[str, List]:
        """执行考虑稳定性的骨干路径切换策略"""
        if not self.backbone_network:
            return vehicle_paths
        
        analysis = strategy_params['conflict_analysis']
        
        # 选择切换车辆 - 考虑切换历史
        switch_agent = self._select_switch_agent_with_stability(conflict, analysis)
        
        # 检查是否应该避免切换
        commitment = self.vehicle_backbone_commitment.get(switch_agent, {})
        if commitment.get('switches_count', 0) >= 3:
            self.resolution_stats['backbone_switches_avoided'] += 1
            print(f"避免车辆 {switch_agent} 频繁切换骨干路径")
            # 回退到重规划
            return self.execute_replan_strategy_enhanced(conflict, strategy_params, vehicle_paths)
        
        # 执行切换
        current_path = vehicle_paths[switch_agent]
        if not current_path:
            return vehicle_paths
        
        try:
            # 获取目标信息（简化处理）
            target_type = 'unloading'
            target_id = 0
            
            # 查找稳定的备选骨干路径
            alternative_paths = self.backbone_network.find_alternative_backbone_paths(
                target_type, target_id, exclude_path_id=commitment.get('path_id')
            )
            
            if alternative_paths:
                # 选择负载最低且稳定的备选路径
                best_alternative = min(alternative_paths, 
                                     key=lambda p: p.get_load_factor())
                
                if best_alternative:
                    # 使用备选骨干路径重新规划
                    new_path_result = self.backbone_network.get_path_from_position_to_target(
                        current_path[0],
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
                            
                            # 更新切换统计
                            self._record_backbone_switch(switch_agent, best_alternative.path_id)
                            
                            self.resolution_stats['strategies_used'][ResolutionStrategy.BACKBONE_SWITCH] += 1
                            
                            print(f"车辆 {switch_agent} 执行稳定性骨干路径切换")
                            
                            return updated_paths
        
        except Exception as e:
            print(f"骨干路径切换失败: {e}")
        
        # 切换失败，回退到重规划
        return self.execute_replan_strategy_enhanced(conflict, strategy_params, vehicle_paths)
    
    def _select_replan_agent_smart(self, conflict: EnhancedConflict, analysis: Dict) -> str:
        """智能选择重规划车辆"""
        priorities = analysis['vehicle_priorities']
        stability_scores = analysis['path_stability_scores']
        
        # 综合考虑优先级和路径稳定性
        agent1_score = priorities[conflict.agent1] + stability_scores.get(conflict.agent1, 0.5)
        agent2_score = priorities[conflict.agent2] + stability_scores.get(conflict.agent2, 0.5)
        
        # 选择综合分数较低的车辆进行重规划
        if agent1_score <= agent2_score:
            return conflict.agent1
        else:
            return conflict.agent2
    
    def _select_switch_agent_with_stability(self, conflict: EnhancedConflict, analysis: Dict) -> str:
        """考虑稳定性选择切换车辆"""
        # 优先选择切换次数较少的车辆
        agent1_switches = analysis['backbone_commitment_status'].get(conflict.agent1, {}).get('switches_count', 0)
        agent2_switches = analysis['backbone_commitment_status'].get(conflict.agent2, {}).get('switches_count', 0)
        
        if agent1_switches < agent2_switches:
            return conflict.agent1
        elif agent2_switches < agent1_switches:
            return conflict.agent2
        else:
            # 切换次数相同时，选择优先级较低的
            priorities = analysis['vehicle_priorities']
            if priorities[conflict.agent1] <= priorities[conflict.agent2]:
                return conflict.agent1
            else:
                return conflict.agent2
    
    def _update_backbone_commitment(self, vehicle_id: str, path_result: Any):
        """更新车辆骨干路径承诺"""
        if hasattr(path_result, 'structure') and path_result.structure:
            path_id = path_result.structure.get('path_id')
            if path_id:
                current_commitment = self.vehicle_backbone_commitment.get(vehicle_id, {})
                
                if current_commitment.get('path_id') != path_id:
                    # 路径发生变化，记录切换
                    self.vehicle_backbone_commitment[vehicle_id] = {
                        'path_id': path_id,
                        'commitment_time': time.time(),
                        'switches_count': current_commitment.get('switches_count', 0) + 1
                    }
                else:
                    # 路径保持不变，延长承诺时间
                    current_commitment['commitment_time'] = time.time()
    
    def _record_backbone_switch(self, vehicle_id: str, new_path_id: str):
        """记录骨干路径切换"""
        current_commitment = self.vehicle_backbone_commitment.get(vehicle_id, {})
        
        self.vehicle_backbone_commitment[vehicle_id] = {
            'path_id': new_path_id,
            'commitment_time': time.time(),
            'switches_count': current_commitment.get('switches_count', 0) + 1
        }
    
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
                    avg_success_rate = sum(success_rates) / len(success_rates) if success_rates else 0.85
                    stats[f'{strategy.value}_success_rate'] = avg_success_rate
        
        stats['active_parking_maneuvers'] = len(self.active_parking_maneuvers)
        stats['available_parking_spots'] = len([spot for spot in self.available_parking_spots 
                                               if self._is_parking_spot_available(spot)])
        stats['backbone_stability_maintained'] = stats['backbone_switches_avoided']
        
        return stats

class OptimizedTrafficManager:
    """优化交通管理器 - 安全矩形冲突检测与策略优化"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # 优化的冲突解决策略
        self.resolution_strategy = ConflictResolutionStrategy(
            env, path_planner, backbone_network
        )
        
        # 车辆预测管理
        self.vehicle_predictions = {}  # {vehicle_id: VehiclePrediction}
        self.prediction_horizon = 30   # 预测30个步骤
        
        # 路径管理
        self.active_paths = {}
        self.path_reservations = {}
        
        # 车辆参数
        self.vehicle_params = {
            'length': 6.0,
            'width': 3.0,
            'safety_margin': 1.5  # 安全边距
        }
        
        # 检测参数
        self.safety_distance = 8.0
        self.time_discretization = 1.0
        
        # 线程锁
        self.state_lock = threading.RLock()
        
        # 增强统计
        self.stats = {
            'total_conflicts': 0,
            'resolved_conflicts': 0,
            'rectangle_conflicts_detected': 0,
            'prediction_accuracy': 0.0,
            'strategy_distribution': defaultdict(int),
            'backbone_stability_score': 1.0,
            'detection_time': 0,
            'resolution_time': 0
        }
        
        print("初始化优化交通管理器（安全矩形冲突检测+策略优化）")
    
    def register_vehicle_path(self, vehicle_id: str, path: List, 
                            start_time: float = 0, speed: float = 1.0) -> bool:
        """注册车辆路径并生成预测"""
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
                'backbone_path_id': None
            }
            
            self.active_paths[vehicle_id] = path_info
            
            # 生成车辆预测
            self._generate_vehicle_prediction(vehicle_id, path, start_time, speed)
            
            return True
    
    def _generate_vehicle_prediction(self, vehicle_id: str, path: List, 
                                   start_time: float, speed: float):
        """生成车辆未来位置预测和安全矩形"""
        if len(path) < 2:
            return
        
        future_positions = []
        safety_rectangles = []
        current_time = start_time
        
        # 预测未来30个位置点
        path_index = 0
        for step in range(self.prediction_horizon):
            if path_index >= len(path) - 1:
                # 如果路径结束，使用最后位置
                position = path[-1]
            else:
                # 根据速度和时间计算位置
                segment_progress = (current_time - start_time) * speed
                
                # 找到当前应该在哪个路径段
                accumulated_distance = 0
                for i in range(len(path) - 1):
                    segment_length = self._calculate_distance(path[i], path[i + 1])
                    if accumulated_distance + segment_length >= segment_progress:
                        # 在这个段内插值
                        local_progress = (segment_progress - accumulated_distance) / segment_length
                        position = self._interpolate_path_position(path[i], path[i + 1], local_progress)
                        break
                    accumulated_distance += segment_length
                else:
                    position = path[-1]
            
            future_positions.append(position)
            
            # 为每个位置创建安全矩形
            safety_rect = self._create_safety_rectangle(
                position, vehicle_id
            )
            safety_rectangles.append(safety_rect)
            
            current_time += self.time_discretization
        
        # 计算路径稳定性分数
        stability_score = self._calculate_path_stability(vehicle_id, path)
        
        # 创建预测对象
        prediction = VehiclePrediction(
            vehicle_id=vehicle_id,
            future_positions=future_positions,
            safety_rectangles=safety_rectangles,
            backbone_path_id=self._extract_backbone_path_id(path),
            path_stability_score=stability_score
        )
        
        self.vehicle_predictions[vehicle_id] = prediction
    
    def _create_safety_rectangle(self, position: Tuple, vehicle_id: str) -> SafetyRectangle:
        """为车辆位置创建安全矩形"""
        x, y = position[0], position[1]
        theta = position[2] if len(position) > 2 else 0.0
        
        # 车辆尺寸加上安全边距
        safe_width = self.vehicle_params['width'] + self.vehicle_params['safety_margin']
        safe_length = self.vehicle_params['length'] + self.vehicle_params['safety_margin']
        
        return SafetyRectangle(
            center_x=x,
            center_y=y,
            width=safe_width,
            height=safe_length,
            rotation=theta
        )
    
    def _calculate_path_stability(self, vehicle_id: str, path: List) -> float:
        """计算路径稳定性分数"""
        # 检查车辆的骨干路径切换历史
        commitment = self.resolution_strategy.vehicle_backbone_commitment.get(vehicle_id, {})
        switches_count = commitment.get('switches_count', 0)
        
        # 切换次数越少，稳定性越高
        stability_score = max(0.1, 1.0 - (switches_count * 0.2))
        
        return stability_score
    
    def _extract_backbone_path_id(self, path: List) -> Optional[str]:
        """从路径中提取骨干路径ID（简化实现）"""
        # 这里应该根据实际路径结构提取
        # 简化处理，返回None
        return None
    
    def _interpolate_path_position(self, pos1: Tuple, pos2: Tuple, progress: float) -> Tuple:
        """在路径两点间插值"""
        x = pos1[0] + progress * (pos2[0] - pos1[0])
        y = pos1[1] + progress * (pos2[1] - pos1[1])
        theta = pos1[2] if len(pos1) > 2 else 0.0
        if len(pos2) > 2:
            theta = theta + progress * (pos2[2] - theta)
        
        return (x, y, theta)
    
    def detect_all_conflicts(self) -> List[EnhancedConflict]:
        """检测所有冲突 - 使用安全矩形方法"""
        detection_start = time.time()
        conflicts = []
        
        with self.state_lock:
            vehicle_ids = list(self.vehicle_predictions.keys())
            
            # 两两检测安全矩形冲突
            for i in range(len(vehicle_ids)):
                for j in range(i + 1, len(vehicle_ids)):
                    vehicle1, vehicle2 = vehicle_ids[i], vehicle_ids[j]
                    
                    # 检测安全矩形冲突
                    rectangle_conflicts = self._detect_safety_rectangle_conflicts(
                        vehicle1, vehicle2
                    )
                    conflicts.extend(rectangle_conflicts)
                    
                    # 检测骨干路径拥堵冲突
                    if self.backbone_network:
                        congestion_conflicts = self._detect_backbone_congestion_conflicts(
                            vehicle1, vehicle2
                        )
                        conflicts.extend(congestion_conflicts)
        
        # 记录统计
        detection_time = time.time() - detection_start
        self.stats['detection_time'] += detection_time
        self.stats['total_conflicts'] += len(conflicts)
        self.stats['rectangle_conflicts_detected'] += len([c for c in conflicts 
                                                          if c.conflict_type == ConflictType.SAFETY_RECTANGLE])
        
        return conflicts
    
    def _detect_safety_rectangle_conflicts(self, agent1: str, agent2: str) -> List[EnhancedConflict]:
        """检测安全矩形冲突"""
        conflicts = []
        
        pred1 = self.vehicle_predictions.get(agent1)
        pred2 = self.vehicle_predictions.get(agent2)
        
        if not pred1 or not pred2:
            return conflicts
        
        # 检查每个时间步的矩形重叠
        min_steps = min(len(pred1.safety_rectangles), len(pred2.safety_rectangles))
        
        conflict_start = None
        conflict_duration = 0
        
        for t in range(min_steps):
            rect1 = pred1.safety_rectangles[t]
            rect2 = pred2.safety_rectangles[t]
            
            if rect1.intersects(rect2):
                if conflict_start is None:
                    conflict_start = t
                conflict_duration += 1
                
                # 计算冲突严重程度
                center_distance = math.sqrt(
                    (rect1.center_x - rect2.center_x)**2 + 
                    (rect1.center_y - rect2.center_y)**2
                )
                
                # 基于距离和矩形重叠面积计算严重程度
                max_safe_distance = max(rect1.width, rect1.height, rect2.width, rect2.height)
                severity = min(1.0, max(0.1, 1.0 - (center_distance / max_safe_distance)))
                
            else:
                # 矩形不再重叠，如果之前有冲突，创建冲突对象
                if conflict_start is not None:
                    conflict = self._create_rectangle_conflict(
                        agent1, agent2, conflict_start, conflict_duration, severity
                    )
                    conflicts.append(conflict)
                    
                    conflict_start = None
                    conflict_duration = 0
        
        # 处理持续到预测结束的冲突
        if conflict_start is not None:
            conflict = self._create_rectangle_conflict(
                agent1, agent2, conflict_start, conflict_duration, severity
            )
            conflicts.append(conflict)
        
        return conflicts
    
    def _create_rectangle_conflict(self, agent1: str, agent2: str, 
                                 time_step: int, duration: int, severity: float) -> EnhancedConflict:
        """创建安全矩形冲突对象"""
        pred1 = self.vehicle_predictions[agent1]
        pred2 = self.vehicle_predictions[agent2]
        
        # 冲突位置为两车辆预测位置的中点
        pos1 = pred1.future_positions[time_step]
        pos2 = pred2.future_positions[time_step]
        
        conflict_location = (
            (pos1[0] + pos2[0]) / 2,
            (pos1[1] + pos2[1]) / 2
        )
        
        conflict = EnhancedConflict(
            conflict_id=f"rect_{agent1}_{agent2}_{time_step}",
            agent1=agent1,
            agent2=agent2,
            location=conflict_location,
            time_step=time_step,
            conflict_type=ConflictType.SAFETY_RECTANGLE,
            severity=severity,
            predicted_duration=duration,
            suggested_strategy=ResolutionStrategy.REPLAN  # 优先重规划
        )
        
        return conflict
    
    def _detect_backbone_congestion_conflicts(self, agent1: str, agent2: str) -> List[EnhancedConflict]:
        """检测骨干路径拥堵冲突"""
        conflicts = []
        
        pred1 = self.vehicle_predictions.get(agent1)
        pred2 = self.vehicle_predictions.get(agent2)
        
        if not pred1 or not pred2:
            return conflicts
        
        # 检查两个车辆是否使用同一骨干路径
        if (pred1.backbone_path_id and pred2.backbone_path_id and 
            pred1.backbone_path_id == pred2.backbone_path_id):
            
            # 检查骨干路径负载
            if pred1.backbone_path_id in self.backbone_network.bidirectional_paths:
                backbone_path = self.backbone_network.bidirectional_paths[pred1.backbone_path_id]
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
                        predicted_duration=5,
                        suggested_strategy=ResolutionStrategy.BACKBONE_SWITCH
                    )
                    conflicts.append(conflict)
        
        return conflicts
    
    def resolve_conflicts(self, conflicts: List[EnhancedConflict]) -> Dict[str, List]:
        """解决冲突 - 使用优化策略"""
        if not conflicts:
            return {vid: info['path'] for vid, info in self.active_paths.items()}
        
        resolution_start = time.time()
        
        # 获取当前路径
        current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
        
        # 按严重程度排序
        sorted_conflicts = sorted(conflicts, reverse=True)
        
        resolved_count = 0
        
        for conflict in sorted_conflicts[:8]:  # 限制处理数量
            print(f"解决冲突: {conflict.conflict_id} "
                  f"(类型: {conflict.conflict_type.value}, 严重程度: {conflict.severity:.2f})")
            
            # 选择解决策略
            strategy, strategy_params = self.resolution_strategy.select_resolution_strategy(
                conflict, self.vehicle_predictions
            )
            
            print(f"  选择策略: {strategy.value} (分数: {strategy_params['score']:.2f})")
            
            # 执行策略
            success = False
            if strategy == ResolutionStrategy.REPLAN:
                updated_paths = self.resolution_strategy.execute_replan_strategy_enhanced(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
                
            elif strategy == ResolutionStrategy.BACKBONE_SWITCH:
                updated_paths = self.resolution_strategy.execute_backbone_switch_with_stability(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
                
            elif strategy == ResolutionStrategy.WAIT:
                # 使用原有的等待策略实现
                updated_paths = self._execute_wait_strategy_fallback(
                    conflict, strategy_params, current_paths
                )
                success = updated_paths != current_paths
            
            else:
                # 其他策略的简化实现
                updated_paths = current_paths
                success = False
            
            if success:
                current_paths = updated_paths
                resolved_count += 1
                
                # 更新预测
                self._update_predictions_after_resolution(conflict, updated_paths)
                
                # 记录策略使用统计
                self.stats['strategy_distribution'][strategy.value] += 1
                
                print(f"  ✅ 冲突解决成功")
            else:
                print(f"  ❌ 冲突解决失败")
        
        # 更新路径
        for vehicle_id, new_path in current_paths.items():
            if vehicle_id in self.active_paths:
                if self.active_paths[vehicle_id]['path'] != new_path:
                    self.active_paths[vehicle_id]['path'] = new_path
                    # 重新生成预测
                    self._generate_vehicle_prediction(
                        vehicle_id, new_path, 
                        self.active_paths[vehicle_id]['start_time'],
                        self.active_paths[vehicle_id]['speed']
                    )
                    print(f"车辆 {vehicle_id} 路径已更新并重新预测")
        
        # 记录统计
        resolution_time = time.time() - resolution_start
        self.stats['resolution_time'] += resolution_time
        self.stats['resolved_conflicts'] += resolved_count
        
        # 更新骨干稳定性分数
        self._update_backbone_stability_score()
        
        print(f"冲突解决完成: {resolved_count}/{len(conflicts)} 个冲突已解决, 耗时: {resolution_time:.2f}s")
        
        return current_paths
    
    def _execute_wait_strategy_fallback(self, conflict: EnhancedConflict,
                                      strategy_params: Dict, vehicle_paths: Dict) -> Dict[str, List]:
        """回退的等待策略实现"""
        # 简化的等待策略，让优先级低的车辆减速
        analysis = strategy_params['conflict_analysis']
        priorities = analysis['vehicle_priorities']
        
        if priorities[conflict.agent1] < priorities[conflict.agent2]:
            wait_agent = conflict.agent1
        else:
            wait_agent = conflict.agent2
        
        # 简单的减速处理 - 在路径中插入额外点
        current_path = vehicle_paths[wait_agent]
        if len(current_path) > 2:
            # 在路径开始部分插入减速点
            new_path = []
            for i in range(min(5, len(current_path))):
                new_path.append(current_path[i])
                if i < len(current_path) - 1:
                    # 插入中间点以减速
                    mid_point = self._interpolate_path_position(
                        current_path[i], current_path[i + 1], 0.5
                    )
                    new_path.append(mid_point)
            
            new_path.extend(current_path[5:])
            
            updated_paths = vehicle_paths.copy()
            updated_paths[wait_agent] = new_path
            
            print(f"车辆 {wait_agent} 执行减速等待策略")
            return updated_paths
        
        return vehicle_paths
    
    def _update_predictions_after_resolution(self, conflict: EnhancedConflict, updated_paths: Dict):
        """冲突解决后更新预测"""
        # 重新生成涉及冲突车辆的预测
        for agent in [conflict.agent1, conflict.agent2]:
            if agent in updated_paths and agent in self.active_paths:
                path_info = self.active_paths[agent]
                self._generate_vehicle_prediction(
                    agent, updated_paths[agent],
                    path_info['start_time'], path_info['speed']
                )
    
    def _update_backbone_stability_score(self):
        """更新骨干稳定性分数"""
        total_vehicles = len(self.resolution_strategy.vehicle_backbone_commitment)
        if total_vehicles == 0:
            self.stats['backbone_stability_score'] = 1.0
            return
        
        stable_vehicles = 0
        for commitment in self.resolution_strategy.vehicle_backbone_commitment.values():
            if commitment.get('switches_count', 0) <= 2:
                stable_vehicles += 1
        
        self.stats['backbone_stability_score'] = stable_vehicles / total_vehicles
    
    def release_vehicle_path(self, vehicle_id: str) -> bool:
        """释放车辆路径"""
        with self.state_lock:
            if vehicle_id not in self.active_paths:
                return False
            
            # 移除路径
            del self.active_paths[vehicle_id]
            
            # 移除预测
            if vehicle_id in self.vehicle_predictions:
                del self.vehicle_predictions[vehicle_id]
            
            # 清理停车操作
            if vehicle_id in self.resolution_strategy.active_parking_maneuvers:
                del self.resolution_strategy.active_parking_maneuvers[vehicle_id]
            
            # 释放骨干网络资源
            if self.backbone_network:
                self.backbone_network.release_vehicle_from_path(vehicle_id)
            
            return True
    
    def update(self, time_delta: float):
        """更新管理器"""
        current_time = time.time()
        
        # 清理过期的停车操作
        self.resolution_strategy.cleanup_expired_parking_maneuvers(current_time)
        
        # 更新车辆预测
        self._update_vehicle_predictions(time_delta)
        
        # 定期检测和解决冲突
        conflicts = self.detect_all_conflicts()
        
        if conflicts:
            print(f"检测到 {len(conflicts)} 个安全矩形冲突，开始优化解决...")
            self.resolve_conflicts(conflicts)
    
    def _update_vehicle_predictions(self, time_delta: float):
        """更新车辆预测"""
        for vehicle_id in list(self.vehicle_predictions.keys()):
            if vehicle_id in self.active_paths:
                path_info = self.active_paths[vehicle_id]
                # 重新生成预测（简化处理）
                self._generate_vehicle_prediction(
                    vehicle_id, path_info['path'],
                    path_info['start_time'], path_info['speed']
                )
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
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
        """获取统计信息"""
        stats = self.stats.copy()
        stats['active_vehicles'] = len(self.active_paths)
        stats['active_predictions'] = len(self.vehicle_predictions)
        
        # 解决策略统计
        resolution_stats = self.resolution_strategy.get_resolution_statistics()
        stats['resolution_strategies'] = resolution_stats
        
        # 预测准确性（简化计算）
        stats['prediction_horizon'] = self.prediction_horizon
        
        return stats
    
    def clear_all(self):
        """清理所有数据"""
        with self.state_lock:
            self.active_paths.clear()
            self.vehicle_predictions.clear()
            self.path_reservations.clear()
            self.resolution_strategy.active_parking_maneuvers.clear()
    
    def shutdown(self):
        """关闭管理器"""
        self.clear_all()
        print("优化交通管理器已关闭")