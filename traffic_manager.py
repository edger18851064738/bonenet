"""
traffic_manager.py - 完整优化的ECBS集成交通管理器
集成了骨干路径感知、节点级时间窗口管理、智能路径切换和停车等待决策
"""

import math
import time
import threading
import heapq
from typing import List, Dict, Tuple, Optional, Any, Set
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass, field
from enum import Enum
import copy
from sortedcontainers import SortedList

class ConflictType(Enum):
    """冲突类型"""
    SAFETY_RECTANGLE = "safety_rectangle"
    INTERFACE = "interface"
    BACKBONE_CONGESTION = "backbone_congestion"
    TEMPORAL = "temporal"
    CAPACITY = "capacity"
    NODE_OCCUPATION = "node_occupation"  # 新增：节点占用冲突
    NARROW_PASSAGE = "narrow_passage"    # 新增：狭窄通道冲突
    ACCESS_POINT = "access_point"        # 新增：接入点冲突

class ConstraintType(Enum):
    """约束类型"""
    VERTEX = "vertex"
    EDGE = "edge"
    BACKBONE = "backbone"
    CAPACITY = "capacity"
    NODE = "node"              # 新增：节点约束
    PATH_SWITCH = "path_switch" # 新增：路径切换约束
    PARKING = "parking"        # 新增：停车约束

@dataclass
class ECBSConstraint:
    """ECBS约束"""
    constraint_type: ConstraintType
    agent: Optional[str] = None
    position: Optional[Tuple] = None
    position2: Optional[Tuple] = None
    time: Optional[int] = None
    time_window: Optional[Tuple[int, int]] = None
    backbone_id: Optional[str] = None
    max_agents: Optional[int] = None
    node_id: Optional[str] = None  # 新增：节点ID
    
    def __hash__(self):
        return hash((self.constraint_type, self.agent, self.position, 
                    self.time, self.backbone_id, self.node_id))

@dataclass
class BackboneNodeConstraint(ECBSConstraint):
    """骨干路径节点约束"""
    node_id: str = None
    backbone_path_id: str = None
    time_window: Tuple[float, float] = None
    
    def __post_init__(self):
        self.constraint_type = ConstraintType.NODE
        # 验证必需字段
        if self.node_id is None or self.backbone_path_id is None or self.time_window is None:
            raise ValueError("BackboneNodeConstraint requires node_id, backbone_path_id and time_window")
    
    def applies_to_path(self, path: List[Tuple], path_structure: Dict) -> bool:
        """检查约束是否适用于给定路径"""
        if path_structure.get('path_id') != self.backbone_path_id:
            return False
            
        # 检查路径是否经过该节点
        for i, point in enumerate(path):
            if self._is_node_position(point, self.node_id):
                # 计算到达该节点的时间
                arrival_time = self._calculate_arrival_time(path, i)
                # 检查是否在约束时间窗口内
                return (self.time_window[0] <= arrival_time <= self.time_window[1])
        return False
    
    def _is_node_position(self, point: Tuple, node_id: str) -> bool:
        """检查点是否为指定节点"""
        # 这里需要从backbone_network获取节点位置信息
        # 简化实现
        return False
    
    def _calculate_arrival_time(self, path: List[Tuple], index: int) -> float:
        """计算到达指定索引的时间"""
        # 简化实现：假设匀速运动
        return index * 1.0

@dataclass
class ParkingDecision:
    """停车决策"""
    vehicle_id: str
    parking_position: Tuple[float, float, float]
    parking_duration: float
    start_time: float
    reason: str
    priority_vehicle_id: Optional[str] = None
    expected_resolution_time: Optional[float] = None

@dataclass
class ECBSConflict:
    """ECBS冲突"""
    conflict_id: str
    conflict_type: ConflictType
    agents: List[str]
    location: Tuple[float, float]
    time: int
    severity: float = 1.0
    backbone_id: Optional[str] = None
    node_id: Optional[str] = None  # 新增：节点ID
    
    def __lt__(self, other):
        return self.severity > other.severity

class ECBSNode:
    """ECBS搜索树节点"""
    
    def __init__(self):
        self.constraints: Set[ECBSConstraint] = set()
        self.solution: Dict[str, List[Tuple]] = {}
        self.cost: float = 0.0
        self.conflicts: List[ECBSConflict] = []
        self.h_value: float = 0.0
        self.f_value: float = 0.0
        self.parking_decisions: Dict[str, ParkingDecision] = {}  # 新增：停车决策
        self.path_switches: Dict[str, str] = {}  # 新增：路径切换记录
    
    def __lt__(self, other):
        if abs(self.f_value - other.f_value) < 1e-6:
            return len(self.conflicts) < len(other.conflicts)
        return self.f_value < other.f_value


class NodeTimeWindowManager:
    """节点级时间窗口管理器"""
    
    def __init__(self, backbone_network):
        self.backbone_network = backbone_network
        self.node_reservations = defaultdict(SortedList)  # {node_id: SortedList[(start_time, end_time, vehicle_id)]}
        self.node_safety_margin = 5.0  # 节点安全时间间隔
        self.node_traversal_time = 3.0  # 默认节点通过时间
        
    def check_node_availability(self, node_id: str, vehicle_id: str, 
                               arrival_time: float, duration: float) -> bool:
        """检查节点在指定时间窗口是否可用"""
        if node_id not in self.node_reservations:
            return True
            
        # 检查与现有预约的冲突
        for start, end, occupant in self.node_reservations[node_id]:
            if occupant == vehicle_id:
                continue
            # 添加安全边际的时间窗口检查
            if not (arrival_time + duration + self.node_safety_margin <= start or 
                    arrival_time >= end + self.node_safety_margin):
                return False
        return True
    
    def reserve_node_sequence(self, path: List[Tuple], vehicle_id: str, 
                            start_time: float, speed: float = 1.0) -> Optional[List[Tuple]]:
        """预约路径上的节点序列"""
        reservations = []
        current_time = start_time
        
        # 获取路径上的所有骨干节点
        backbone_nodes = self._extract_backbone_nodes(path)
        
        for node_info in backbone_nodes:
            node_id = node_info['node_id']
            node_index = node_info['path_index']
            
            # 计算到达时间
            arrival_time = self._calculate_arrival_time(path, node_index, start_time, speed)
            duration = self.node_traversal_time
            
            if self.check_node_availability(node_id, vehicle_id, arrival_time, duration):
                reservations.append((node_id, arrival_time, arrival_time + duration))
            else:
                return None  # 预约失败
                
        return reservations
    
    def _extract_backbone_nodes(self, path: List[Tuple]) -> List[Dict]:
        """提取路径上的骨干节点"""
        nodes = []
        if not self.backbone_network:
            return nodes
            
        # 遍历路径，找出属于骨干网络的节点
        for i, point in enumerate(path):
            # 检查是否为骨干接口节点
            for interface_id, interface_info in self.backbone_network.backbone_interfaces.items():
                if self._is_same_position(point, interface_info['position']):
                    nodes.append({
                        'node_id': interface_id,
                        'path_index': i,
                        'position': point
                    })
                    break
                    
        return nodes
    
    def _is_same_position(self, pos1: Tuple, pos2: Tuple, tolerance: float = 2.0) -> bool:
        """判断两个位置是否相同"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2) < tolerance
    
    def _calculate_arrival_time(self, path: List[Tuple], index: int, 
                               start_time: float, speed: float) -> float:
        """计算到达指定索引的时间"""
        if index <= 0:
            return start_time
            
        # 计算从起点到该索引的距离
        distance = 0.0
        for i in range(index):
            distance += math.sqrt(
                (path[i+1][0] - path[i][0])**2 + 
                (path[i+1][1] - path[i][1])**2
            )
            
        travel_time = distance / speed
        return start_time + travel_time
    
    def release_node_reservations(self, vehicle_id: str):
        """释放车辆的所有节点预约"""
        for node_id in list(self.node_reservations.keys()):
            self.node_reservations[node_id] = SortedList(
                [(start, end, vid) for start, end, vid in self.node_reservations[node_id]
                 if vid != vehicle_id]
            )


class BackbonePathSwitcher:
    """骨干路径切换器"""
    
    def __init__(self, backbone_network, path_planner):
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        self.switch_history = defaultdict(list)  # 记录切换历史
        
    def find_alternative_backbone_path(self, vehicle_id: str, current_pos: Tuple, 
                                     goal: Dict, conflict_info: Dict, 
                                     used_paths: Set[str]) -> Optional[Dict]:
        """寻找替代骨干路径"""
        
        # 1. 获取所有可用骨干路径
        available_paths = self._get_available_backbone_paths(goal, used_paths)
        
        if not available_paths:
            return None
            
        # 2. 评估每条路径
        path_evaluations = []
        for path_id, path_data in available_paths.items():
            score = self._evaluate_alternative_path(
                path_data, current_pos, conflict_info, vehicle_id
            )
            path_evaluations.append((score, path_id, path_data))
        
        # 3. 按评分排序，选择最佳替代路径
        path_evaluations.sort(reverse=True)
        
        for score, path_id, path_data in path_evaluations:
            # 尝试找到无冲突的接入点
            access_point = self._find_conflict_free_access(
                current_pos, path_data, conflict_info
            )
            if access_point:
                # 记录切换历史
                self.switch_history[vehicle_id].append({
                    'time': time.time(),
                    'from_path': conflict_info.get('current_path_id'),
                    'to_path': path_id,
                    'reason': conflict_info.get('conflict_type')
                })
                
                return {
                    'path_id': path_id,
                    'access_point': access_point,
                    'path_data': path_data,
                    'score': score
                }
        
        return None
    
    def _get_available_backbone_paths(self, goal: Dict, used_paths: Set[str]) -> Dict:
        """获取可用的骨干路径"""
        if not self.backbone_network:
            return {}
            
        # 使用backbone_network的方法查找到目标的所有路径
        all_paths = {}
        target_type = goal.get('type', 'unloading')
        target_id = goal.get('id', 0)
        
        # 查找连接到目标的所有骨干路径
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            # 检查路径是否连接到目标
            if ((path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)):
                
                # 排除已使用的路径
                if path_id not in used_paths:
                    all_paths[path_id] = path_data
                    
        return all_paths
    
    def _evaluate_alternative_path(self, path_data: Any, current_pos: Tuple, 
                                  conflict_info: Dict, vehicle_id: str) -> float:
        """评估替代路径的得分"""
        score = 1.0
        
        # 1. 路径质量
        score *= path_data.quality
        
        # 2. 当前负载（负载越低越好）
        load_factor = path_data.get_load_factor()
        score *= (1.0 - load_factor * 0.5)
        
        # 3. 距离因素（接入距离越近越好）
        min_access_distance = self._calculate_min_access_distance(current_pos, path_data)
        distance_score = 1.0 / (1.0 + min_access_distance / 100.0)
        score *= distance_score
        
        # 4. 历史切换频率（频繁切换的路径降低评分）
        switch_count = self._get_recent_switch_count(vehicle_id, path_data.path_id)
        if switch_count > 2:
            score *= 0.7
            
        return score
    
    def _find_conflict_free_access(self, current_pos: Tuple, path_data: Any, 
                                  conflict_info: Dict) -> Optional[Dict]:
        """找到无冲突的接入点"""
        # 获取路径上的所有接口节点
        interface_nodes = self._get_path_interface_nodes(path_data)
        
        best_access = None
        min_cost = float('inf')
        
        for node_info in interface_nodes:
            # 检查节点是否在冲突区域内
            if self._is_node_conflict_free(node_info, conflict_info):
                # 计算接入成本
                access_cost = self._calculate_access_cost(current_pos, node_info)
                
                if access_cost < min_cost:
                    min_cost = access_cost
                    best_access = {
                        'node_id': node_info['id'],
                        'position': node_info['position'],
                        'cost': access_cost
                    }
                    
        return best_access
    
    def _calculate_min_access_distance(self, current_pos: Tuple, path_data: Any) -> float:
        """计算最小接入距离"""
        min_distance = float('inf')
        
        # 检查前向路径
        for point in path_data.forward_path:
            distance = math.sqrt((point[0] - current_pos[0])**2 + (point[1] - current_pos[1])**2)
            min_distance = min(min_distance, distance)
            
        return min_distance
    
    def _get_recent_switch_count(self, vehicle_id: str, path_id: str) -> int:
        """获取最近切换到该路径的次数"""
        if vehicle_id not in self.switch_history:
            return 0
            
        recent_time = time.time() - 300  # 最近5分钟
        count = 0
        
        for switch in self.switch_history[vehicle_id]:
            if switch['time'] > recent_time and switch['to_path'] == path_id:
                count += 1
                
        return count
    
    def _get_path_interface_nodes(self, path_data: Any) -> List[Dict]:
        """获取路径的接口节点"""
        nodes = []
        
        if not self.backbone_network:
            return nodes
            
        # 从backbone_network获取该路径的接口节点
        path_id = path_data.path_id
        if path_id in self.backbone_network.path_interfaces:
            for interface_id in self.backbone_network.path_interfaces[path_id]:
                if interface_id in self.backbone_network.backbone_interfaces:
                    interface_info = self.backbone_network.backbone_interfaces[interface_id]
                    nodes.append({
                        'id': interface_id,
                        'position': interface_info['position'],
                        'is_occupied': interface_info.get('is_occupied', False)
                    })
                    
        return nodes
    
    def _is_node_conflict_free(self, node_info: Dict, conflict_info: Dict) -> bool:
        """检查节点是否无冲突"""
        # 检查节点是否被占用
        if node_info.get('is_occupied', False):
            return False
            
        # 检查节点是否在冲突区域内
        conflict_location = conflict_info.get('location')
        if conflict_location:
            node_pos = node_info['position']
            distance = math.sqrt(
                (node_pos[0] - conflict_location[0])**2 + 
                (node_pos[1] - conflict_location[1])**2
            )
            # 如果节点离冲突位置太近，认为不安全
            if distance < 20.0:
                return False
                
        return True
    
    def _calculate_access_cost(self, current_pos: Tuple, node_info: Dict) -> float:
        """计算接入成本"""
        node_pos = node_info['position']
        distance = math.sqrt(
            (node_pos[0] - current_pos[0])**2 + 
            (node_pos[1] - current_pos[1])**2
        )
        return distance


class IntelligentParkingDecisionMaker:
    """智能停车等待决策器"""
    
    def __init__(self, env, traffic_manager):
        self.env = env
        self.traffic_manager = traffic_manager
        self.default_parking_duration = 20.0
        self.parking_safety_distance = 15.0
        
    def should_park_and_wait(self, vehicle_id: str, conflict_analysis: Dict) -> Tuple[bool, Optional[ParkingDecision]]:
        """决定是否应该停车等待"""
        
        conflict_type = conflict_analysis.get('type')
        
        if conflict_type == 'narrow_passage':
            # 狭窄通道冲突
            return self._handle_narrow_passage_conflict(vehicle_id, conflict_analysis)
            
        elif conflict_type == 'intersection':
            # 交叉口冲突
            return self._handle_intersection_conflict(vehicle_id, conflict_analysis)
            
        elif conflict_type == 'resource_exhausted':
            # 所有路径都被占用
            return self._handle_resource_exhausted(vehicle_id, conflict_analysis)
            
        elif conflict_type == 'node_occupation':
            # 节点占用冲突
            return self._handle_node_occupation_conflict(vehicle_id, conflict_analysis)
            
        return False, None
    
    def _handle_narrow_passage_conflict(self, vehicle_id: str, 
                                      conflict_analysis: Dict) -> Tuple[bool, Optional[ParkingDecision]]:
        """处理狭窄通道冲突"""
        conflicting_vehicles = conflict_analysis.get('conflicting_vehicles', [])
        vehicle_priorities = conflict_analysis.get('priorities', {})
        
        # 获取自己的优先级
        my_priority = vehicle_priorities.get(vehicle_id, 0.5)
        
        # 检查是否是优先级最低的车辆
        should_yield = True
        priority_vehicle = None
        
        for other_id in conflicting_vehicles:
            if other_id != vehicle_id:
                other_priority = vehicle_priorities.get(other_id, 0.5)
                if my_priority > other_priority:
                    should_yield = False
                else:
                    priority_vehicle = other_id
                    
        if should_yield:
            # 找到安全停车位置
            parking_position = self._find_safe_parking_position(
                vehicle_id, conflict_analysis.get('location')
            )
            
            if parking_position:
                parking_decision = ParkingDecision(
                    vehicle_id=vehicle_id,
                    parking_position=parking_position,
                    parking_duration=self.default_parking_duration,
                    start_time=time.time(),
                    reason='narrow_passage_yield',
                    priority_vehicle_id=priority_vehicle
                )
                return True, parking_decision
                
        return False, None
    
    def _handle_intersection_conflict(self, vehicle_id: str, 
                                    conflict_analysis: Dict) -> Tuple[bool, Optional[ParkingDecision]]:
        """处理交叉口冲突"""
        # 基于到达时间和优先级决定
        arrival_times = conflict_analysis.get('arrival_times', {})
        my_arrival = arrival_times.get(vehicle_id, float('inf'))
        
        # 检查是否有车辆会先到达
        for other_id, other_arrival in arrival_times.items():
            if other_id != vehicle_id and other_arrival < my_arrival - 5.0:
                # 其他车辆明显先到，应该等待
                parking_position = self._find_safe_parking_position(
                    vehicle_id, conflict_analysis.get('location')
                )
                
                if parking_position:
                    # 计算需要等待的时间
                    wait_time = max(self.default_parking_duration, other_arrival - my_arrival + 10.0)
                    
                    parking_decision = ParkingDecision(
                        vehicle_id=vehicle_id,
                        parking_position=parking_position,
                        parking_duration=wait_time,
                        start_time=time.time(),
                        reason='intersection_yield',
                        expected_resolution_time=time.time() + wait_time
                    )
                    return True, parking_decision
                    
        return False, None
    
    def _handle_resource_exhausted(self, vehicle_id: str, 
                                 conflict_analysis: Dict) -> Tuple[bool, Optional[ParkingDecision]]:
        """处理资源耗尽（所有路径都被占用）"""
        # 必须停车等待
        current_position = conflict_analysis.get('current_position')
        parking_position = self._find_safe_parking_position(vehicle_id, current_position)
        
        if parking_position:
            parking_decision = ParkingDecision(
                vehicle_id=vehicle_id,
                parking_position=parking_position,
                parking_duration=self.default_parking_duration * 1.5,  # 延长等待时间
                start_time=time.time(),
                reason='resource_exhausted'
            )
            return True, parking_decision
            
        return False, None
    
    def _handle_node_occupation_conflict(self, vehicle_id: str, 
                                       conflict_analysis: Dict) -> Tuple[bool, Optional[ParkingDecision]]:
        """处理节点占用冲突"""
        occupied_until = conflict_analysis.get('occupied_until', time.time() + self.default_parking_duration)
        current_time = time.time()
        wait_time = max(10.0, occupied_until - current_time + 5.0)
        
        parking_position = self._find_safe_parking_position(
            vehicle_id, conflict_analysis.get('location')
        )
        
        if parking_position:
            parking_decision = ParkingDecision(
                vehicle_id=vehicle_id,
                parking_position=parking_position,
                parking_duration=wait_time,
                start_time=current_time,
                reason='node_occupation_wait',
                expected_resolution_time=occupied_until
            )
            return True, parking_decision
            
        return False, None
    
    def _find_safe_parking_position(self, vehicle_id: str, 
                                  reference_position: Optional[Tuple]) -> Optional[Tuple]:
        """找到安全的停车位置"""
        if not reference_position:
            # 使用车辆当前位置
            if self.env and vehicle_id in self.env.vehicles:
                vehicle = self.env.vehicles[vehicle_id]
                reference_position = getattr(vehicle, 'position', (0, 0, 0))
            else:
                return None
                
        # 在参考位置周围寻找安全位置
        search_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        search_distances = [10, 15, 20, 25]
        
        for distance in search_distances:
            for angle in search_angles:
                rad = math.radians(angle)
                candidate = (
                    reference_position[0] + distance * math.cos(rad),
                    reference_position[1] + distance * math.sin(rad),
                    reference_position[2] if len(reference_position) > 2 else 0
                )
                
                if self._is_position_safe_for_parking(candidate, vehicle_id):
                    return candidate
                    
        return None
    
    def _is_position_safe_for_parking(self, position: Tuple, vehicle_id: str) -> bool:
        """检查位置是否适合停车"""
        # 检查是否在地图范围内
        if not self.env:
            return False
            
        x, y = position[0], position[1]
        if x < 0 or x >= self.env.width or y < 0 or y >= self.env.height:
            return False
            
        # 检查是否有障碍物
        if hasattr(self.env, 'grid') and self.env.grid[int(x), int(y)] == 1:
            return False
            
        # 检查与其他车辆的距离
        if hasattr(self.env, 'vehicles'):
            for other_id, other_vehicle in self.env.vehicles.items():
                if other_id != vehicle_id:
                    other_pos = getattr(other_vehicle, 'position', (0, 0, 0))
                    distance = math.sqrt(
                        (position[0] - other_pos[0])**2 + 
                        (position[1] - other_pos[1])**2
                    )
                    if distance < self.parking_safety_distance:
                        return False
                        
        return True
    
    def simulate_parking_outcome(self, parking_decision: ParkingDecision, 
                               time_horizon: float = 30.0) -> Dict:
        """模拟停车后的场景，预测冲突是否会解决"""
        future_time = parking_decision.start_time + parking_decision.parking_duration
        
        # 预测系统状态
        future_state = self._project_system_state(future_time)
        
        # 检查停车后是否还有冲突
        future_conflicts = self._detect_conflicts_in_state(future_state, parking_decision.vehicle_id)
        
        # 计算最优等待时间
        optimal_wait_time = parking_decision.parking_duration
        if future_conflicts:
            # 如果还有冲突，可能需要延长等待时间
            additional_wait = self._calculate_additional_wait_time(future_conflicts)
            optimal_wait_time += additional_wait
            
        return {
            'conflicts_resolved': len(future_conflicts) == 0,
            'remaining_conflicts': future_conflicts,
            'recommended_wait_time': optimal_wait_time,
            'confidence': self._calculate_prediction_confidence(future_time - time.time())
        }
    
    def _project_system_state(self, future_time: float) -> Dict:
        """预测未来的系统状态"""
        projected_state = {
            'vehicle_positions': {},
            'occupied_nodes': {},
            'active_paths': {}
        }
        
        # 简化实现：基于当前速度和路径预测未来位置
        if hasattr(self.traffic_manager, 'active_paths'):
            for vehicle_id, path_info in self.traffic_manager.active_paths.items():
                # 预测车辆位置
                future_position = self._predict_vehicle_position(
                    vehicle_id, path_info, future_time
                )
                projected_state['vehicle_positions'][vehicle_id] = future_position
                
        return projected_state
    
    def _detect_conflicts_in_state(self, state: Dict, parking_vehicle_id: str) -> List[Dict]:
        """在给定状态下检测冲突"""
        conflicts = []
        
        # 简化实现：检查位置冲突
        positions = state.get('vehicle_positions', {})
        parking_pos = positions.get(parking_vehicle_id)
        
        if not parking_pos:
            return conflicts
            
        for other_id, other_pos in positions.items():
            if other_id != parking_vehicle_id:
                distance = math.sqrt(
                    (parking_pos[0] - other_pos[0])**2 + 
                    (parking_pos[1] - other_pos[1])**2
                )
                if distance < 10.0:  # 冲突距离阈值
                    conflicts.append({
                        'type': 'position_conflict',
                        'vehicles': [parking_vehicle_id, other_id],
                        'distance': distance
                    })
                    
        return conflicts
    
    def _calculate_additional_wait_time(self, conflicts: List[Dict]) -> float:
        """计算额外等待时间"""
        # 基于冲突类型和严重程度计算
        max_wait = 0.0
        
        for conflict in conflicts:
            if conflict['type'] == 'position_conflict':
                # 位置冲突需要等待对方通过
                wait_time = 15.0  # 估计通过时间
                max_wait = max(max_wait, wait_time)
                
        return max_wait
    
    def _predict_vehicle_position(self, vehicle_id: str, path_info: Dict, 
                                future_time: float) -> Tuple:
        """预测车辆未来位置"""
        # 简化实现
        current_pos = path_info.get('current_position', (0, 0, 0))
        return current_pos
    
    def _calculate_prediction_confidence(self, time_delta: float) -> float:
        """计算预测置信度"""
        # 时间越远，置信度越低
        return max(0.1, 1.0 - time_delta / 300.0)


class EnhancedConflictDetector:
    """增强冲突检测器 - 支持骨干路径特定冲突"""
    
    def __init__(self, env, backbone_network=None):
        self.env = env
        self.backbone_network = backbone_network
        self.safety_margin = 3.0
        self.node_conflict_radius = 5.0
        
    def detect_all_conflicts(self, solution: Dict[str, List[Tuple]], 
                           backbone_info: Dict[str, Dict] = None) -> List[ECBSConflict]:
        """检测所有冲突，包括骨干路径特定冲突"""
        conflicts = []
        
        # 1. 基础时空冲突检测
        basic_conflicts = self._detect_basic_conflicts(solution)
        conflicts.extend(basic_conflicts)
        
        # 2. 骨干路径特定冲突检测
        if backbone_info:
            backbone_conflicts = self._detect_backbone_specific_conflicts(solution, backbone_info)
            conflicts.extend(backbone_conflicts)
            
        return conflicts
    
    def _detect_basic_conflicts(self, solution: Dict[str, List[Tuple]]) -> List[ECBSConflict]:
        """检测基础时空冲突"""
        conflicts = []
        agents = list(solution.keys())
        
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                agent1, agent2 = agents[i], agents[j]
                path1, path2 = solution[agent1], solution[agent2]
                
                # 时间-空间冲突检测
                temporal_conflicts = self._detect_temporal_conflicts(
                    agent1, agent2, path1, path2
                )
                conflicts.extend(temporal_conflicts)
                
        return conflicts
    
    def _detect_backbone_specific_conflicts(self, solution: Dict[str, List[Tuple]], 
                                          backbone_info: Dict[str, Dict]) -> List[ECBSConflict]:
        """检测骨干路径特定冲突"""
        conflicts = []
        
        # 1. 节点占用冲突
        node_conflicts = self._detect_node_occupation_conflicts(solution, backbone_info)
        conflicts.extend(node_conflicts)
        
        # 2. 狭窄通道冲突
        narrow_passage_conflicts = self._detect_narrow_passage_conflicts(solution, backbone_info)
        conflicts.extend(narrow_passage_conflicts)
        
        # 3. 接入点冲突
        access_conflicts = self._detect_access_point_conflicts(solution, backbone_info)
        conflicts.extend(access_conflicts)
        
        return conflicts
    
    def _detect_node_occupation_conflicts(self, solution: Dict[str, List[Tuple]], 
                                        backbone_info: Dict[str, Dict]) -> List[ECBSConflict]:
        """检测节点占用冲突"""
        conflicts = []
        node_occupations = defaultdict(list)  # {(node_id, time): [vehicles]}
        
        # 收集所有车辆的节点占用信息
        for vehicle_id, path in solution.items():
            if vehicle_id not in backbone_info:
                continue
                
            vehicle_backbone = backbone_info[vehicle_id]
            backbone_id = vehicle_backbone.get('backbone_id')
            
            if not backbone_id or not self.backbone_network:
                continue
                
            # 提取路径上的骨干节点
            nodes = self._extract_path_nodes(path, backbone_id)
            
            for node_info in nodes:
                node_id = node_info['node_id']
                arrival_time = node_info['arrival_time']
                
                # 量化时间到时间步
                time_step = int(arrival_time)
                
                # 检查时间窗口内的占用
                for t in range(time_step - 2, time_step + 3):
                    node_occupations[(node_id, t)].append(vehicle_id)
                    
        # 检测冲突
        for (node_id, time_step), vehicles in node_occupations.items():
            if len(vehicles) > 1:
                # 获取节点位置
                node_position = self._get_node_position(node_id)
                
                conflict = ECBSConflict(
                    conflict_id=f"node_{node_id}_{time_step}",
                    conflict_type=ConflictType.NODE_OCCUPATION,
                    agents=vehicles,
                    location=node_position,
                    time=time_step,
                    severity=0.9,
                    node_id=node_id
                )
                conflicts.append(conflict)
                
        return conflicts
    
    def _detect_narrow_passage_conflicts(self, solution: Dict[str, List[Tuple]], 
                                       backbone_info: Dict[str, Dict]) -> List[ECBSConflict]:
        """检测狭窄通道冲突"""
        conflicts = []
        
        # 按骨干路径分组车辆
        backbone_vehicles = defaultdict(list)
        for vehicle_id, info in backbone_info.items():
            backbone_id = info.get('backbone_id')
            if backbone_id:
                backbone_vehicles[backbone_id].append((vehicle_id, info))
                
        # 检查每条骨干路径上的车辆
        for backbone_id, vehicles in backbone_vehicles.items():
            if len(vehicles) < 2:
                continue
                
            # 检查车辆之间的时空重叠
            for i in range(len(vehicles)):
                for j in range(i + 1, len(vehicles)):
                    vehicle1_id, info1 = vehicles[i]
                    vehicle2_id, info2 = vehicles[j]
                    
                    # 检查时间窗口重叠
                    overlap = self._check_time_window_overlap(info1, info2)
                    
                    if overlap:
                        # 找到冲突位置
                        conflict_location = self._find_narrow_passage_location(
                            solution[vehicle1_id], solution[vehicle2_id], overlap
                        )
                        
                        conflict = ECBSConflict(
                            conflict_id=f"narrow_{backbone_id}_{vehicle1_id}_{vehicle2_id}",
                            conflict_type=ConflictType.NARROW_PASSAGE,
                            agents=[vehicle1_id, vehicle2_id],
                            location=conflict_location,
                            time=overlap[0],
                            severity=0.85,
                            backbone_id=backbone_id
                        )
                        conflicts.append(conflict)
                        
        return conflicts
    
    def _detect_access_point_conflicts(self, solution: Dict[str, List[Tuple]], 
                                     backbone_info: Dict[str, Dict]) -> List[ECBSConflict]:
        """检测接入点冲突"""
        conflicts = []
        access_points = defaultdict(list)  # {(interface_id, time_window): [vehicles]}
        
        # 收集接入点使用信息
        for vehicle_id, info in backbone_info.items():
            if 'access_point' in info:
                interface_id = info['access_point'].get('interface_id')
                access_time = info['access_point'].get('time')
                
                if interface_id and access_time is not None:
                    time_window = (int(access_time - 5), int(access_time + 5))
                    access_points[(interface_id, time_window)].append(vehicle_id)
                    
        # 检测冲突
        for (interface_id, time_window), vehicles in access_points.items():
            if len(vehicles) > 1:
                interface_position = self._get_interface_position(interface_id)
                
                conflict = ECBSConflict(
                    conflict_id=f"access_{interface_id}_{time_window[0]}",
                    conflict_type=ConflictType.ACCESS_POINT,
                    agents=vehicles,
                    location=interface_position,
                    time=time_window[0],
                    severity=0.8
                )
                conflicts.append(conflict)
                
        return conflicts
    
    def _detect_temporal_conflicts(self, agent1: str, agent2: str,
                                 path1: List[Tuple], path2: List[Tuple]) -> List[ECBSConflict]:
        """检测时间-空间冲突"""
        conflicts = []
        max_time = min(len(path1), len(path2))
        
        for t in range(max_time):
            pos1, pos2 = path1[t], path2[t]
            
            # 顶点冲突检测
            if self._positions_conflict(pos1, pos2):
                conflict = ECBSConflict(
                    conflict_id=f"vertex_{agent1}_{agent2}_{t}",
                    conflict_type=ConflictType.TEMPORAL,
                    agents=[agent1, agent2],
                    location=((pos1[0] + pos2[0])/2, (pos1[1] + pos2[1])/2),
                    time=t,
                    severity=self._calculate_conflict_severity(pos1, pos2)
                )
                conflicts.append(conflict)
            
            # 边冲突检测
            if t < max_time - 1:
                next_pos1, next_pos2 = path1[t+1], path2[t+1]
                if self._edge_conflict(pos1, next_pos1, pos2, next_pos2):
                    conflict = ECBSConflict(
                        conflict_id=f"edge_{agent1}_{agent2}_{t}",
                        conflict_type=ConflictType.TEMPORAL,
                        agents=[agent1, agent2],
                        location=((pos1[0] + pos2[0])/2, (pos1[1] + pos2[1])/2),
                        time=t,
                        severity=0.8
                    )
                    conflicts.append(conflict)
        
        return conflicts
    
    def _extract_path_nodes(self, path: List[Tuple], backbone_id: str) -> List[Dict]:
        """提取路径上的骨干节点"""
        nodes = []
        
        if not self.backbone_network or backbone_id not in self.backbone_network.path_interfaces:
            return nodes
            
        # 获取该骨干路径的所有接口节点
        interface_ids = self.backbone_network.path_interfaces[backbone_id]
        
        for i, point in enumerate(path):
            for interface_id in interface_ids:
                interface_info = self.backbone_network.backbone_interfaces.get(interface_id)
                if interface_info and self._is_same_position(point, interface_info['position']):
                    nodes.append({
                        'node_id': interface_id,
                        'arrival_time': i * 1.0,  # 简化：假设匀速
                        'position': point
                    })
                    break
                    
        return nodes
    
    def _get_node_position(self, node_id: str) -> Tuple:
        """获取节点位置"""
        if self.backbone_network and node_id in self.backbone_network.backbone_interfaces:
            return self.backbone_network.backbone_interfaces[node_id]['position']
        return (0, 0)
    
    def _get_interface_position(self, interface_id: str) -> Tuple:
        """获取接口位置"""
        return self._get_node_position(interface_id)
    
    def _check_time_window_overlap(self, info1: Dict, info2: Dict) -> Optional[Tuple]:
        """检查时间窗口重叠"""
        start1 = info1.get('usage_start_time', 0)
        end1 = info1.get('usage_end_time', 100)
        start2 = info2.get('usage_start_time', 0)
        end2 = info2.get('usage_end_time', 100)
        
        overlap_start = max(start1, start2)
        overlap_end = min(end1, end2)
        
        if overlap_start < overlap_end:
            return (overlap_start, overlap_end)
        return None
    
    def _find_narrow_passage_location(self, path1: List[Tuple], path2: List[Tuple], 
                                    time_window: Tuple) -> Tuple:
        """找到狭窄通道冲突位置"""
        # 简化：返回时间窗口中点对应的位置
        mid_time = int((time_window[0] + time_window[1]) / 2)
        
        if mid_time < len(path1) and mid_time < len(path2):
            pos1 = path1[mid_time]
            pos2 = path2[mid_time]
            return ((pos1[0] + pos2[0])/2, (pos1[1] + pos2[1])/2)
            
        return (0, 0)
    
    def _positions_conflict(self, pos1: Tuple, pos2: Tuple) -> bool:
        """检查位置冲突"""
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return distance < self.safety_margin
    
    def _edge_conflict(self, pos1: Tuple, next_pos1: Tuple, 
                      pos2: Tuple, next_pos2: Tuple) -> bool:
        """检查边冲突（交叉路径）"""
        return (self._positions_conflict(pos1, next_pos2) and 
                self._positions_conflict(next_pos1, pos2))
    
    def _calculate_conflict_severity(self, pos1: Tuple, pos2: Tuple) -> float:
        """计算冲突严重程度"""
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return max(0.1, 1.0 - (distance / self.safety_margin))
    
    def _is_same_position(self, pos1: Tuple, pos2: Tuple, tolerance: float = 2.0) -> bool:
        """判断两个位置是否相同"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2) < tolerance
class ECBSConstraint:
    """ECBS约束"""
    constraint_type: ConstraintType
    agent: Optional[str] = None
    position: Optional[Tuple] = None
    position2: Optional[Tuple] = None
    time: Optional[int] = None
    time_window: Optional[Tuple[int, int]] = None
    backbone_id: Optional[str] = None
    max_agents: Optional[int] = None
    
    def __hash__(self):
        return hash((self.constraint_type, self.agent, self.position, 
                    self.time, self.backbone_id))
class ConstraintManager:
    """约束管理器"""
    
    def __init__(self):
        self.constraints_by_agent = defaultdict(set)
        self.backbone_constraints = defaultdict(list)
        self.capacity_constraints = defaultdict(list)
    
    def add_constraint(self, constraint: ECBSConstraint):
        """添加约束"""
        if constraint.agent:
            self.constraints_by_agent[constraint.agent].add(constraint)
        
        if constraint.constraint_type == ConstraintType.BACKBONE:
            self.backbone_constraints[constraint.backbone_id].append(constraint)
        elif constraint.constraint_type == ConstraintType.CAPACITY:
            self.capacity_constraints[constraint.backbone_id].append(constraint)
    
    def get_constraints_for_agent(self, agent: str) -> Set[ECBSConstraint]:
        """获取特定智能体的约束"""
        return self.constraints_by_agent.get(agent, set())
    
    def violates_constraint(self, agent: str, path: List[Tuple], 
                          backbone_info: Dict = None) -> bool:
        """检查路径是否违反约束"""
        constraints = self.get_constraints_for_agent(agent)
        
        for constraint in constraints:
            if self._path_violates_constraint(path, constraint, backbone_info):
                return True
        
        # 检查骨干路径约束
        if backbone_info and backbone_info.get('backbone_id'):
            backbone_id = backbone_info['backbone_id']
            for constraint in self.backbone_constraints.get(backbone_id, []):
                if self._path_violates_backbone_constraint(path, constraint, backbone_info):
                    return True
        
        return False
    
    def _path_violates_constraint(self, path: List[Tuple], 
                                constraint: ECBSConstraint,
                                backbone_info: Dict = None) -> bool:
        """检查路径是否违反特定约束"""
        if constraint.constraint_type == ConstraintType.VERTEX:
            return self._violates_vertex_constraint(path, constraint)
        elif constraint.constraint_type == ConstraintType.EDGE:
            return self._violates_edge_constraint(path, constraint)
        elif constraint.constraint_type == ConstraintType.BACKBONE:
            return self._violates_backbone_constraint(path, constraint, backbone_info)
        
        return False
    
    def _violates_vertex_constraint(self, path: List[Tuple], 
                                  constraint: ECBSConstraint) -> bool:
        """检查顶点约束违反"""
        if constraint.time is None or constraint.position is None:
            return False
        
        if constraint.time < len(path):
            current_pos = path[constraint.time]
            constraint_pos = constraint.position
            
            distance = math.sqrt(
                (current_pos[0] - constraint_pos[0])**2 + 
                (current_pos[1] - constraint_pos[1])**2
            )
            
            return distance < 2.0  # 2米内认为冲突
        
        return False
    
    def _violates_edge_constraint(self, path: List[Tuple], 
                                constraint: ECBSConstraint) -> bool:
        """检查边约束违反"""
        if (constraint.time is None or constraint.position is None or 
            constraint.position2 is None):
            return False
        
        if constraint.time < len(path) - 1:
            current_pos = path[constraint.time]
            next_pos = path[constraint.time + 1]
            
            # 检查是否使用了被约束的边
            return (self._positions_close(current_pos, constraint.position) and
                    self._positions_close(next_pos, constraint.position2))
        
        return False
    
    def _violates_backbone_constraint(self, path: List[Tuple], 
                                    constraint: ECBSConstraint,
                                    backbone_info: Dict) -> bool:
        """检查骨干路径约束违反"""
        if not backbone_info or constraint.backbone_id != backbone_info.get('backbone_id'):
            return False
        
        if constraint.time_window:
            start_time, end_time = constraint.time_window
            # 检查使用时间窗口是否冲突
            usage_start = backbone_info.get('usage_start_time', 0)
            usage_end = backbone_info.get('usage_end_time', len(path))
            
            return not (usage_end <= start_time or usage_start >= end_time)
        
        return False
    
    def _positions_close(self, pos1: Tuple, pos2: Tuple, threshold: float = 2.0) -> bool:
        """检查两个位置是否接近"""
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return distance < threshold

class BackboneAwareConstraintManager(ConstraintManager):
    """骨干路径感知的约束管理器"""
    
    def __init__(self):
        super().__init__()
        self.node_constraints = defaultdict(list)  # {node_id: [constraints]}
        self.path_switch_constraints = defaultdict(set)  # {agent: {forbidden_paths}}
        
    def add_constraint(self, constraint: ECBSConstraint):
        """添加约束 - 增强版"""
        super().add_constraint(constraint)
        
        # 处理节点约束
        if isinstance(constraint, BackboneNodeConstraint):
            self.node_constraints[constraint.node_id].append(constraint)
            
        # 处理路径切换约束
        if constraint.constraint_type == ConstraintType.PATH_SWITCH:
            if constraint.agent and constraint.backbone_id:
                self.path_switch_constraints[constraint.agent].add(constraint.backbone_id)
                
    def violates_constraint(self, agent: str, path: List[Tuple], 
                          backbone_info: Dict = None) -> bool:
        """检查路径是否违反约束 - 增强版"""
        # 先检查基础约束
        if super().violates_constraint(agent, path, backbone_info):
            return True
            
        # 检查节点约束
        if backbone_info and self._violates_node_constraints(agent, path, backbone_info):
            return True
            
        # 检查路径切换约束
        if backbone_info and self._violates_path_switch_constraints(agent, backbone_info):
            return True
            
        return False
    
    def _violates_node_constraints(self, agent: str, path: List[Tuple], 
                                  backbone_info: Dict) -> bool:
        """检查是否违反节点约束"""
        # 获取路径上的所有节点
        path_nodes = self._extract_path_nodes(path, backbone_info)
        
        for node_info in path_nodes:
            node_id = node_info['node_id']
            arrival_time = node_info['arrival_time']
            
            # 检查该节点的所有约束
            for constraint in self.node_constraints.get(node_id, []):
                if constraint.agent == agent:
                    continue  # 跳过自己的约束
                    
                # 检查时间窗口冲突
                if (constraint.time_window[0] <= arrival_time <= constraint.time_window[1]):
                    return True
                    
        return False
    
    def _violates_path_switch_constraints(self, agent: str, backbone_info: Dict) -> bool:
        """检查是否违反路径切换约束"""
        backbone_id = backbone_info.get('backbone_id')
        
        if backbone_id and agent in self.path_switch_constraints:
            forbidden_paths = self.path_switch_constraints[agent]
            return backbone_id in forbidden_paths
            
        return False
    
    def _extract_path_nodes(self, path: List[Tuple], backbone_info: Dict) -> List[Dict]:
        """提取路径上的节点信息"""
        # 简化实现
        nodes = []
        backbone_id = backbone_info.get('backbone_id')
        
        if backbone_id:
            # 这里应该调用backbone_network的方法获取节点信息
            # 简化处理
            for i in range(0, len(path), 10):  # 每10个点作为一个节点
                nodes.append({
                    'node_id': f"{backbone_id}_node_{i}",
                    'arrival_time': i * 1.0,
                    'position': path[i]
                })
                
        return nodes

class ECBSSolver:
    """ECBS求解器核心"""
    
    def __init__(self, env, path_planner, backbone_network=None, 
                 max_expansions=500, timeout=30.0, suboptimality_bound=1.5):
        self.env = env
        self.path_planner = path_planner
        self.backbone_network = backbone_network
        self.max_expansions = max_expansions
        self.timeout = timeout
        self.suboptimality_bound = suboptimality_bound
        
        # 组件
        self.constraint_manager = ConstraintManager()
        self.conflict_detector = EnhancedConflictDetector(env, backbone_network)
        
        # 统计
        self.stats = {
            'expansions': 0,
            'constraints_generated': 0,
            'conflicts_resolved': 0,
            'solve_time': 0.0
        }
    
    def solve(self, agent_requests: Dict[str, Dict], 
              backbone_allocations: Dict[str, str] = None) -> Dict[str, Any]:
        """
        ECBS主求解算法
        
        Args:
            agent_requests: {agent_id: {'start': tuple, 'goal': tuple, 'priority': float}}
            backbone_allocations: {agent_id: backbone_path_id}
        
        Returns:
            解决方案字典
        """
        solve_start = time.time()
        self.stats['expansions'] = 0
        
        try:
            print(f"开始ECBS求解: {len(agent_requests)} 个智能体")
            
            # 步骤1: 计算初始解
            root_node = self._compute_initial_solution(agent_requests, backbone_allocations)
            if not root_node.solution:
                return {'success': False, 'error': '无法计算初始解'}
            
            # 步骤2: 检测初始冲突
            initial_conflicts = self.conflict_detector.detect_all_conflicts(
                root_node.solution, self._extract_backbone_info(root_node.solution, backbone_allocations)
            )
            root_node.conflicts = initial_conflicts
            root_node.h_value = len(initial_conflicts)
            root_node.f_value = root_node.cost + root_node.h_value
            
            if not initial_conflicts:
                # 无冲突，直接返回
                solve_time = time.time() - solve_start
                return {
                    'success': True,
                    'paths': root_node.solution,
                    'structures': self._extract_path_structures(root_node.solution, backbone_allocations),
                    'total_cost': root_node.cost,
                    'initial_conflicts': 0,
                    'final_conflicts': 0,
                    'expansions': 0,
                    'solve_time': solve_time
                }
            
            # 步骤3: ECBS搜索
            open_list = [root_node]
            
            while open_list and self.stats['expansions'] < self.max_expansions:
                if time.time() - solve_start > self.timeout:
                    break
                
                # 获取最优节点
                current_node = heapq.heappop(open_list)
                self.stats['expansions'] += 1
                
                if not current_node.conflicts:
                    # 找到无冲突解
                    solve_time = time.time() - solve_start
                    self.stats['solve_time'] = solve_time
                    
                    return {
                        'success': True,
                        'paths': current_node.solution,
                        'structures': self._extract_path_structures(current_node.solution, backbone_allocations),
                        'total_cost': current_node.cost,
                        'initial_conflicts': len(initial_conflicts),
                        'final_conflicts': 0,
                        'expansions': self.stats['expansions'],
                        'constraints': self.stats['constraints_generated'],
                        'solve_time': solve_time
                    }
                
                # 选择关键冲突
                critical_conflict = self._select_critical_conflict(current_node.conflicts)
                
                # 生成子节点
                child_nodes = self._generate_child_nodes(current_node, critical_conflict, 
                                                      agent_requests, backbone_allocations)
                
                for child in child_nodes:
                    if child.solution:  # 只添加有效解
                        heapq.heappush(open_list, child)
                
                if self.stats['expansions'] % 50 == 0:
                    print(f"  ECBS扩展 {self.stats['expansions']}, 开集大小: {len(open_list)}")
            
            # 搜索结束，返回最佳解
            solve_time = time.time() - solve_start
            self.stats['solve_time'] = solve_time
            
            if open_list:
                best_node = min(open_list, key=lambda n: (len(n.conflicts), n.cost))
                return {
                    'success': True,
                    'paths': best_node.solution,
                    'structures': self._extract_path_structures(best_node.solution, backbone_allocations),
                    'total_cost': best_node.cost,
                    'initial_conflicts': len(initial_conflicts),
                    'final_conflicts': len(best_node.conflicts),
                    'expansions': self.stats['expansions'],
                    'constraints': self.stats['constraints_generated'],
                    'solve_time': solve_time
                }
            
            return {'success': False, 'error': 'ECBS搜索失败'}
        
        except Exception as e:
            return {'success': False, 'error': f'ECBS求解异常: {str(e)}'}
    
    def _compute_initial_solution(self, agent_requests: Dict[str, Dict],
                                backbone_allocations: Dict[str, str] = None) -> ECBSNode:
        """计算初始解（各智能体独立规划）"""
        root_node = ECBSNode()
        total_cost = 0.0
        
        for agent_id, request in agent_requests.items():
            try:
                # 为每个智能体单独规划路径
                path_result = self.path_planner.plan_path(
                    vehicle_id=agent_id,
                    start=request['start'],
                    goal=request['goal'],
                    use_backbone=True,
                    context='normal'
                )
                
                if path_result:
                    if isinstance(path_result, tuple):
                        path = path_result[0]
                    else:
                        path = path_result
                    
                    if path and len(path) >= 2:
                        root_node.solution[agent_id] = path
                        total_cost += len(path)  # 简化代价计算
                    else:
                        print(f"智能体 {agent_id} 路径规划失败")
                        return ECBSNode()  # 返回空解
                else:
                    print(f"智能体 {agent_id} 无法规划路径")
                    return ECBSNode()
                    
            except Exception as e:
                print(f"智能体 {agent_id} 规划异常: {e}")
                return ECBSNode()
        
        root_node.cost = total_cost
        return root_node
    
    def _select_critical_conflict(self, conflicts: List[ECBSConflict]) -> ECBSConflict:
        """选择关键冲突"""
        if not conflicts:
            return None
        
        # 按严重程度和类型选择
        priority_weights = {
            ConflictType.SAFETY_RECTANGLE: 1.0,
            ConflictType.BACKBONE_CONGESTION: 0.9,
            ConflictType.TEMPORAL: 0.8,
            ConflictType.CAPACITY: 0.85
        }
        
        def conflict_priority(conflict):
            type_weight = priority_weights.get(conflict.conflict_type, 0.5)
            return conflict.severity * type_weight
        
        return max(conflicts, key=conflict_priority)
    
    def _generate_child_nodes(self, parent_node: ECBSNode, conflict: ECBSConflict,
                            agent_requests: Dict[str, Dict],
                            backbone_allocations: Dict[str, str] = None) -> List[ECBSNode]:
        """生成子节点"""
        child_nodes = []
        
        # 为冲突中的每个智能体生成约束
        for agent in conflict.agents:
            child_node = ECBSNode()
            child_node.constraints = parent_node.constraints.copy()
            
            # 生成约束
            new_constraint = self._generate_constraint_for_conflict(conflict, agent)
            if new_constraint:
                child_node.constraints.add(new_constraint)
                self.constraint_manager.add_constraint(new_constraint)
                self.stats['constraints_generated'] += 1
                
                # 在约束下重新规划该智能体的路径
                child_node.solution = parent_node.solution.copy()
                
                new_path = self._replan_with_constraints(
                    agent, agent_requests[agent], child_node.constraints,
                    backbone_allocations
                )
                
                if new_path:
                    child_node.solution[agent] = new_path
                    child_node.cost = sum(len(path) for path in child_node.solution.values())
                    
                    # 检测新解的冲突
                    backbone_info = self._extract_backbone_info(child_node.solution, backbone_allocations)
                    child_node.conflicts = self.conflict_detector.detect_all_conflicts(
                        child_node.solution, backbone_info
                    )
                    child_node.h_value = len(child_node.conflicts)
                    child_node.f_value = child_node.cost + child_node.h_value
                    
                    child_nodes.append(child_node)
        
        return child_nodes
    
    def _generate_constraint_for_conflict(self, conflict: ECBSConflict, 
                                        agent: str) -> Optional[ECBSConstraint]:
        """为冲突生成约束"""
        if conflict.conflict_type == ConflictType.TEMPORAL:
            # 顶点约束
            return ECBSConstraint(
                constraint_type=ConstraintType.VERTEX,
                agent=agent,
                position=conflict.location,
                time=conflict.time
            )
        
        elif conflict.conflict_type == ConflictType.BACKBONE_CONGESTION:
            # 骨干路径约束
            return ECBSConstraint(
                constraint_type=ConstraintType.BACKBONE,
                agent=agent,
                backbone_id=conflict.backbone_id,
                time_window=(conflict.time, conflict.time + 10)
            )
        
        elif conflict.conflict_type == ConflictType.CAPACITY:
            # 容量约束
            return ECBSConstraint(
                constraint_type=ConstraintType.CAPACITY,
                backbone_id=conflict.backbone_id,
                max_agents=1,
                time_window=(conflict.time, conflict.time + 5)
            )
        
        return None
    
    def _replan_with_constraints(self, agent: str, request: Dict,
                               constraints: Set[ECBSConstraint],
                               backbone_allocations: Dict[str, str] = None) -> Optional[List[Tuple]]:
        """在约束下重新规划路径"""
        try:
            # 临时设置约束管理器
            temp_constraint_manager = ConstraintManager()
            for constraint in constraints:
                temp_constraint_manager.add_constraint(constraint)
            
            # 使用约束规划路径
            for attempt in range(3):  # 最多尝试3次
                path_result = self.path_planner.plan_path(
                    vehicle_id=agent,
                    start=request['start'],
                    goal=request['goal'],
                    use_backbone=True,
                    context='conflict_resolution',
                    quality_threshold=0.6 - attempt * 0.1  # 逐步降低质量要求
                )
                
                if path_result:
                    if isinstance(path_result, tuple):
                        path = path_result[0]
                    else:
                        path = path_result
                    
                    if path and len(path) >= 2:
                        # 检查路径是否违反约束
                        backbone_info = {}
                        if backbone_allocations and agent in backbone_allocations:
                            backbone_info = {
                                'backbone_id': backbone_allocations[agent],
                                'usage_start_time': 0,
                                'usage_end_time': len(path)
                            }
                        
                        if not temp_constraint_manager.violates_constraint(agent, path, backbone_info):
                            return path
            
            return None
            
        except Exception as e:
            print(f"约束下重规划失败 {agent}: {e}")
            return None
    
    def _extract_backbone_info(self, solution: Dict[str, List[Tuple]],
                             backbone_allocations: Dict[str, str] = None) -> Dict[str, Dict]:
        """提取骨干路径信息"""
        backbone_info = {}
        
        if backbone_allocations:
            for agent_id, path in solution.items():
                if agent_id in backbone_allocations:
                    backbone_info[agent_id] = {
                        'backbone_id': backbone_allocations[agent_id],
                        'usage_start_time': 0,
                        'usage_end_time': len(path),
                        'path_length': len(path)
                    }
        
        return backbone_info
    
    def _extract_path_structures(self, solution: Dict[str, List[Tuple]],
                               backbone_allocations: Dict[str, str] = None) -> Dict[str, Dict]:
        """提取路径结构信息"""
        structures = {}
        
        for agent_id, path in solution.items():
            structure = {
                'type': 'ecbs_coordinated',
                'total_length': len(path),
                'backbone_utilization': 0.0,
                'coordination_quality': 1.0
            }
            
            if backbone_allocations and agent_id in backbone_allocations:
                structure['backbone_id'] = backbone_allocations[agent_id]
                structure['backbone_utilization'] = 0.8  # 估计值
            
            structures[agent_id] = structure
        
        return structures
class BackboneAwareECBSSolver(ECBSSolver):
    """骨干路径感知的ECBS求解器"""
    
    def __init__(self, env, path_planner, backbone_network=None, **kwargs):
        super().__init__(env, path_planner, backbone_network, **kwargs)
        
        # 增强组件
        self.constraint_manager = BackboneAwareConstraintManager()
        self.conflict_detector = EnhancedConflictDetector(env, backbone_network)
        self.node_time_manager = NodeTimeWindowManager(backbone_network)
        self.path_switcher = BackbonePathSwitcher(backbone_network, path_planner)
        self.parking_decision_maker = IntelligentParkingDecisionMaker(env, None)
        
        # 骨干路径缓存
        self.backbone_alternatives = {}  # {(start_type, start_id, goal_type, goal_id): [path_ids]}
        
        # 增强统计
        self.enhanced_stats = {
            'path_switches': 0,
            'parking_decisions': 0,
            'node_conflicts_resolved': 0,
            'narrow_passage_handled': 0
        }
        
    def solve_with_backbone_awareness(self, agent_requests: Dict[str, Dict], 
                                    backbone_allocations: Dict[str, str] = None) -> Dict[str, Any]:
        """骨干路径感知的求解方法"""
        solve_start = time.time()
        
        # 1. 预处理：为每个车辆找出所有可用骨干路径
        self._precompute_backbone_alternatives(agent_requests)
        
        # 2. 初始化节点时间窗口
        self._initialize_node_time_windows()
        
        # 3. 执行增强的ECBS搜索
        result = self._enhanced_ecbs_search(agent_requests, backbone_allocations)
        
        # 4. 记录统计
        result['enhanced_stats'] = self.enhanced_stats.copy()
        result['solve_time'] = time.time() - solve_start
        
        return result
    
    def _precompute_backbone_alternatives(self, agent_requests: Dict[str, Dict]):
        """预计算骨干路径备选方案"""
        if not self.backbone_network:
            return
            
        for agent_id, request in agent_requests.items():
            goal = request.get('goal')
            
            # 解析目标类型和ID
            goal_type, goal_id = self._parse_goal_info(goal)
            
            if goal_type and goal_id is not None:
                # 查找所有可达该目标的骨干路径
                alternatives = []
                
                for path_id, path_data in self.backbone_network.bidirectional_paths.items():
                    if ((path_data.point_a['type'] == goal_type and path_data.point_a['id'] == goal_id) or
                        (path_data.point_b['type'] == goal_type and path_data.point_b['id'] == goal_id)):
                        alternatives.append(path_id)
                        
                key = (None, None, goal_type, goal_id)  # 简化键
                self.backbone_alternatives[key] = alternatives
                
    def _initialize_node_time_windows(self):
        """初始化节点时间窗口"""
        self.node_time_manager.node_reservations.clear()
        
    def _enhanced_ecbs_search(self, agent_requests: Dict[str, Dict],
                            backbone_allocations: Dict[str, str] = None) -> Dict[str, Any]:
        """增强的ECBS搜索"""
        # 使用父类的solve方法，但重写关键方法
        return self.solve(agent_requests, backbone_allocations)
    
    def _generate_child_nodes(self, parent_node: ECBSNode, conflict: ECBSConflict,
                            agent_requests: Dict[str, Dict],
                            backbone_allocations: Dict[str, str] = None) -> List[ECBSNode]:
        """生成子节点 - 增强版支持多种冲突解决策略"""
        child_nodes = []
        
        # 分析冲突类型
        conflict_analysis = self._analyze_conflict_type(conflict, parent_node)
        
        # 根据冲突类型采用不同策略
        if conflict.conflict_type in [ConflictType.NODE_OCCUPATION, ConflictType.ACCESS_POINT]:
            # 节点冲突 - 优先尝试路径切换
            child_nodes.extend(self._try_path_switch_strategy(
                parent_node, conflict, agent_requests, backbone_allocations
            ))
            
        elif conflict.conflict_type == ConflictType.NARROW_PASSAGE:
            # 狭窄通道 - 考虑停车等待
            child_nodes.extend(self._try_parking_strategy(
                parent_node, conflict, agent_requests, conflict_analysis
            ))
            
        # 如果特殊策略失败或不适用，使用传统约束方法
        if not child_nodes:
            child_nodes.extend(self._try_traditional_constraints(
                parent_node, conflict, agent_requests, backbone_allocations
            ))
            
        return child_nodes
    
    def _analyze_conflict_type(self, conflict: ECBSConflict, 
                              parent_node: ECBSNode) -> Dict[str, Any]:
        """分析冲突类型和上下文"""
        analysis = {
            'type': conflict.conflict_type.value,
            'severity': conflict.severity,
            'location': conflict.location,
            'time': conflict.time,
            'agents_involved': conflict.agents
        }
        
        # 获取涉及车辆的优先级
        priorities = {}
        for agent in conflict.agents:
            # 从parent_node的solution中获取优先级信息
            priorities[agent] = 0.5  # 默认优先级
            
        analysis['priorities'] = priorities
        
        # 检查是否为不可避免的冲突
        if conflict.conflict_type == ConflictType.NARROW_PASSAGE:
            # 检查是否所有备选路径都被占用
            all_paths_blocked = self._check_all_paths_blocked(conflict, parent_node)
            analysis['unavoidable'] = all_paths_blocked
            
        return analysis
    
    def _try_path_switch_strategy(self, parent_node: ECBSNode, conflict: ECBSConflict,
                                agent_requests: Dict[str, Dict],
                                backbone_allocations: Dict[str, str]) -> List[ECBSNode]:
        """尝试路径切换策略"""
        child_nodes = []
        
        for agent in conflict.agents:
            # 查找替代骨干路径
            current_path_id = backbone_allocations.get(agent) if backbone_allocations else None
            used_paths = {current_path_id} if current_path_id else set()
            
            # 添加已尝试过的路径
            if agent in parent_node.path_switches:
                used_paths.add(parent_node.path_switches[agent])
                
            request = agent_requests[agent]
            goal_info = self._extract_goal_info(request)
            
            alternative = self.path_switcher.find_alternative_backbone_path(
                agent, request['start'], goal_info, 
                {'conflict_type': conflict.conflict_type, 'location': conflict.location},
                used_paths
            )
            
            if alternative:
                # 创建新节点，使用替代路径
                child_node = self._create_path_switch_node(
                    parent_node, agent, alternative, agent_requests, backbone_allocations
                )
                
                if child_node:
                    child_nodes.append(child_node)
                    self.enhanced_stats['path_switches'] += 1
                    
        return child_nodes
    
    def _try_parking_strategy(self, parent_node: ECBSNode, conflict: ECBSConflict,
                            agent_requests: Dict[str, Dict],
                            conflict_analysis: Dict) -> List[ECBSNode]:
        """尝试停车等待策略"""
        child_nodes = []
        
        # 决定哪些车辆应该停车
        for agent in conflict.agents:
            should_park, parking_decision = self.parking_decision_maker.should_park_and_wait(
                agent, conflict_analysis
            )
            
            if should_park and parking_decision:
                # 创建停车节点
                child_node = self._create_parking_node(
                    parent_node, agent, parking_decision, agent_requests
                )
                
                if child_node:
                    # 模拟停车结果
                    simulation = self.parking_decision_maker.simulate_parking_outcome(
                        parking_decision
                    )
                    
                    # 如果预测冲突会解决，添加节点
                    if simulation['conflicts_resolved'] or simulation['confidence'] > 0.7:
                        child_nodes.append(child_node)
                        self.enhanced_stats['parking_decisions'] += 1
                        
        return child_nodes
    
    def _try_traditional_constraints(self, parent_node: ECBSNode, conflict: ECBSConflict,
                                   agent_requests: Dict[str, Dict],
                                   backbone_allocations: Dict[str, str]) -> List[ECBSNode]:
        """使用传统约束方法"""
        child_nodes = []
        
        for agent in conflict.agents:
            child_node = ECBSNode()
            child_node.constraints = parent_node.constraints.copy()
            child_node.parking_decisions = parent_node.parking_decisions.copy()
            child_node.path_switches = parent_node.path_switches.copy()
            
            # 生成约束
            new_constraint = self._generate_enhanced_constraint(conflict, agent)
            if new_constraint:
                child_node.constraints.add(new_constraint)
                self.constraint_manager.add_constraint(new_constraint)
                self.stats['constraints_generated'] += 1
                
                # 重新规划
                child_node.solution = parent_node.solution.copy()
                
                new_path = self._replan_with_enhanced_constraints(
                    agent, agent_requests[agent], child_node.constraints,
                    backbone_allocations, child_node.parking_decisions
                )
                
                if new_path:
                    child_node.solution[agent] = new_path
                    child_node.cost = sum(len(path) for path in child_node.solution.values())
                    
                    # 检测新冲突
                    backbone_info = self._extract_backbone_info(child_node.solution, backbone_allocations)
                    child_node.conflicts = self.conflict_detector.detect_all_conflicts(
                        child_node.solution, backbone_info
                    )
                    child_node.h_value = len(child_node.conflicts)
                    child_node.f_value = child_node.cost + child_node.h_value
                    
                    child_nodes.append(child_node)
                    
        return child_nodes
    
    def _create_path_switch_node(self, parent_node: ECBSNode, agent: str,
                               alternative: Dict, agent_requests: Dict[str, Dict],
                               backbone_allocations: Dict[str, str]) -> Optional[ECBSNode]:
        """创建路径切换节点"""
        child_node = ECBSNode()
        child_node.constraints = parent_node.constraints.copy()
        child_node.parking_decisions = parent_node.parking_decisions.copy()
        child_node.path_switches = parent_node.path_switches.copy()
        
        # 记录路径切换
        child_node.path_switches[agent] = alternative['path_id']
        
        # 添加路径切换约束，防止切换回原路径
        if agent in backbone_allocations:
            switch_constraint = ECBSConstraint(
                constraint_type=ConstraintType.PATH_SWITCH,
                agent=agent,
                backbone_id=backbone_allocations[agent]
            )
            child_node.constraints.add(switch_constraint)
            
        # 使用新路径重新规划
        request = agent_requests[agent]
        new_backbone_allocations = backbone_allocations.copy() if backbone_allocations else {}
        new_backbone_allocations[agent] = alternative['path_id']
        
        # 规划新路径
        new_path = self._plan_with_specific_backbone(
            agent, request, alternative['path_id'], alternative['access_point']
        )
        
        if new_path:
            child_node.solution = parent_node.solution.copy()
            child_node.solution[agent] = new_path
            child_node.cost = sum(len(path) for path in child_node.solution.values())
            
            # 检测新冲突
            backbone_info = self._extract_backbone_info(child_node.solution, new_backbone_allocations)
            child_node.conflicts = self.conflict_detector.detect_all_conflicts(
                child_node.solution, backbone_info
            )
            child_node.h_value = len(child_node.conflicts)
            child_node.f_value = child_node.cost + child_node.h_value
            
            return child_node
            
        return None
    
    def _create_parking_node(self, parent_node: ECBSNode, agent: str,
                           parking_decision: ParkingDecision,
                           agent_requests: Dict[str, Dict]) -> Optional[ECBSNode]:
        """创建停车节点"""
        child_node = ECBSNode()
        child_node.constraints = parent_node.constraints.copy()
        child_node.parking_decisions = parent_node.parking_decisions.copy()
        child_node.path_switches = parent_node.path_switches.copy()
        
        # 添加停车决策
        child_node.parking_decisions[agent] = parking_decision
        
        # 生成包含停车的路径
        request = agent_requests[agent]
        parking_path = self._generate_parking_path(
            agent, request['start'], request['goal'], parking_decision
        )
        
        if parking_path:
            child_node.solution = parent_node.solution.copy()
            child_node.solution[agent] = parking_path
            child_node.cost = sum(len(path) for path in child_node.solution.values())
            
            # 添加停车成本（鼓励尽量少停车）
            child_node.cost += parking_decision.parking_duration * 0.5
            
            # 检测新冲突
            backbone_info = self._extract_backbone_info(child_node.solution, None)
            child_node.conflicts = self.conflict_detector.detect_all_conflicts(
                child_node.solution, backbone_info
            )
            
            # 降低停车后的启发式值（因为停车可以解决很多冲突）
            child_node.h_value = len(child_node.conflicts) * 0.7
            child_node.f_value = child_node.cost + child_node.h_value
            
            return child_node
            
        return None
    
    def _generate_enhanced_constraint(self, conflict: ECBSConflict, 
                                    agent: str) -> Optional[ECBSConstraint]:
        """生成增强约束"""
        if conflict.conflict_type == ConflictType.NODE_OCCUPATION:
            # 节点占用约束
            return BackboneNodeConstraint(
                node_id=conflict.node_id,
                backbone_path_id=conflict.backbone_id,
                time_window=(conflict.time - 2, conflict.time + 2),
                agent=agent
            )
            
        elif conflict.conflict_type == ConflictType.NARROW_PASSAGE:
            # 骨干路径时间窗口约束
            return ECBSConstraint(
                constraint_type=ConstraintType.BACKBONE,
                agent=agent,
                backbone_id=conflict.backbone_id,
                time_window=(conflict.time - 5, conflict.time + 5)
            )
            
        else:
            # 使用父类的约束生成
            return self._generate_constraint_for_conflict(conflict, agent)
    
    def _replan_with_enhanced_constraints(self, agent: str, request: Dict,
                                        constraints: Set[ECBSConstraint],
                                        backbone_allocations: Dict[str, str],
                                        parking_decisions: Dict[str, ParkingDecision]) -> Optional[List[Tuple]]:
        """在增强约束下重新规划"""
        # 检查是否有停车决策
        if agent in parking_decisions:
            parking_decision = parking_decisions[agent]
            return self._generate_parking_path(
                agent, request['start'], request['goal'], parking_decision
            )
            
        # 尝试使用备选骨干路径
        goal_type, goal_id = self._parse_goal_info(request['goal'])
        key = (None, None, goal_type, goal_id)
        
        if key in self.backbone_alternatives:
            alternatives = self.backbone_alternatives[key]
            current_backbone = backbone_allocations.get(agent) if backbone_allocations else None
            
            # 尝试每个备选路径
            for alt_backbone_id in alternatives:
                if alt_backbone_id == current_backbone:
                    continue
                    
                # 检查路径切换约束
                if self._violates_path_switch_constraint(agent, alt_backbone_id, constraints):
                    continue
                    
                # 尝试使用该骨干路径规划
                path = self._plan_with_specific_backbone(
                    agent, request, alt_backbone_id, None
                )
                
                if path:
                    # 检查是否违反约束
                    backbone_info = {'backbone_id': alt_backbone_id}
                    if not self.constraint_manager.violates_constraint(agent, path, backbone_info):
                        return path
                        
        # 如果都失败，使用父类方法
        return super()._replan_with_constraints(agent, request, constraints, backbone_allocations)
    
    def _plan_with_specific_backbone(self, agent: str, request: Dict,
                                   backbone_id: str, access_point: Optional[Dict]) -> Optional[List[Tuple]]:
        """使用特定骨干路径规划"""
        if not self.path_planner or not self.backbone_network:
            return None
            
        try:
            # 这里应该调用path_planner的特定接口
            # 简化实现：直接使用path_planner
            result = self.path_planner.plan_path(
                vehicle_id=agent,
                start=request['start'],
                goal=request['goal'],
                use_backbone=True,
                context='ecbs_coordination'
            )
            
            if result:
                if isinstance(result, tuple):
                    return result[0]
                else:
                    return result
                    
        except Exception as e:
            print(f"规划失败 {agent}: {e}")
            
        return None
    
    def _generate_parking_path(self, agent: str, start: Tuple, goal: Tuple,
                             parking_decision: ParkingDecision) -> Optional[List[Tuple]]:
        """生成包含停车的路径"""
        if not self.path_planner:
            return None
            
        try:
            # 1. 规划到停车位置的路径
            to_parking = self.path_planner.plan_path(
                vehicle_id=agent,
                start=start,
                goal=parking_decision.parking_position,
                use_backbone=False,
                context='parking'
            )
            
            if not to_parking:
                return None
                
            if isinstance(to_parking, tuple):
                to_parking = to_parking[0]
                
            # 2. 停车等待（通过重复位置表示）
            parking_steps = int(parking_decision.parking_duration)
            wait_path = [parking_decision.parking_position] * parking_steps
            
            # 3. 从停车位置到目标的路径
            from_parking = self.path_planner.plan_path(
                vehicle_id=agent,
                start=parking_decision.parking_position,
                goal=goal,
                use_backbone=True,
                context='resume'
            )
            
            if not from_parking:
                return None
                
            if isinstance(from_parking, tuple):
                from_parking = from_parking[0]
                
            # 合并路径
            complete_path = to_parking[:-1] + wait_path + from_parking
            
            return complete_path
            
        except Exception as e:
            print(f"生成停车路径失败 {agent}: {e}")
            return None
    
    def _check_all_paths_blocked(self, conflict: ECBSConflict, 
                               parent_node: ECBSNode) -> bool:
        """检查是否所有路径都被阻塞"""
        # 简化实现
        if conflict.conflict_type == ConflictType.NARROW_PASSAGE:
            # 检查骨干路径的负载
            if self.backbone_network and conflict.backbone_id:
                path_data = self.backbone_network.bidirectional_paths.get(conflict.backbone_id)
                if path_data and path_data.get_load_factor() > 0.8:
                    return True
                    
        return False
    
    def _violates_path_switch_constraint(self, agent: str, backbone_id: str,
                                       constraints: Set[ECBSConstraint]) -> bool:
        """检查是否违反路径切换约束"""
        for constraint in constraints:
            if (constraint.constraint_type == ConstraintType.PATH_SWITCH and
                constraint.agent == agent and constraint.backbone_id == backbone_id):
                return True
        return False
    
    def _parse_goal_info(self, goal: Any) -> Tuple[Optional[str], Optional[int]]:
        """解析目标信息"""
        # 这里需要根据实际的目标格式解析
        # 简化实现
        return ('unloading', 0)
    
    def _extract_goal_info(self, request: Dict) -> Dict:
        """提取目标信息"""
        goal_type, goal_id = self._parse_goal_info(request.get('goal'))
        return {
            'type': goal_type,
            'id': goal_id,
            'position': request.get('goal')
        }


class OptimizedTrafficManagerWithECBS:
    """集成完整BA-ECBS的优化交通管理器"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # 使用增强的ECBS求解器
        self.ecbs_solver = BackboneAwareECBSSolver(env, path_planner, backbone_network)
        
        # 设置parking_decision_maker的traffic_manager引用
        self.ecbs_solver.parking_decision_maker.traffic_manager = self
        
        # 原有组件
        self.vehicle_predictions = {}
        self.prediction_horizon = 30
        self.active_paths = {}
        self.path_reservations = {}
        self.vehicle_priorities = {}
        
        # 增强的冲突消解器
        self.use_enhanced_resolution = True
        self.init_enhanced_resolver()
        
        # 车辆参数
        self.vehicle_params = {
            'length': 6.0,
            'width': 3.0,
            'safety_margin': 1.5
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
            'ecbs_coordinations': 0,
            'ecbs_success_rate': 0.0,
            'average_solve_time': 0.0,
            'constraint_violations': 0,
            'backbone_conflicts': 0,
            'priority_adjustments': 0,
            'path_switches': 0,
            'parking_maneuvers': 0,
            'backbone_switches': 0,
            'constraints_generated': 0
        }
        
        print("初始化集成完整BA-ECBS的优化交通管理器")
    
    def init_enhanced_resolver(self):
        """初始化增强冲突消解器"""
        try:
            from enhanced_conflict_resolver import EnhancedConflictResolver
            self.enhanced_resolver = EnhancedConflictResolver(
                self, self.backbone_network, self.path_planner
            )
            self.use_enhanced_resolution = True
            print("✅ 增强冲突消解器已启用")
        except ImportError:
            self.enhanced_resolver = None
            self.use_enhanced_resolution = False
            print("⚠️ 增强冲突消解器不可用，使用BA-ECBS")
    
    def ecbs_coordinate_paths(self, vehicle_requests: Dict[str, Dict],
                            backbone_allocations: Dict[str, str] = None,
                            max_solve_time: float = 30.0) -> Dict[str, Any]:
        """使用增强的BA-ECBS协调多车辆路径规划"""
        coordination_start = time.time()
        self.stats['ecbs_coordinations'] += 1
        
        try:
            print(f"开始BA-ECBS多车辆协调: {len(vehicle_requests)} 个车辆")
            
            # 考虑车辆优先级
            self._adjust_requests_by_priority(vehicle_requests)
            
            # 设置求解器参数
            self.ecbs_solver.timeout = max_solve_time
            
            # 执行BA-ECBS求解
            result = self.ecbs_solver.solve_with_backbone_awareness(
                vehicle_requests, backbone_allocations
            )
            
            if result.get('success', False):
                # 注册协调后的路径
                self._register_coordinated_paths(result)
                
                # 更新统计
                self._update_ecbs_statistics(True, result)
                
                print(f"✅ BA-ECBS协调成功: 最终冲突{result.get('final_conflicts', 0)}个, "
                      f"路径切换{result.get('enhanced_stats', {}).get('path_switches', 0)}次, "
                      f"停车决策{result.get('enhanced_stats', {}).get('parking_decisions', 0)}个")
                
                return result
            else:
                self._update_ecbs_statistics(False, result)
                print(f"❌ BA-ECBS协调失败: {result.get('error', 'Unknown error')}")
                return result
                
        except Exception as e:
            print(f"BA-ECBS协调异常: {e}")
            import traceback
            traceback.print_exc()
            return {'success': False, 'error': f'协调异常: {str(e)}'}
    
    def _register_coordinated_paths(self, result: Dict):
        """注册协调后的路径"""
        paths = result.get('paths', {})
        structures = result.get('structures', {})
        parking_decisions = result.get('parking_decisions', {})
        
        for vehicle_id, path in paths.items():
            # 注册路径
            self.register_vehicle_path(
                vehicle_id, path, time.time(), 
                path_structure=structures.get(vehicle_id)
            )
            
            # 处理停车决策
            if vehicle_id in parking_decisions:
                self._apply_parking_decision(vehicle_id, parking_decisions[vehicle_id])
                
            # 更新节点预约
            if hasattr(self.ecbs_solver, 'node_time_manager'):
                self.ecbs_solver.node_time_manager.reserve_node_sequence(
                    path, vehicle_id, time.time()
                )
    
    def _apply_parking_decision(self, vehicle_id: str, parking_decision: ParkingDecision):
        """应用停车决策"""
        # 记录停车信息
        if not hasattr(self, 'parking_vehicles'):
            self.parking_vehicles = {}
            
        self.parking_vehicles[vehicle_id] = {
            'position': parking_decision.parking_position,
            'duration': parking_decision.parking_duration,
            'start_time': parking_decision.start_time,
            'reason': parking_decision.reason,
            'priority_vehicle': parking_decision.priority_vehicle_id
        }
        
        print(f"车辆 {vehicle_id} 执行停车决策: {parking_decision.reason}")
        
        # 通知调度器
        if hasattr(self, 'vehicle_scheduler') and self.vehicle_scheduler:
            self.vehicle_scheduler.handle_parking_decision(vehicle_id, parking_decision.__dict__)
    
    def register_vehicle_path(self, vehicle_id: str, path: List, 
                            start_time: float = 0, speed: float = 1.0,
                            path_structure: Dict = None) -> bool:
        """注册车辆路径 - 增强版"""
        if not path or len(path) < 2:
            return False
        
        with self.state_lock:
            # 移除旧路径
            self.release_vehicle_path(vehicle_id)
            
            # 获取车辆优先级
            priority = self.get_vehicle_priority(vehicle_id)
            
            # 注册新路径
            path_info = {
                'path': path,
                'start_time': start_time,
                'speed': speed,
                'priority': priority,
                'registered_time': time.time(),
                'coordination_method': 'ba_ecbs',
                'path_structure': path_structure or {}
            }
            
            self.active_paths[vehicle_id] = path_info
            
            # 更新骨干网络负载
            if path_structure and 'backbone_id' in path_structure:
                backbone_id = path_structure['backbone_id']
                if self.backbone_network and backbone_id in self.backbone_network.bidirectional_paths:
                    self.backbone_network.bidirectional_paths[backbone_id].add_vehicle(vehicle_id)
                    
            return True
    
    def detect_all_conflicts(self) -> List:
        """检测所有冲突 - 使用增强检测器"""
        if not self.active_paths:
            return []
        
        # 准备数据
        solution = {vid: info['path'] for vid, info in self.active_paths.items()}
        backbone_info = {}
        
        for vid, info in self.active_paths.items():
            structure = info.get('path_structure', {})
            if 'backbone_id' in structure:
                backbone_info[vid] = {
                    'backbone_id': structure['backbone_id'],
                    'usage_start_time': 0,
                    'usage_end_time': len(info['path'])
                }
                
        # 使用增强检测器
        conflicts = self.ecbs_solver.conflict_detector.detect_all_conflicts(
            solution, backbone_info
        )
        
        # 更新统计
        self.stats['total_conflicts'] += len(conflicts)
        
        # 统计骨干路径相关冲突
        backbone_conflicts = [c for c in conflicts if c.conflict_type in [
            ConflictType.NODE_OCCUPATION, ConflictType.NARROW_PASSAGE, 
            ConflictType.ACCESS_POINT, ConflictType.BACKBONE_CONGESTION
        ]]
        self.stats['backbone_conflicts'] += len(backbone_conflicts)
        
        # 转换为兼容格式
        return self._convert_conflicts_to_compatible_format(conflicts)
    
    def _convert_conflicts_to_compatible_format(self, conflicts: List[ECBSConflict]) -> List:
        """转换冲突格式以保持兼容性"""
        converted_conflicts = []
        
        for conflict in conflicts:
            converted = type('Conflict', (), {
                'conflict_id': conflict.conflict_id,
                'agents': conflict.agents,
                'location': conflict.location,
                'time_step': conflict.time,
                'conflict_type': conflict.conflict_type,
                'severity': conflict.severity,
                'backbone_id': getattr(conflict, 'backbone_id', None),
                'node_id': getattr(conflict, 'node_id', None)
            })()
            converted_conflicts.append(converted)
            
        return converted_conflicts
    
    def resolve_conflicts(self, conflicts: List) -> Dict[str, List]:
        """解决冲突 - 优先使用BA-ECBS"""
        if not conflicts:
            return {vid: info['path'] for vid, info in self.active_paths.items()}
        
        print(f"检测到 {len(conflicts)} 个冲突，开始BA-ECBS冲突消解...")
        
        # 准备车辆请求
        vehicle_requests = {}
        backbone_allocations = {}
        
        involved_vehicles = set()
        for conflict in conflicts:
            if hasattr(conflict, 'agents'):
                involved_vehicles.update(conflict.agents)
                
        for vehicle_id in involved_vehicles:
            if vehicle_id in self.active_paths:
                path_info = self.active_paths[vehicle_id]
                path = path_info['path']
                
                vehicle_requests[vehicle_id] = {
                    'start': path[0],
                    'goal': path[-1],
                    'priority': self.get_vehicle_priority(vehicle_id)
                }
                
                # 提取骨干分配信息
                structure = path_info.get('path_structure', {})
                if 'backbone_id' in structure:
                    backbone_allocations[vehicle_id] = structure['backbone_id']
                    
        # 执行BA-ECBS协调
        coordination_result = self.ecbs_coordinate_paths(
            vehicle_requests, backbone_allocations, max_solve_time=30.0
        )
        
        if coordination_result.get('success', False):
            # 获取协调后的路径
            coordinated_paths = coordination_result.get('paths', {})
            current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
            
            # 更新路径
            for vehicle_id, new_path in coordinated_paths.items():
                current_paths[vehicle_id] = new_path
                
            self.stats['resolved_conflicts'] += len(conflicts)
            
            # 更新增强统计
            enhanced_stats = coordination_result.get('enhanced_stats', {})
            self.stats['path_switches'] += enhanced_stats.get('path_switches', 0)
            self.stats['parking_maneuvers'] += enhanced_stats.get('parking_decisions', 0)
            
            print(f"✅ BA-ECBS成功解决 {len(conflicts)} 个冲突")
            return current_paths
        else:
            print(f"❌ BA-ECBS冲突解决失败，保持原路径")
            return {vid: info['path'] for vid, info in self.active_paths.items()}
    
    def release_vehicle_path(self, vehicle_id: str) -> bool:
        """释放车辆路径 - 增强版"""
        with self.state_lock:
            if vehicle_id not in self.active_paths:
                return False
            
            path_info = self.active_paths[vehicle_id]
            
            # 释放骨干网络资源
            structure = path_info.get('path_structure', {})
            if 'backbone_id' in structure:
                backbone_id = structure['backbone_id']
                if self.backbone_network and backbone_id in self.backbone_network.bidirectional_paths:
                    self.backbone_network.bidirectional_paths[backbone_id].remove_vehicle(vehicle_id)
                    
            # 释放节点预约
            if hasattr(self.ecbs_solver, 'node_time_manager'):
                self.ecbs_solver.node_time_manager.release_node_reservations(vehicle_id)
                
            del self.active_paths[vehicle_id]
            
            if vehicle_id in self.vehicle_predictions:
                del self.vehicle_predictions[vehicle_id]
                
            return True
    
    def update(self, time_delta: float):
        """更新状态"""
        # 更新停车车辆
        if hasattr(self, 'parking_vehicles'):
            current_time = time.time()
            completed = []
            
            for vehicle_id, parking_info in self.parking_vehicles.items():
                if current_time - parking_info['start_time'] > parking_info['duration']:
                    completed.append(vehicle_id)
                    print(f"车辆 {vehicle_id} 完成停车等待")
                    
            for vehicle_id in completed:
                del self.parking_vehicles[vehicle_id]
                
        # 清理过期的节点预约
        if hasattr(self.ecbs_solver, 'node_time_manager'):
            # 这里可以添加清理逻辑
            pass
    
    def get_statistics(self) -> Dict:
        """获取增强统计信息"""
        base_stats = self.stats.copy()
        
        # 添加BA-ECBS特定统计
        if hasattr(self.ecbs_solver, 'enhanced_stats'):
            base_stats['ba_ecbs'] = self.ecbs_solver.enhanced_stats.copy()
            
        # 添加当前状态
        base_stats['active_parking_vehicles'] = len(getattr(self, 'parking_vehicles', {}))
        base_stats['active_paths'] = len(self.active_paths)
        
        return base_stats
    
    # ========== 保留原有接口方法 ==========
    
    def set_vehicle_priority(self, vehicle_id: str, priority: float) -> bool:
        """设置车辆优先级"""
        try:
            priority = max(0.0, min(1.0, priority))
            
            with self.state_lock:
                old_priority = self.vehicle_priorities.get(vehicle_id, 0.5)
                self.vehicle_priorities[vehicle_id] = priority
                
                if abs(old_priority - priority) > 0.1:
                    self.stats['priority_adjustments'] += 1
                    
            return True
            
        except Exception as e:
            print(f"设置车辆优先级失败 {vehicle_id}: {e}")
            return False
    
    def get_vehicle_priority(self, vehicle_id: str) -> float:
        """获取车辆优先级"""
        return self.vehicle_priorities.get(vehicle_id, 0.5)
    
    def _adjust_requests_by_priority(self, vehicle_requests: Dict[str, Dict]):
        """根据车辆优先级调整请求"""
        for vehicle_id, request in vehicle_requests.items():
            vehicle_priority = self.get_vehicle_priority(vehicle_id)
            
            if 'priority' not in request:
                request['priority'] = vehicle_priority
            else:
                request['priority'] = (request['priority'] * 0.7 + vehicle_priority * 0.3)
    
    def _update_ecbs_statistics(self, success: bool, result: Dict):
        """更新ECBS统计信息"""
        solve_time = result.get('solve_time', 0.0)
        
        # 更新成功率
        current_rate = self.stats['ecbs_success_rate']
        total_coords = self.stats['ecbs_coordinations']
        
        if success:
            new_rate = (current_rate * (total_coords - 1) + 1.0) / total_coords
        else:
            new_rate = (current_rate * (total_coords - 1)) / total_coords
            
        self.stats['ecbs_success_rate'] = new_rate
        
        # 更新平均求解时间
        current_avg = self.stats['average_solve_time']
        new_avg = (current_avg * (total_coords - 1) + solve_time) / total_coords
        self.stats['average_solve_time'] = new_avg
        
        # 更新其他统计
        if 'enhanced_stats' in result:
            enhanced = result['enhanced_stats']
            self.stats['constraints_generated'] += result.get('constraints', 0)
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.ecbs_solver.backbone_network = backbone_network
        
        if hasattr(self.ecbs_solver, 'node_time_manager'):
            self.ecbs_solver.node_time_manager.backbone_network = backbone_network
        if hasattr(self.ecbs_solver, 'path_switcher'):
            self.ecbs_solver.path_switcher.backbone_network = backbone_network
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        self.ecbs_solver.path_planner = path_planner
        
        if hasattr(self.ecbs_solver, 'path_switcher'):
            self.ecbs_solver.path_switcher.path_planner = path_planner
    
    def clear_all(self):
        """清理所有数据"""
        with self.state_lock:
            self.active_paths.clear()
            self.vehicle_predictions.clear()
            self.path_reservations.clear()
            self.vehicle_priorities.clear()
            
            if hasattr(self, 'parking_vehicles'):
                self.parking_vehicles.clear()
                
            if hasattr(self.ecbs_solver, 'node_time_manager'):
                self.ecbs_solver.node_time_manager.node_reservations.clear()
    
    def shutdown(self):
        """关闭管理器"""
        self.clear_all()
        print("集成BA-ECBS的优化交通管理器已关闭")


# 向后兼容性
OptimizedTrafficManager = OptimizedTrafficManagerWithECBS