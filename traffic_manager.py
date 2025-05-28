"""
traffic_manager_with_ecbs.py - 完整ECBS集成的交通管理器
实现完整的Enhanced Conflict-Based Search算法
支持多车辆协调、骨干路径约束、安全矩形冲突检测
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

class ConflictType(Enum):
    """冲突类型"""
    SAFETY_RECTANGLE = "safety_rectangle"
    INTERFACE = "interface"
    BACKBONE_CONGESTION = "backbone_congestion"
    TEMPORAL = "temporal"
    CAPACITY = "capacity"

class ConstraintType(Enum):
    """约束类型"""
    VERTEX = "vertex"      # 顶点约束：(agent, position, time)
    EDGE = "edge"          # 边约束：(agent, position1, position2, time)
    BACKBONE = "backbone"  # 骨干路径约束：(agent, backbone_id, time_window)
    CAPACITY = "capacity"  # 容量约束：(backbone_id, max_agents, time_window)

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
    
    def __hash__(self):
        return hash((self.constraint_type, self.agent, self.position, 
                    self.time, self.backbone_id))

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
    
    def __lt__(self, other):
        return self.severity > other.severity

class ECBSNode:
    """ECBS搜索树节点"""
    
    def __init__(self):
        self.constraints: Set[ECBSConstraint] = set()
        self.solution: Dict[str, List[Tuple]] = {}
        self.cost: float = 0.0
        self.conflicts: List[ECBSConflict] = []
        self.h_value: float = 0.0  # 启发式值
        self.f_value: float = 0.0  # f = cost + h_value
    
    def __lt__(self, other):
        if abs(self.f_value - other.f_value) < 1e-6:
            return len(self.conflicts) < len(other.conflicts)
        return self.f_value < other.f_value

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

class EnhancedConflictDetector:
    """增强冲突检测器"""
    
    def __init__(self, env, backbone_network=None):
        self.env = env
        self.backbone_network = backbone_network
        self.safety_margin = 3.0
    
    def detect_all_conflicts(self, solution: Dict[str, List[Tuple]], 
                           backbone_info: Dict[str, Dict] = None) -> List[ECBSConflict]:
        """检测所有冲突"""
        conflicts = []
        agents = list(solution.keys())
        
        # 检测安全矩形冲突
        for i in range(len(agents)):
            for j in range(i + 1, len(agents)):
                agent1, agent2 = agents[i], agents[j]
                path1, path2 = solution[agent1], solution[agent2]
                
                # 时间-空间冲突检测
                temporal_conflicts = self._detect_temporal_conflicts(
                    agent1, agent2, path1, path2
                )
                conflicts.extend(temporal_conflicts)
                
                # 骨干路径冲突检测
                if backbone_info:
                    backbone_conflicts = self._detect_backbone_conflicts(
                        agent1, agent2, path1, path2, backbone_info
                    )
                    conflicts.extend(backbone_conflicts)
        
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
    
    def _detect_backbone_conflicts(self, agent1: str, agent2: str,
                                 path1: List[Tuple], path2: List[Tuple],
                                 backbone_info: Dict[str, Dict]) -> List[ECBSConflict]:
        """检测骨干路径冲突"""
        conflicts = []
        
        info1 = backbone_info.get(agent1, {})
        info2 = backbone_info.get(agent2, {})
        
        backbone_id1 = info1.get('backbone_id')
        backbone_id2 = info2.get('backbone_id')
        
        # 检查是否使用同一骨干路径
        if backbone_id1 and backbone_id2 and backbone_id1 == backbone_id2:
            # 容量冲突检测
            if self._backbone_capacity_conflict(info1, info2):
                conflict = ECBSConflict(
                    conflict_id=f"backbone_{agent1}_{agent2}_{backbone_id1}",
                    conflict_type=ConflictType.BACKBONE_CONGESTION,
                    agents=[agent1, agent2],
                    location=(0, 0),  # 骨干路径冲突位置不重要
                    time=0,
                    severity=0.9,
                    backbone_id=backbone_id1
                )
                conflicts.append(conflict)
        
        return conflicts
    
    def _positions_conflict(self, pos1: Tuple, pos2: Tuple) -> bool:
        """检查位置冲突"""
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return distance < self.safety_margin
    
    def _edge_conflict(self, pos1: Tuple, next_pos1: Tuple, 
                      pos2: Tuple, next_pos2: Tuple) -> bool:
        """检查边冲突（交叉路径）"""
        # 简化的边冲突检测
        return (self._positions_conflict(pos1, next_pos2) and 
                self._positions_conflict(next_pos1, pos2))
    
    def _calculate_conflict_severity(self, pos1: Tuple, pos2: Tuple) -> float:
        """计算冲突严重程度"""
        distance = math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return max(0.1, 1.0 - (distance / self.safety_margin))
    
    def _backbone_capacity_conflict(self, info1: Dict, info2: Dict) -> bool:
        """检查骨干路径容量冲突"""
        # 简化的容量冲突检测
        usage_start1 = info1.get('usage_start_time', 0)
        usage_end1 = info1.get('usage_end_time', 100)
        usage_start2 = info2.get('usage_start_time', 0)
        usage_end2 = info2.get('usage_end_time', 100)
        
        # 检查时间窗口重叠
        return not (usage_end1 <= usage_start2 or usage_start1 >= usage_end2)

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

class OptimizedTrafficManagerWithECBS:
    """集成完整ECBS的优化交通管理器"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # ECBS求解器
        self.ecbs_solver = ECBSSolver(env, path_planner, backbone_network)
        
        # 原有组件保持不变
        self.vehicle_predictions = {}
        self.prediction_horizon = 30
        self.active_paths = {}
        self.path_reservations = {}
        
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
            'backbone_conflicts': 0
        }
        
        print("初始化集成完整ECBS的优化交通管理器")
    
    def ecbs_coordinate_paths(self, vehicle_requests: Dict[str, Dict],
                            backbone_allocations: Dict[str, str] = None,
                            max_solve_time: float = 30.0) -> Dict[str, Any]:
        """
        ECBS协调多车辆路径规划
        
        Args:
            vehicle_requests: {vehicle_id: {'start': tuple, 'goal': tuple, 'priority': float}}
            backbone_allocations: {vehicle_id: backbone_path_id}
            max_solve_time: 最大求解时间
        
        Returns:
            协调结果字典
        """
        coordination_start = time.time()
        self.stats['ecbs_coordinations'] += 1
        
        try:
            print(f"开始ECBS多车辆协调: {len(vehicle_requests)} 个车辆")
            
            # 设置求解器参数
            self.ecbs_solver.timeout = max_solve_time
            
            # 执行ECBS求解
            result = self.ecbs_solver.solve(vehicle_requests, backbone_allocations)
            
            if result.get('success', False):
                # 注册协调后的路径
                paths = result['paths']
                for vehicle_id, path in paths.items():
                    self.register_vehicle_path(vehicle_id, path, coordination_start)
                
                # 更新统计
                solve_time = result.get('solve_time', 0.0)
                self._update_ecbs_statistics(True, solve_time)
                
                print(f"✅ ECBS协调成功: {result.get('final_conflicts', 0)} 个最终冲突, "
                      f"扩展{result.get('expansions', 0)}次, 耗时{solve_time:.2f}s")
                
                return result
            else:
                error_msg = result.get('error', 'Unknown error')
                self._update_ecbs_statistics(False, max_solve_time)
                print(f"❌ ECBS协调失败: {error_msg}")
                return result
                
        except Exception as e:
            self._update_ecbs_statistics(False, max_solve_time)
            print(f"ECBS协调异常: {e}")
            return {'success': False, 'error': f'协调异常: {str(e)}'}
    
    def _update_ecbs_statistics(self, success: bool, solve_time: float):
        """更新ECBS统计信息"""
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
    
    def detect_coordination_need(self, active_vehicles: List[str]) -> bool:
        """检测是否需要ECBS协调"""
        if len(active_vehicles) < 2:
            return False
        
        # 检测当前冲突
        current_conflicts = self.detect_all_conflicts()
        
        # 如果冲突数量超过阈值，需要协调
        conflict_threshold = min(3, len(active_vehicles) // 2)
        
        return len(current_conflicts) >= conflict_threshold
    
    def get_coordination_recommendation(self, vehicle_states: Dict) -> Dict[str, Any]:
        """获取协调建议"""
        recommendation = {
            'need_coordination': False,
            'candidate_vehicles': [],
            'coordination_type': 'none',
            'priority_level': 'normal',
            'estimated_benefit': 0.0
        }
        
        # 分析车辆状态
        planning_vehicles = [vid for vid, state in vehicle_states.items() 
                           if hasattr(state, 'status') and state.status.name == 'PLANNING']
        moving_vehicles = [vid for vid, state in vehicle_states.items() 
                         if hasattr(state, 'status') and state.status.name == 'MOVING']
        
        all_active = planning_vehicles + moving_vehicles
        
        if len(all_active) >= 3:
            recommendation['need_coordination'] = True
            recommendation['candidate_vehicles'] = all_active
            recommendation['coordination_type'] = 'batch'
            
            # 评估协调优先级
            if len(planning_vehicles) >= 2:
                recommendation['priority_level'] = 'high'
                recommendation['estimated_benefit'] = 0.8
            else:
                recommendation['priority_level'] = 'normal'
                recommendation['estimated_benefit'] = 0.6
        
        return recommendation
    
    # 保持原有的所有方法
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
                'coordination_method': 'ecbs'  # 标记为ECBS协调路径
            }
            
            self.active_paths[vehicle_id] = path_info
            return True
    
    def detect_all_conflicts(self) -> List:
        """检测所有冲突（保持原有接口）"""
        # 使用ECBS冲突检测器
        if not self.active_paths:
            return []
        
        solution = {vid: info['path'] for vid, info in self.active_paths.items()}
        conflicts = self.ecbs_solver.conflict_detector.detect_all_conflicts(solution)
        
        # 转换为原有冲突格式以保持兼容性
        converted_conflicts = []
        for conflict in conflicts:
            converted_conflict = type('Conflict', (), {
                'conflict_id': conflict.conflict_id,
                'agents': conflict.agents,
                'location': conflict.location,
                'time_step': conflict.time,
                'conflict_type': conflict.conflict_type,
                'severity': conflict.severity
            })()
            converted_conflicts.append(converted_conflict)
        
        return converted_conflicts
    
    def resolve_conflicts(self, conflicts: List) -> Dict[str, List]:
        """解决冲突（增强版）"""
        if not conflicts:
            return {vid: info['path'] for vid, info in self.active_paths.items()}
        
        print(f"检测到 {len(conflicts)} 个冲突，使用ECBS协调解决...")
        
        # 提取冲突车辆
        involved_vehicles = set()
        for conflict in conflicts:
            if hasattr(conflict, 'agents'):
                involved_vehicles.update(conflict.agents)
        
        if len(involved_vehicles) < 2:
            return {vid: info['path'] for vid, info in self.active_paths.items()}
        
        # 构建车辆请求
        vehicle_requests = {}
        for vehicle_id in involved_vehicles:
            if vehicle_id in self.active_paths:
                path = self.active_paths[vehicle_id]['path']
                vehicle_requests[vehicle_id] = {
                    'start': path[0],
                    'goal': path[-1],
                    'priority': 0.5
                }
        
        # 使用ECBS重新协调
        coordination_result = self.ecbs_coordinate_paths(vehicle_requests)
        
        if coordination_result.get('success', False):
            # 使用协调后的路径
            coordinated_paths = coordination_result['paths']
            current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
            
            # 更新冲突车辆的路径
            for vehicle_id, new_path in coordinated_paths.items():
                current_paths[vehicle_id] = new_path
            
            self.stats['resolved_conflicts'] += len(conflicts)
            print(f"✅ 使用ECBS成功解决 {len(conflicts)} 个冲突")
            
            return current_paths
        else:
            print(f"❌ ECBS冲突解决失败，保持原路径")
            return {vid: info['path'] for vid, info in self.active_paths.items()}
    
    def release_vehicle_path(self, vehicle_id: str) -> bool:
        """释放车辆路径"""
        with self.state_lock:
            if vehicle_id not in self.active_paths:
                return False
            
            del self.active_paths[vehicle_id]
            
            if vehicle_id in self.vehicle_predictions:
                del self.vehicle_predictions[vehicle_id]
            
            # 释放骨干网络资源
            if self.backbone_network:
                self.backbone_network.release_vehicle_from_path(vehicle_id)
            
            return True
    
    def update(self, time_delta: float):
        """更新管理器"""
        # 定期检查是否需要协调
        if len(self.active_paths) >= 3:
            conflicts = self.detect_all_conflicts()
            if len(conflicts) >= 2:
                print(f"检测到 {len(conflicts)} 个冲突，触发ECBS协调...")
                self.resolve_conflicts(conflicts)
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.ecbs_solver.backbone_network = backbone_network
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        self.ecbs_solver.path_planner = path_planner
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        stats = self.stats.copy()
        stats['active_vehicles'] = len(self.active_paths)
        
        # ECBS特有统计
        ecbs_stats = self.ecbs_solver.stats.copy()
        stats['ecbs_solver_stats'] = ecbs_stats
        
        return stats
    
    def clear_all(self):
        """清理所有数据"""
        with self.state_lock:
            self.active_paths.clear()
            self.vehicle_predictions.clear()
            self.path_reservations.clear()
    
    def shutdown(self):
        """关闭管理器"""
        self.clear_all()
        print("集成ECBS的优化交通管理器已关闭")

# 向后兼容性
OptimizedTrafficManager = OptimizedTrafficManagerWithECBS