"""
traffic_manager.py - 精简优化版交通管理器
专注经典ECBS冲突解决，移除复杂预测功能
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

@dataclass
class SimpleConflict:
    """简化冲突表示"""
    conflict_id: str
    agent1: str
    agent2: str
    location: Tuple[float, float]
    time_step: int
    conflict_type: ConflictType = ConflictType.VERTEX
    
    def __lt__(self, other):
        """优先级比较 - 时间越早优先级越高"""
        return self.time_step < other.time_step

@dataclass
class SimpleConstraint:
    """简化约束条件"""
    agent_id: str
    constraint_type: ConflictType
    location: Tuple[float, float] = None
    time_step: int = None

class SimplifiedECBSSolver:
    """简化ECBS求解器 - 专注经典算法"""
    
    def __init__(self, env, path_planner=None):
        self.env = env
        self.path_planner = path_planner
        
        # 基本配置
        self.max_search_time = 15.0
        self.max_iterations = 500
        
        # 统计
        self.stats = {
            'conflicts_resolved': 0,
            'total_time': 0,
            'iterations': 0
        }
    
    def solve_conflicts(self, initial_paths: Dict[str, List], 
                       conflicts: List[SimpleConflict]) -> Dict[str, List]:
        """经典ECBS冲突解决"""
        if not conflicts:
            return initial_paths
        
        start_time = time.time()
        resolved_paths = initial_paths.copy()
        
        # 按优先级排序冲突
        sorted_conflicts = sorted(conflicts)
        
        resolved_count = 0
        for conflict in sorted_conflicts[:10]:  # 限制处理数量
            if time.time() - start_time > self.max_search_time:
                break
            
            success = self._resolve_single_conflict(conflict, resolved_paths)
            if success:
                resolved_count += 1
        
        # 更新统计
        self.stats['conflicts_resolved'] += resolved_count
        self.stats['total_time'] += time.time() - start_time
        
        return resolved_paths
    
    def _resolve_single_conflict(self, conflict: SimpleConflict, 
                                paths: Dict[str, List]) -> bool:
        """解决单个冲突"""
        # 选择重规划的智能体（选择较短路径的）
        agent1_path = paths.get(conflict.agent1, [])
        agent2_path = paths.get(conflict.agent2, [])
        
        if len(agent1_path) <= len(agent2_path):
            replan_agent = conflict.agent1
        else:
            replan_agent = conflict.agent2
        
        # 生成约束
        constraint = self._generate_constraint(conflict, replan_agent)
        
        # 重规划
        new_path = self._replan_with_constraint(replan_agent, paths, constraint)
        
        if new_path:
            paths[replan_agent] = new_path
            return True
        
        return False
    
    def _generate_constraint(self, conflict: SimpleConflict, 
                           agent: str) -> SimpleConstraint:
        """生成约束"""
        return SimpleConstraint(
            agent_id=agent,
            constraint_type=conflict.conflict_type,
            location=conflict.location,
            time_step=conflict.time_step
        )
    
    def _replan_with_constraint(self, agent: str, paths: Dict[str, List], 
                              constraint: SimpleConstraint) -> Optional[List]:
        """带约束重规划"""
        if not self.path_planner:
            return None
        
        current_path = paths.get(agent, [])
        if len(current_path) < 2:
            return None
        
        try:
            # 简单重规划（忽略约束细节，由规划器处理）
            result = self.path_planner.plan_path(
                agent, current_path[0], current_path[-1],
                use_backbone=True
            )
            
            if isinstance(result, tuple):
                return result[0]
            return result
        
        except Exception:
            return None

class OptimizedTrafficManager:
    """精简交通管理器 - 专注冲突解决"""
    
    def __init__(self, env, backbone_network=None, path_planner=None):
        # 核心组件
        self.env = env
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # 简化数据结构
        self.active_paths = {}           # {vehicle_id: path_info}
        self.path_reservations = {}      # 简单时空预留
        
        # ECBS求解器
        self.ecbs_solver = SimplifiedECBSSolver(env, path_planner)
        
        # 检测参数
        self.safety_distance = 7.0
        self.time_discretization = 1.0
        
        # 线程锁
        self.state_lock = threading.RLock()
        
        # 统计
        self.stats = {
            'total_conflicts': 0,
            'resolved_conflicts': 0,
            'detection_time': 0,
            'resolution_time': 0
        }
        
        print("初始化精简交通管理器")
    
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
                'registered_time': time.time()
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
    
    def detect_all_conflicts(self) -> List[SimpleConflict]:
        """检测所有冲突"""
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
                    vertex_conflicts = self._detect_vertex_conflicts(
                        vehicle1, path1, vehicle2, path2
                    )
                    conflicts.extend(vertex_conflicts)
                    
                    # 边冲突
                    edge_conflicts = self._detect_edge_conflicts(
                        vehicle1, path1, vehicle2, path2
                    )
                    conflicts.extend(edge_conflicts)
        
        # 记录统计
        detection_time = time.time() - detection_start
        self.stats['detection_time'] += detection_time
        self.stats['total_conflicts'] += len(conflicts)
        
        return conflicts
    
    def _detect_vertex_conflicts(self, agent1: str, path1: List, 
                               agent2: str, path2: List) -> List[SimpleConflict]:
        """检测顶点冲突"""
        conflicts = []
        min_len = min(len(path1), len(path2))
        
        for t in range(min_len):
            p1, p2 = path1[t], path2[t]
            distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
            if distance < self.safety_distance:
                conflict = SimpleConflict(
                    conflict_id=f"vertex_{agent1}_{agent2}_{t}",
                    agent1=agent1,
                    agent2=agent2,
                    location=((p1[0] + p2[0])/2, (p1[1] + p2[1])/2),
                    time_step=t,
                    conflict_type=ConflictType.VERTEX
                )
                conflicts.append(conflict)
        
        return conflicts
    
    def _detect_edge_conflicts(self, agent1: str, path1: List, 
                             agent2: str, path2: List) -> List[SimpleConflict]:
        """检测边冲突"""
        conflicts = []
        min_len = min(len(path1), len(path2)) - 1
        
        for t in range(min_len):
            # 检查位置交换
            if (self._points_close(path1[t], path2[t+1]) and 
                self._points_close(path1[t+1], path2[t])):
                
                center_x = (path1[t][0] + path2[t][0]) / 2
                center_y = (path1[t][1] + path2[t][1]) / 2
                
                conflict = SimpleConflict(
                    conflict_id=f"edge_{agent1}_{agent2}_{t}",
                    agent1=agent1,
                    agent2=agent2,
                    location=(center_x, center_y),
                    time_step=t,
                    conflict_type=ConflictType.EDGE
                )
                conflicts.append(conflict)
        
        return conflicts
    
    def _points_close(self, p1: Tuple, p2: Tuple) -> bool:
        """检查两点是否接近"""
        distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return distance < self.safety_distance
    
    def resolve_conflicts(self, conflicts: List[SimpleConflict]) -> Dict[str, List]:
        """解决冲突"""
        if not conflicts:
            return {vid: info['path'] for vid, info in self.active_paths.items()}
        
        resolution_start = time.time()
        
        # 获取当前路径
        current_paths = {vid: info['path'] for vid, info in self.active_paths.items()}
        
        # 使用ECBS求解
        resolved_paths = self.ecbs_solver.solve_conflicts(current_paths, conflicts)
        
        # 更新路径
        for vehicle_id, new_path in resolved_paths.items():
            if vehicle_id in self.active_paths:
                if self.active_paths[vehicle_id]['path'] != new_path:
                    self.active_paths[vehicle_id]['path'] = new_path
                    print(f"车辆 {vehicle_id} 路径已更新")
        
        # 记录统计
        resolution_time = time.time() - resolution_start
        self.stats['resolution_time'] += resolution_time
        self.stats['resolved_conflicts'] += len(conflicts)
        
        return resolved_paths
    
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
            return True
    
    def update(self, time_delta: float):
        """更新管理器"""
        # 定期检测和解决冲突
        conflicts = self.detect_all_conflicts()
        
        if conflicts:
            print(f"检测到 {len(conflicts)} 个冲突，开始解决...")
            self.resolve_conflicts(conflicts)
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        self.ecbs_solver.path_planner = path_planner
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        stats = self.stats.copy()
        stats['active_vehicles'] = len(self.active_paths)
        stats['active_reservations'] = len(self.path_reservations)
        
        # ECBS统计
        stats['ecbs_stats'] = self.ecbs_solver.stats.copy()
        
        return stats
    
    def clear_all(self):
        """清理所有数据"""
        with self.state_lock:
            self.active_paths.clear()
            self.path_reservations.clear()
    
    def shutdown(self):
        """关闭管理器"""
        self.clear_all()
        print("交通管理器已关闭")