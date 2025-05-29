"""
enhanced_conflict_resolver.py - 增强的两阶段冲突消解器
实现骨干路径切换和停车等待的智能冲突消解策略
"""

import math
import time
from typing import List, Dict, Tuple, Optional, Set, Any
from dataclasses import dataclass, field
from enum import Enum
import heapq

class ConflictResolutionPhase(Enum):
    """冲突消解阶段"""
    BACKBONE_SWITCHING = "backbone_switching"  # 第一阶段：骨干路径切换
    PARKING_WAIT = "parking_wait"             # 第二阶段：停车等待
    ECBS_FALLBACK = "ecbs_fallback"          # 回退到ECBS

@dataclass
class ConflictZone:
    """冲突区域"""
    center: Tuple[float, float]
    radius: float
    time_window: Tuple[float, float]
    involved_vehicles: Set[str]
    severity: float
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """检查点是否在冲突区域内"""
        dx = point[0] - self.center[0]
        dy = point[1] - self.center[1]
        return math.sqrt(dx*dx + dy*dy) <= self.radius

@dataclass
class ParkingDecision:
    """停车决策"""
    vehicle_id: str
    parking_position: Tuple[float, float, float]
    parking_duration: float = 15.0
    start_time: float = 0.0
    priority_vehicle_id: str = None  # 优先通行的车辆

@dataclass
class PathSwitchDecision:
    """路径切换决策"""
    vehicle_id: str
    original_backbone_id: str
    new_backbone_id: str
    new_interface_position: Tuple[float, float]
    avoidance_score: float  # 避让效果评分

class EnhancedConflictResolver:
    """增强的两阶段冲突消解器"""
    
    def __init__(self, traffic_manager, backbone_network, path_planner):
        self.traffic_manager = traffic_manager
        self.backbone_network = backbone_network
        self.path_planner = path_planner
        
        # 配置参数
        self.config = {
            'conflict_zone_radius': 15.0,      # 冲突区域半径
            'parking_duration': 15.0,          # 默认停车时长
            'prediction_horizon': 30.0,        # 预测时域
            'min_switch_benefit': 0.3,         # 最小切换收益
            'max_switch_attempts': 3,          # 最大切换尝试次数
            'parking_safety_distance': 10.0,   # 停车安全距离
            'enable_multi_stage': True         # 启用多阶段策略
        }
        
        # 状态追踪
        self.active_resolutions = {}  # 活跃的冲突消解
        self.resolution_history = []  # 消解历史
        self.parking_vehicles = {}    # 停车中的车辆
        
        # 统计
        self.stats = {
            'total_conflicts': 0,
            'backbone_switch_success': 0,
            'parking_wait_success': 0,
            'ecbs_fallback_count': 0,
            'avg_resolution_time': 0.0,
            'conflict_recurrence_rate': 0.0
        }
    
    def resolve_conflicts(self, conflicts: List, vehicle_states: Dict) -> Dict[str, Any]:
        """
        主冲突消解方法 - 实现两阶段策略
        
        Returns:
            resolution_result: 包含消解决策和新路径的结果
        """
        self.stats['total_conflicts'] += len(conflicts)
        
        if not self.config['enable_multi_stage']:
            # 直接使用ECBS
            return self._ecbs_resolution(conflicts, vehicle_states)
        
        print(f"\n=== 增强冲突消解开始: {len(conflicts)} 个冲突 ===")
        
        # 1. 分析冲突区域
        conflict_zones = self._analyze_conflict_zones(conflicts, vehicle_states)
        
        # 2. 第一阶段：尝试骨干路径切换
        switch_result = self._phase1_backbone_switching(
            conflicts, conflict_zones, vehicle_states
        )
        
        if switch_result['success']:
            self.stats['backbone_switch_success'] += 1
            print("✅ 第一阶段成功：通过骨干路径切换解决冲突")
            return switch_result
        
        # 3. 第二阶段：停车等待策略
        parking_result = self._phase2_parking_wait(
            conflicts, conflict_zones, vehicle_states
        )
        
        if parking_result['success']:
            # 预判停车后是否还有冲突
            future_conflicts = self._predict_future_conflicts(
                parking_result, vehicle_states
            )
            
            if not future_conflicts:
                self.stats['parking_wait_success'] += 1
                print("✅ 第二阶段成功：通过停车等待解决冲突")
                return parking_result
            else:
                print("⚠️ 停车后仍有冲突，返回第一阶段重试")
                # 递归调用，但限制递归深度
                if parking_result.get('retry_count', 0) < 2:
                    parking_result['retry_count'] = parking_result.get('retry_count', 0) + 1
                    return self.resolve_conflicts(future_conflicts, vehicle_states)
        
        # 4. 回退到ECBS
        print("❌ 两阶段策略失败，回退到ECBS")
        self.stats['ecbs_fallback_count'] += 1
        return self._ecbs_resolution(conflicts, vehicle_states)
    
    def _analyze_conflict_zones(self, conflicts: List, 
                               vehicle_states: Dict) -> List[ConflictZone]:
        """分析冲突区域"""
        zones = []
        
        # 聚类相近的冲突点
        processed = set()
        
        for i, conflict in enumerate(conflicts):
            if i in processed:
                continue
            
            # 创建冲突区域
            zone = ConflictZone(
                center=conflict.location,
                radius=self.config['conflict_zone_radius'],
                time_window=(conflict.time, conflict.time + 30),
                involved_vehicles=set(conflict.agents),
                severity=conflict.severity
            )
            
            # 合并相近的冲突
            for j in range(i + 1, len(conflicts)):
                if j in processed:
                    continue
                
                other_conflict = conflicts[j]
                distance = math.sqrt(
                    (other_conflict.location[0] - zone.center[0])**2 +
                    (other_conflict.location[1] - zone.center[1])**2
                )
                
                if distance < zone.radius * 1.5:
                    # 合并到同一区域
                    zone.involved_vehicles.update(other_conflict.agents)
                    zone.severity = max(zone.severity, other_conflict.severity)
                    processed.add(j)
            
            zones.append(zone)
        
        print(f"识别出 {len(zones)} 个冲突区域")
        return zones
    
    def _phase1_backbone_switching(self, conflicts: List, 
                                  conflict_zones: List[ConflictZone],
                                  vehicle_states: Dict) -> Dict[str, Any]:
        """
        第一阶段：骨干路径切换
        """
        print("\n--- 第一阶段：骨干路径切换 ---")
        
        # 选择切换候选车辆（优先级较低的车辆）
        switch_candidates = self._select_switch_candidates(
            conflicts, vehicle_states
        )
        
        best_switch_plan = None
        best_score = float('-inf')
        
        for vehicle_id in switch_candidates:
            # 获取当前路径信息
            current_path_info = self._get_vehicle_path_info(vehicle_id)
            if not current_path_info:
                continue
            
            # 查找备选骨干路径
            alternatives = self.backbone_network.find_alternative_backbone_paths(
                current_path_info['target_type'],
                current_path_info['target_id'],
                exclude_path_id=current_path_info['backbone_id']
            )
            
            print(f"车辆 {vehicle_id} 有 {len(alternatives)} 条备选路径")
            
            for alt_path in alternatives:
                # 评估切换方案
                switch_plan = self._evaluate_path_switch(
                    vehicle_id, alt_path, conflict_zones, vehicle_states
                )
                
                if switch_plan and switch_plan.avoidance_score > best_score:
                    best_score = switch_plan.avoidance_score
                    best_switch_plan = switch_plan
        
        if best_switch_plan and best_score > self.config['min_switch_benefit']:
            # 执行最佳切换方案
            return self._execute_path_switch(best_switch_plan, vehicle_states)
        
        return {'success': False, 'reason': '没有找到有效的路径切换方案'}
    
    def _phase2_parking_wait(self, conflicts: List,
                            conflict_zones: List[ConflictZone],
                            vehicle_states: Dict) -> Dict[str, Any]:
        """
        第二阶段：停车等待策略
        """
        print("\n--- 第二阶段：停车等待 ---")
        
        # 确定停车车辆和优先通行车辆
        parking_decisions = self._determine_parking_decisions(
            conflicts, vehicle_states
        )
        
        if not parking_decisions:
            return {'success': False, 'reason': '无法确定停车策略'}
        
        # 为每个停车决策生成停车路径
        modified_paths = {}
        
        for decision in parking_decisions:
            # 找到安全停车位置
            parking_position = self._find_safe_parking_position(
                decision.vehicle_id, conflict_zones, vehicle_states
            )
            
            if parking_position:
                decision.parking_position = parking_position
                
                # 生成包含停车的新路径
                new_path = self._generate_parking_path(
                    decision, vehicle_states[decision.vehicle_id]
                )
                
                if new_path:
                    modified_paths[decision.vehicle_id] = new_path
                    self.parking_vehicles[decision.vehicle_id] = decision
        
        if modified_paths:
            return {
                'success': True,
                'phase': 'parking_wait',
                'modified_paths': modified_paths,
                'parking_decisions': parking_decisions
            }
        
        return {'success': False, 'reason': '无法生成停车路径'}
    
    def _predict_future_conflicts(self, parking_result: Dict,
                                 vehicle_states: Dict) -> List:
        """预判停车后的冲突情况"""
        print("\n预判停车后冲突...")
        
        # 模拟停车时长后的车辆位置
        future_time = time.time() + self.config['parking_duration']
        predicted_positions = {}
        
        for vehicle_id, state in vehicle_states.items():
            if vehicle_id in parking_result.get('parking_decisions', {}):
                # 停车车辆位置不变
                predicted_positions[vehicle_id] = state.position
            else:
                # 预测移动车辆位置
                if hasattr(state, 'path') and state.path:
                    future_progress = self._estimate_progress_at_time(
                        state, future_time
                    )
                    predicted_positions[vehicle_id] = self._interpolate_position(
                        state.path, future_progress
                    )
                else:
                    predicted_positions[vehicle_id] = state.position
        
        # 检测预测位置的冲突
        future_conflicts = []
        vehicles = list(predicted_positions.keys())
        
        for i in range(len(vehicles)):
            for j in range(i + 1, len(vehicles)):
                v1, v2 = vehicles[i], vehicles[j]
                pos1, pos2 = predicted_positions[v1], predicted_positions[v2]
                
                distance = math.sqrt(
                    (pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2
                )
                
                if distance < self.config['parking_safety_distance']:
                    # 创建预测冲突
                    conflict = type('Conflict', (), {
                        'agents': [v1, v2],
                        'location': ((pos1[0] + pos2[0])/2, (pos1[1] + pos2[1])/2),
                        'time': future_time,
                        'severity': 1.0 - distance / self.config['parking_safety_distance']
                    })()
                    future_conflicts.append(conflict)
        
        print(f"预测 {self.config['parking_duration']}s 后有 {len(future_conflicts)} 个冲突")
        return future_conflicts
    
    def _select_switch_candidates(self, conflicts: List,
                                 vehicle_states: Dict) -> List[str]:
        """选择路径切换候选车辆"""
        candidates = []
        
        # 统计每个车辆的冲突次数和优先级
        vehicle_scores = {}
        
        for conflict in conflicts:
            for vehicle_id in conflict.agents:
                if vehicle_id not in vehicle_scores:
                    vehicle_scores[vehicle_id] = {
                        'conflict_count': 0,
                        'priority': vehicle_states.get(vehicle_id, {}).get('priority_level', 0.5)
                    }
                vehicle_scores[vehicle_id]['conflict_count'] += 1
        
        # 选择优先级较低且冲突较多的车辆
        sorted_vehicles = sorted(
            vehicle_scores.items(),
            key=lambda x: (x[1]['priority'], -x[1]['conflict_count'])
        )
        
        # 选择前50%作为候选
        candidate_count = max(1, len(sorted_vehicles) // 2)
        candidates = [v[0] for v in sorted_vehicles[:candidate_count]]
        
        return candidates
    
    def _evaluate_path_switch(self, vehicle_id: str, 
                            alternative_path: Any,
                            conflict_zones: List[ConflictZone],
                            vehicle_states: Dict) -> Optional[PathSwitchDecision]:
        """评估路径切换方案"""
        current_position = vehicle_states[vehicle_id].position
        
        # 找到最佳接入点（避开冲突区域）
        best_interface = None
        best_score = float('-inf')
        
        # 获取备选路径的所有接口点
        if hasattr(alternative_path, 'path_id'):
            path_id = alternative_path.path_id
            interfaces = self.backbone_network.path_interfaces.get(path_id, [])
            
            for interface_id in interfaces:
                interface_info = self.backbone_network.backbone_interfaces.get(interface_id)
                if not interface_info:
                    continue
                
                interface_pos = interface_info['position']
                
                # 计算避让分数
                avoidance_score = self._calculate_avoidance_score(
                    interface_pos, conflict_zones, current_position
                )
                
                if avoidance_score > best_score:
                    best_score = avoidance_score
                    best_interface = interface_pos
        
        if best_interface and best_score > self.config['min_switch_benefit']:
            return PathSwitchDecision(
                vehicle_id=vehicle_id,
                original_backbone_id=self._get_current_backbone_id(vehicle_id),
                new_backbone_id=path_id,
                new_interface_position=best_interface,
                avoidance_score=best_score
            )
        
        return None
    
    def _calculate_avoidance_score(self, interface_pos: Tuple,
                                  conflict_zones: List[ConflictZone],
                                  current_pos: Tuple) -> float:
        """计算避让分数"""
        score = 1.0
        
        # 检查接口点到冲突区域的距离
        for zone in conflict_zones:
            if zone.contains_point(interface_pos):
                # 在冲突区域内，大幅降低分数
                score *= 0.1
            else:
                # 计算到冲突区域的距离
                distance = math.sqrt(
                    (interface_pos[0] - zone.center[0])**2 +
                    (interface_pos[1] - zone.center[1])**2
                )
                
                # 距离越远分数越高
                distance_factor = min(1.0, distance / (zone.radius * 3))
                score *= (0.5 + 0.5 * distance_factor)
        
        # 考虑接入距离
        access_distance = math.sqrt(
            (interface_pos[0] - current_pos[0])**2 +
            (interface_pos[1] - current_pos[1])**2
        )
        
        # 距离过远会降低分数
        if access_distance > 100:
            score *= 0.8
        
        return score
    
    def _execute_path_switch(self, switch_plan: PathSwitchDecision,
                           vehicle_states: Dict) -> Dict[str, Any]:
        """执行路径切换"""
        vehicle_id = switch_plan.vehicle_id
        vehicle_state = vehicle_states[vehicle_id]
        
        try:
            # 使用新的骨干路径重新规划
            new_path_result = self.backbone_network.get_path_from_position_to_target(
                vehicle_state.position,
                self._get_vehicle_target_type(vehicle_id),
                self._get_vehicle_target_id(vehicle_id),
                vehicle_id
            )
            
            if new_path_result:
                return {
                    'success': True,
                    'phase': 'backbone_switching',
                    'modified_paths': {vehicle_id: new_path_result[0]},
                    'switch_decisions': [switch_plan]
                }
        except Exception as e:
            print(f"路径切换执行失败: {e}")
        
        return {'success': False, 'reason': '路径切换执行失败'}
    
    def _determine_parking_decisions(self, conflicts: List,
                                   vehicle_states: Dict) -> List[ParkingDecision]:
        """确定停车决策"""
        decisions = []
        
        # 基于优先级和位置确定谁停车
        for conflict in conflicts:
            if len(conflict.agents) != 2:
                continue
            
            v1_id, v2_id = conflict.agents[0], conflict.agents[1]
            v1_priority = vehicle_states.get(v1_id, {}).get('priority_level', 0.5)
            v2_priority = vehicle_states.get(v2_id, {}).get('priority_level', 0.5)
            
            # 优先级低的停车
            if v1_priority < v2_priority:
                parking_vehicle = v1_id
                priority_vehicle = v2_id
            elif v2_priority < v1_priority:
                parking_vehicle = v2_id
                priority_vehicle = v1_id
            else:
                # 优先级相同，ID小的优先
                parking_vehicle = v2_id if v1_id < v2_id else v1_id
                priority_vehicle = v1_id if v1_id < v2_id else v2_id
            
            decision = ParkingDecision(
                vehicle_id=parking_vehicle,
                parking_position=None,  # 后续确定
                parking_duration=self.config['parking_duration'],
                start_time=time.time(),
                priority_vehicle_id=priority_vehicle
            )
            
            decisions.append(decision)
        
        # 去重
        unique_decisions = {}
        for decision in decisions:
            if decision.vehicle_id not in unique_decisions:
                unique_decisions[decision.vehicle_id] = decision
        
        return list(unique_decisions.values())
    
    def _find_safe_parking_position(self, vehicle_id: str,
                                  conflict_zones: List[ConflictZone],
                                  vehicle_states: Dict) -> Optional[Tuple]:
        """找到安全停车位置"""
        current_state = vehicle_states[vehicle_id]
        current_pos = current_state.position
        
        # 在当前位置附近寻找安全位置
        search_radius = 20.0
        best_position = None
        best_score = float('-inf')
        
        # 生成候选位置
        for angle in range(0, 360, 45):
            for distance in [5, 10, 15, 20]:
                rad = math.radians(angle)
                candidate = (
                    current_pos[0] + distance * math.cos(rad),
                    current_pos[1] + distance * math.sin(rad),
                    current_pos[2] if len(current_pos) > 2 else 0
                )
                
                # 评估位置安全性
                safety_score = self._evaluate_parking_position_safety(
                    candidate, conflict_zones, vehicle_states
                )
                
                if safety_score > best_score:
                    best_score = safety_score
                    best_position = candidate
        
        return best_position if best_score > 0.5 else None
    
    def _evaluate_parking_position_safety(self, position: Tuple,
                                        conflict_zones: List[ConflictZone],
                                        vehicle_states: Dict) -> float:
        """评估停车位置安全性"""
        score = 1.0
        
        # 检查是否在冲突区域内
        for zone in conflict_zones:
            if zone.contains_point(position[:2]):
                return 0.0  # 不能停在冲突区域内
        
        # 检查与其他车辆的距离
        for other_id, other_state in vehicle_states.items():
            other_pos = other_state.position
            distance = math.sqrt(
                (position[0] - other_pos[0])**2 +
                (position[1] - other_pos[1])**2
            )
            
            if distance < self.config['parking_safety_distance']:
                score *= (distance / self.config['parking_safety_distance'])
        
        # 检查是否在道路边缘（假设有道路信息）
        # 这里简化处理
        
        return score
    
    def _generate_parking_path(self, decision: ParkingDecision,
                             vehicle_state: Any) -> Optional[List[Tuple]]:
        """生成包含停车的路径"""
        if not decision.parking_position:
            return None
        
        try:
            # 1. 规划到停车位置的路径
            parking_path = self.path_planner.plan_path(
                vehicle_state.vehicle_id,
                vehicle_state.position,
                decision.parking_position,
                use_backbone=False,  # 停车时不使用骨干路径
                context='emergency'
            )
            
            if not parking_path:
                return None
            
            # 2. 在停车位置等待（通过重复点表示）
            wait_points = []
            wait_steps = int(decision.parking_duration / 0.5)  # 假设0.5秒一个步长
            for _ in range(wait_steps):
                wait_points.append(decision.parking_position)
            
            # 3. 继续原有目标的路径
            resume_path = self.path_planner.plan_path(
                vehicle_state.vehicle_id,
                decision.parking_position,
                vehicle_state.goal,  # 原目标
                use_backbone=True,
                context='normal'
            )
            
            if not resume_path:
                return None
            
            # 合并路径
            if isinstance(parking_path, tuple):
                parking_path = parking_path[0]
            if isinstance(resume_path, tuple):
                resume_path = resume_path[0]
            
            complete_path = parking_path[:-1] + wait_points + resume_path
            
            return complete_path
            
        except Exception as e:
            print(f"生成停车路径失败: {e}")
            return None
    
    def _ecbs_resolution(self, conflicts: List,
                        vehicle_states: Dict) -> Dict[str, Any]:
        """回退到ECBS冲突消解"""
        # 调用原有的ECBS解决方案
        if self.traffic_manager:
            return self.traffic_manager.resolve_conflicts(conflicts)
        
        return {'success': False, 'reason': 'ECBS不可用'}
    
    # 辅助方法
    def _get_vehicle_path_info(self, vehicle_id: str) -> Optional[Dict]:
        """获取车辆路径信息"""
        # 从traffic_manager或其他来源获取
        if self.traffic_manager and hasattr(self.traffic_manager, 'active_paths'):
            path_info = self.traffic_manager.active_paths.get(vehicle_id)
            if path_info:
                return {
                    'path': path_info.get('path'),
                    'backbone_id': path_info.get('backbone_id'),
                    'target_type': path_info.get('target_type', 'unloading'),
                    'target_id': path_info.get('target_id', 0)
                }
        return None
    
    def _get_current_backbone_id(self, vehicle_id: str) -> Optional[str]:
        """获取当前骨干路径ID"""
        path_info = self._get_vehicle_path_info(vehicle_id)
        return path_info['backbone_id'] if path_info else None
    
    def _get_vehicle_target_type(self, vehicle_id: str) -> str:
        """获取车辆目标类型"""
        # 这需要从任务信息中获取
        return 'unloading'  # 简化处理
    
    def _get_vehicle_target_id(self, vehicle_id: str) -> int:
        """获取车辆目标ID"""
        # 这需要从任务信息中获取
        return 0  # 简化处理
    
    def _estimate_progress_at_time(self, vehicle_state: Any,
                                  future_time: float) -> float:
        """估算未来时刻的路径进度"""
        if not hasattr(vehicle_state, 'speed'):
            return 0.0
        
        time_delta = future_time - time.time()
        distance = vehicle_state.speed * time_delta
        
        if hasattr(vehicle_state, 'path') and vehicle_state.path:
            path_length = self._calculate_path_length(vehicle_state.path)
            if path_length > 0:
                return min(1.0, distance / path_length)
        
        return 0.0
    
    def _interpolate_position(self, path: List[Tuple],
                            progress: float) -> Tuple:
        """路径位置插值"""
        if not path:
            return (0, 0, 0)
        
        if progress <= 0:
            return path[0]
        if progress >= 1:
            return path[-1]
        
        # 简化的线性插值
        index = int(progress * (len(path) - 1))
        if index >= len(path) - 1:
            return path[-1]
        
        return path[index]
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径长度"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.sqrt(dx*dx + dy*dy)
        
        return length
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        total_attempts = (self.stats['backbone_switch_success'] + 
                         self.stats['parking_wait_success'] + 
                         self.stats['ecbs_fallback_count'])
        
        if total_attempts > 0:
            backbone_rate = self.stats['backbone_switch_success'] / total_attempts
            parking_rate = self.stats['parking_wait_success'] / total_attempts
            ecbs_rate = self.stats['ecbs_fallback_count'] / total_attempts
        else:
            backbone_rate = parking_rate = ecbs_rate = 0.0
        
        return {
            'total_conflicts': self.stats['total_conflicts'],
            'resolution_success_rate': {
                'backbone_switching': backbone_rate,
                'parking_wait': parking_rate,
                'ecbs_fallback': ecbs_rate
            },
            'active_parking_vehicles': len(self.parking_vehicles),
            'average_resolution_time': self.stats['avg_resolution_time']
        }
    
    def clear_parking_vehicles(self):
        """清理已完成停车的车辆"""
        current_time = time.time()
        completed = []
        
        for vehicle_id, decision in self.parking_vehicles.items():
            if current_time - decision.start_time > decision.parking_duration:
                completed.append(vehicle_id)
        
        for vehicle_id in completed:
            del self.parking_vehicles[vehicle_id]
            print(f"车辆 {vehicle_id} 完成停车等待")