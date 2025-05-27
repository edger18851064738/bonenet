"""
optimized_backbone_network.py - 修复版骨干路径网络
修复路径生成逻辑，确保生成所有必要的路径组合：
- 装载点 ↔ 卸载点
- 装载点 ↔ 停车场  
- 卸载点 ↔ 停车场
"""

import math
import time
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import threading
@dataclass
class BiDirectionalPath:
    """双向路径数据结构"""
    path_id: str
    point_a: Dict  # 起点信息
    point_b: Dict  # 终点信息
    forward_path: List[Tuple]  # A->B路径
    reverse_path: List[Tuple]  # B->A路径（自动生成）
    length: float
    quality: float
    planner_used: str
    created_time: float
    usage_count: int = 0
    current_load: int = 0  # 当前使用该路径的车辆数
    max_capacity: int = 5  # 最大容量
    
    def get_path(self, from_point_type: str, from_point_id: int, 
                to_point_type: str, to_point_id: int) -> Optional[List[Tuple]]:
        """获取指定方向的路径"""
        # 检查是否匹配A->B方向
        if (self.point_a['type'] == from_point_type and self.point_a['id'] == from_point_id and
            self.point_b['type'] == to_point_type and self.point_b['id'] == to_point_id):
            return self.forward_path
        
        # 检查是否匹配B->A方向
        if (self.point_b['type'] == from_point_type and self.point_b['id'] == from_point_id and
            self.point_a['type'] == to_point_type and self.point_a['id'] == to_point_id):
            return self.reverse_path
        
        return None
    
    def increment_usage(self):
        """增加使用计数"""
        self.usage_count += 1
    
    def add_vehicle(self, vehicle_id: str):
        """添加车辆到路径"""
        self.current_load += 1
    
    def remove_vehicle(self, vehicle_id: str):
        """从路径移除车辆"""
        self.current_load = max(0, self.current_load - 1)
    
    def get_load_factor(self) -> float:
        """获取负载因子"""
        return self.current_load / self.max_capacity
class InterfaceReservationManager:
    """接口节点预留管理器"""
    
    def __init__(self):
        self.reservations = {}  # {interface_id: reservation_info}
        self.lock = threading.RLock()
    
    def reserve_interface(self, interface_id: str, vehicle_id: str, 
                         start_time: float, duration: float = 60.0) -> bool:
        """预留接口节点"""
        with self.lock:
            if self.is_interface_available(interface_id, start_time, duration):
                self.reservations[interface_id] = {
                    'vehicle_id': vehicle_id,
                    'start_time': start_time,
                    'duration': duration,
                    'end_time': start_time + duration
                }
                return True
            return False
    
    def is_interface_available(self, interface_id: str, start_time: float, 
                              duration: float) -> bool:
        """检查接口节点是否可用"""
        with self.lock:
            if interface_id not in self.reservations:
                return True
            
            reservation = self.reservations[interface_id]
            reserved_start = reservation['start_time']
            reserved_end = reservation['end_time']
            
            request_end = start_time + duration
            
            # 检查时间冲突
            return request_end <= reserved_start or start_time >= reserved_end
    
    def release_interface(self, interface_id: str, vehicle_id: str):
        """释放接口节点"""
        with self.lock:
            if (interface_id in self.reservations and 
                self.reservations[interface_id]['vehicle_id'] == vehicle_id):
                del self.reservations[interface_id]
    
    def cleanup_expired_reservations(self, current_time: float):
        """清理过期预留"""
        with self.lock:
            expired_interfaces = []
            for interface_id, reservation in self.reservations.items():
                if current_time > reservation['end_time']:
                    expired_interfaces.append(interface_id)
            
            for interface_id in expired_interfaces:
                del self.reservations[interface_id]

class OptimizedBackboneNetwork:
    """增强版骨干路径网络 - 智能节点选择与负载均衡"""
    
    def __init__(self, env):
        self.env = env
        self.path_planner = None
        
        # 核心数据结构 - 双向路径
        self.bidirectional_paths = {}  # {path_id: BiDirectionalPath}
        self.special_points = {'loading': [], 'unloading': [], 'parking': []}
        
        # 接口系统（增强版）
        self.backbone_interfaces = {}
        self.path_interfaces = defaultdict(list)
        self.interface_manager = InterfaceReservationManager()
        
        # 路径查找索引
        self.connection_index = {}  # {(type_a, id_a, type_b, id_b): path_id}
        
        # 负载均衡追踪
        self.vehicle_path_assignments = {}  # {vehicle_id: path_id}
        self.path_load_history = defaultdict(list)  # {path_id: [load_samples]}
        
        # 优化配置
        self.config = {
            'primary_quality_threshold': 0.7,
            'fallback_quality_threshold': 0.4,
            'max_planning_time_per_path': 20.0,
            'enable_progressive_fallback': True,
            'interface_spacing': 8,
            'retry_with_relaxed_params': True,
            'load_balancing_weight': 0.3,  # 负载均衡权重
            'path_switching_threshold': 0.8,  # 负载切换阈值
            'interface_reservation_duration': 60.0  # 接口预留时长
        }
        
        # 统计信息
        self.stats = {
            'total_path_pairs': 0,
            'successful_paths': 0,
            'astar_success': 0,
            'rrt_success': 0,
            'direct_fallback': 0,
            'generation_time': 0,
            'loading_to_unloading': 0,
            'loading_to_parking': 0,
            'unloading_to_parking': 0,
            'load_balancing_decisions': 0,
            'path_switches': 0,
            'interface_reservations': 0
        }
        
        print("初始化增强版骨干路径网络（智能节点选择+负载均衡）")
    
    def set_path_planner(self, path_planner):
        """设置路径规划器"""
        self.path_planner = path_planner
        print("已设置路径规划器")
    
    def generate_backbone_network(self, quality_threshold: float = None) -> bool:
        """
        优化的骨干网络生成 - 完整路径组合策略
        """
        start_time = time.time()
        print("开始生成完整骨干路径网络...")
        
        # 更新配置
        if quality_threshold is not None:
            self.config['primary_quality_threshold'] = quality_threshold
        
        try:
            # 步骤1: 加载特殊点
            self._load_special_points()
            if not self._validate_special_points():
                return False
            
            # 步骤2: 生成所有路径组合（核心修复）
            success_count = self._generate_complete_bidirectional_paths()
            
            if success_count == 0:
                print("❌ 没有成功生成任何骨干路径")
                return False
            
            # 步骤3: 生成接口
            total_interfaces = self._generate_interfaces_for_bidirectional_paths()
            
            # 步骤4: 建立连接索引
            self._build_connection_index()
            
            # 更新统计
            generation_time = time.time() - start_time
            self.stats.update({
                'successful_paths': len(self.bidirectional_paths),
                'generation_time': generation_time
            })
            
            # 成功率计算
            success_rate = success_count / max(1, self.stats['total_path_pairs'])
            
            print(f"\n🎉 完整骨干网络生成完成!")
            print(f"  双向路径: {len(self.bidirectional_paths)} 条")
            print(f"  成功率: {success_rate:.1%}")
            print(f"  路径组合分布:")
            print(f"    装载点↔卸载点: {self.stats['loading_to_unloading']} 条")
            print(f"    装载点↔停车场: {self.stats['loading_to_parking']} 条")
            print(f"    卸载点↔停车场: {self.stats['unloading_to_parking']} 条")
            print(f"  接口数量: {total_interfaces} 个")
            print(f"  生成耗时: {generation_time:.2f}s")
            
            return True
        
        except Exception as e:
            print(f"❌ 骨干网络生成失败: {e}")
            return False
    
    def _load_special_points(self):
        """加载特殊点"""
        # 装载点
        self.special_points['loading'] = []
        for i, point in enumerate(self.env.loading_points):
            self.special_points['loading'].append({
                'id': i, 'type': 'loading', 'position': self._ensure_3d_point(point)
            })
        
        # 卸载点
        self.special_points['unloading'] = []
        for i, point in enumerate(self.env.unloading_points):
            self.special_points['unloading'].append({
                'id': i, 'type': 'unloading', 'position': self._ensure_3d_point(point)
            })
        
        # 停车点
        self.special_points['parking'] = []
        parking_areas = getattr(self.env, 'parking_areas', [])
        for i, point in enumerate(parking_areas):
            self.special_points['parking'].append({
                'id': i, 'type': 'parking', 'position': self._ensure_3d_point(point)
            })
        
        print(f"加载特殊点: 装载{len(self.special_points['loading'])}个, "
              f"卸载{len(self.special_points['unloading'])}个, "
              f"停车{len(self.special_points['parking'])}个")
    
    def _validate_special_points(self) -> bool:
        """验证特殊点"""
        if not self.special_points['loading'] or not self.special_points['unloading']:
            print("❌ 缺少必要的装载点或卸载点")
            return False
        return True
    
    def _generate_complete_bidirectional_paths(self) -> int:
        """生成完整的双向路径组合"""
        if not self.path_planner:
            print("❌ 未设置路径规划器")
            return 0
        
        success_count = 0
        path_pairs = []
        
        # 定义需要连接的点类型组合
        connection_types = [
            ('loading', 'unloading'),
            ('loading', 'parking'),
            ('unloading', 'parking')
        ]
        
        # 收集所有需要连接的点对
        for type_a, type_b in connection_types:
            if (len(self.special_points[type_a]) == 0 or 
                len(self.special_points[type_b]) == 0):
                continue
                
            for point_a in self.special_points[type_a]:
                for point_b in self.special_points[type_b]:
                    path_pairs.append((point_a, point_b, f"{type_a}_to_{type_b}"))
        
        self.stats['total_path_pairs'] = len(path_pairs)
        print(f"\n需要生成 {len(path_pairs)} 条双向路径")
        
        # 生成每条双向路径
        for i, (point_a, point_b, connection_type) in enumerate(path_pairs, 1):
            print(f"\n[{i}/{len(path_pairs)}] 生成路径: {point_a['type'][0].upper()}{point_a['id']} ↔ {point_b['type'][0].upper()}{point_b['id']}")
            
            path_result = self._generate_single_bidirectional_path(point_a, point_b)
            
            if path_result:
                success_count += 1
                self.stats[connection_type] += 1
                print(f"  ✅ 成功: 长度{len(path_result.forward_path)}, "
                      f"质量{path_result.quality:.2f}, "
                      f"规划器: {path_result.planner_used}")
            else:
                print(f"  ❌ 失败")
        
        return success_count
    
    def _generate_single_bidirectional_path(self, point_a: Dict, point_b: Dict) -> Optional[BiDirectionalPath]:
        """生成单条双向路径"""
        path_id = f"{point_a['type'][0].upper()}{point_a['id']}_to_{point_b['type'][0].upper()}{point_b['id']}"
        
        start_pos = point_a['position']
        end_pos = point_b['position']
        
        # 渐进回退策略
        planning_strategies = [
            {
                'name': 'hybrid_astar_strict',
                'planner_type': 'hybrid_astar',
                'quality_threshold': self.config['primary_quality_threshold'],
                'max_time': 15.0,
                'context': 'backbone'
            },
            {
                'name': 'hybrid_astar_relaxed',
                'planner_type': 'hybrid_astar', 
                'quality_threshold': self.config['fallback_quality_threshold'],
                'max_time': 20.0,
                'context': 'backbone'
            },
            {
                'name': 'rrt_standard',
                'planner_type': 'rrt',
                'quality_threshold': self.config['fallback_quality_threshold'],
                'max_time': 15.0,
                'context': 'backbone'
            },
            {
                'name': 'direct_fallback',
                'planner_type': 'direct',
                'quality_threshold': 0.3,
                'max_time': 1.0,
                'context': 'fallback'
            }
        ]
        
        for strategy in planning_strategies:
            if not self.config['enable_progressive_fallback'] and strategy['name'] != 'hybrid_astar_strict':
                continue
            
            try:
                # 尝试双向规划，选择更好的方向
                result_ab = self._plan_with_strategy(start_pos, end_pos, strategy, f"{path_id}_AB")
                result_ba = self._plan_with_strategy(end_pos, start_pos, strategy, f"{path_id}_BA")
                
                # 选择更好的结果
                best_result = None
                best_direction = None
                
                if result_ab and result_ba:
                    if result_ab[1] >= result_ba[1]:
                        best_result, best_direction = result_ab, 'AB'
                    else:
                        best_result, best_direction = result_ba, 'BA'
                elif result_ab:
                    best_result, best_direction = result_ab, 'AB'
                elif result_ba:
                    best_result, best_direction = result_ba, 'BA'
                
                if best_result:
                    path, quality = best_result
                    
                    # 创建双向路径对象
                    if best_direction == 'AB':
                        forward_path = path
                        reverse_path = self._reverse_path(path)
                    else:
                        reverse_path = path
                        forward_path = self._reverse_path(path)
                    
                    bidirectional_path = BiDirectionalPath(
                        path_id=path_id,
                        point_a=point_a,
                        point_b=point_b,
                        forward_path=forward_path,
                        reverse_path=reverse_path,
                        length=self._calculate_path_length(forward_path),
                        quality=quality,
                        planner_used=strategy['planner_type'],
                        created_time=time.time()
                    )
                    
                    # 存储路径
                    self.bidirectional_paths[path_id] = bidirectional_path
                    
                    # 更新统计
                    if strategy['planner_type'] == 'hybrid_astar':
                        self.stats['astar_success'] += 1
                    elif strategy['planner_type'] == 'rrt':
                        self.stats['rrt_success'] += 1
                    elif strategy['planner_type'] == 'direct':
                        self.stats['direct_fallback'] += 1
                    
                    return bidirectional_path
            
            except Exception as e:
                continue
        
        return None
    
    def _plan_with_strategy(self, start: Tuple, goal: Tuple, strategy: Dict, 
                           agent_id: str) -> Optional[Tuple[List, float]]:
        """使用指定策略进行规划"""
        try:
            result = self.path_planner.plan_path(
                vehicle_id=agent_id,
                start=start,
                goal=goal,
                use_backbone=False,
                planner_type=strategy['planner_type'],
                context=strategy['context'],
                quality_threshold=strategy['quality_threshold'],
                return_object=True
            )
            
            if result and hasattr(result, 'path') and result.path:
                if len(result.path) >= 2:
                    quality = getattr(result, 'quality_score', 0.5)
                    
                    if quality >= strategy['quality_threshold']:
                        return (result.path, quality)
            
        except Exception as e:
            pass
        
        return None
    
    def _reverse_path(self, path: List[Tuple]) -> List[Tuple]:
        """反转路径方向"""
        if not path:
            return []
        
        reversed_path = []
        for point in reversed(path):
            if len(point) >= 3:
                x, y, theta = point[0], point[1], point[2]
                reverse_theta = (theta + math.pi) % (2 * math.pi)
                reversed_path.append((x, y, reverse_theta))
            else:
                reversed_path.append(point)
        
        return reversed_path
    
    def _generate_interfaces_for_bidirectional_paths(self) -> int:
        """为双向路径生成接口"""
        total_interfaces = 0
        spacing = self.config['interface_spacing']
        
        for path_id, path_data in self.bidirectional_paths.items():
            forward_path = path_data.forward_path
            
            if len(forward_path) < 2:
                continue
            
            interface_count = 0
            
            # 在路径上等间距生成接口
            for i in range(0, len(forward_path), spacing):
                if i >= len(forward_path):
                    break
                
                interface_id = f"{path_id}_if_{interface_count}"
                
                # 增强接口存储
                self.backbone_interfaces[interface_id] = {
                    'id': interface_id,
                    'position': forward_path[i],
                    'path_id': path_id,
                    'path_index': i,
                    'is_occupied': False,
                    'reservation_count': 0,
                    'usage_history': []
                }
                
                self.path_interfaces[path_id].append(interface_id)
                interface_count += 1
                total_interfaces += 1
        
        return total_interfaces
    
    def _build_connection_index(self):
        """建立连接索引"""
        self.connection_index.clear()
        
        for path_id, path_data in self.bidirectional_paths.items():
            point_a = path_data.point_a
            point_b = path_data.point_b
            
            # 双向索引
            key_ab = (point_a['type'], point_a['id'], point_b['type'], point_b['id'])
            key_ba = (point_b['type'], point_b['id'], point_a['type'], point_a['id'])
            
            self.connection_index[key_ab] = path_id
            self.connection_index[key_ba] = path_id
    
    def get_path_from_position_to_target(self, current_position: Tuple, 
                                       target_type: str, target_id: int,
                                       vehicle_id: str = None) -> Optional[Tuple]:
        """
        智能路径查找 - 优化节点选择和负载均衡
        """
        print(f"智能路径查找: {current_position} -> {target_type}_{target_id}")
        
        # 查找所有到达目标的双向路径
        candidate_paths = []
        
        for path_id, path_data in self.bidirectional_paths.items():
            if ((path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)):
                candidate_paths.append(path_data)
        
        if not candidate_paths:
            print(f"  没有找到到 {target_type}_{target_id} 的骨干路径")
            return self._direct_planning_fallback(current_position, target_type, target_id)
        
        # 使用负载均衡选择最佳路径
        best_path_data = self._select_best_path_with_load_balancing(candidate_paths)
        
        # 智能节点选择
        best_option = self._find_optimal_interface_node(
            current_position, best_path_data, target_type, target_id
        )
        
        if not best_option:
            return self._direct_planning_fallback(current_position, target_type, target_id)
        
        # 尝试预留接口节点
        if vehicle_id:
            current_time = time.time()
            interface_id = f"{best_path_data.path_id}_if_{best_option['interface_index'] // self.config['interface_spacing']}"
            
            if self.interface_manager.reserve_interface(
                interface_id, vehicle_id, current_time, 
                self.config['interface_reservation_duration']
            ):
                self.stats['interface_reservations'] += 1
                print(f"  已预留接口节点: {interface_id}")
        
        # 构建完整路径
        complete_path, structure = self._build_complete_path_with_optimization(
            current_position, best_option, best_path_data, vehicle_id
        )
        
        if complete_path:
            # 更新路径负载
            if vehicle_id:
                self._assign_vehicle_to_path(vehicle_id, best_path_data.path_id)
            
            print(f"  ✅ 智能骨干路径成功: 总长度{len(complete_path)}")
            return complete_path, structure
        
        return None
    
    def _select_best_path_with_load_balancing(self, candidate_paths: List) -> Any:
        """使用负载均衡选择最佳路径"""
        best_path = None
        best_score = float('inf')
        
        for path_data in candidate_paths:
            # 基础路径质量分数 (越小越好)
            quality_score = 1.0 / max(0.1, path_data.quality)
            
            # 负载惩罚因子
            load_factor = path_data.get_load_factor()
            load_penalty = 1.0 + (load_factor * self.config['load_balancing_weight'] * 3.0)
            
            # 使用历史统计的动态负载
            avg_historical_load = self._get_average_historical_load(path_data.path_id)
            history_penalty = 1.0 + (avg_historical_load * 0.2)
            
            # 综合分数
            total_score = quality_score * load_penalty * history_penalty
            
            if total_score < best_score:
                best_score = total_score
                best_path = path_data
        
        # 记录负载均衡决策
        self.stats['load_balancing_decisions'] += 1
        
        return best_path
    
    def _get_average_historical_load(self, path_id: str) -> float:
        """获取路径的平均历史负载"""
        if path_id not in self.path_load_history:
            return 0.0
        
        history = self.path_load_history[path_id]
        if not history:
            return 0.0
        
        # 最近10次的平均值
        recent_samples = history[-10:]
        return sum(recent_samples) / len(recent_samples)
    
    def _find_optimal_interface_node(self, current_position: Tuple, 
                                   path_data: Any, target_type: str, target_id: int) -> Optional[Dict]:
        """选择使总路径最短的最优接口节点"""
        # 确定使用方向
        if path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id:
            backbone_path = path_data.reverse_path
            target_point = path_data.point_a['position']
        else:
            backbone_path = path_data.forward_path
            target_point = path_data.point_b['position']
        
        best_option = None
        min_total_cost = float('inf')
        
        # 遍历所有接口节点，选择总代价最小的
        for i in range(0, len(backbone_path), self.config['interface_spacing']):
            interface_pos = backbone_path[i]
            
            # 计算：当前位置→接口节点的距离
            access_distance = self._calculate_distance(current_position, interface_pos)
            
            # 计算：接口节点→目标点的骨干路径距离
            remaining_backbone = backbone_path[i:]
            backbone_distance = self._calculate_path_length(remaining_backbone)
            
            # 接口节点拥堵因子
            interface_id = f"{path_data.path_id}_if_{i // self.config['interface_spacing']}"
            congestion_factor = 1.0
            if interface_id in self.backbone_interfaces:
                reservation_count = self.backbone_interfaces[interface_id].get('reservation_count', 0)
                congestion_factor = 1.0 + (reservation_count * 0.1)
            
            # 总代价（距离 + 拥堵惩罚）
            total_cost = (access_distance + backbone_distance) * congestion_factor
            
            if total_cost < min_total_cost:
                min_total_cost = total_cost
                best_option = {
                    'interface_index': i,
                    'interface_position': interface_pos,
                    'access_distance': access_distance,
                    'backbone_distance': backbone_distance,
                    'total_cost': total_cost,
                    'remaining_path': remaining_backbone,
                    'congestion_factor': congestion_factor
                }
        
        return best_option
    
    def _build_complete_path_with_optimization(self, current_position: Tuple, 
                                             best_option: Dict, path_data: Any,
                                             vehicle_id: str = None) -> Tuple[Optional[List], Dict]:
        """构建优化的完整路径"""
        interface_pos = best_option['interface_position']
        remaining_path = best_option['remaining_path']
        
        # 如果距离接口很近，直接使用骨干路径
        if best_option['access_distance'] < 3.0:
            structure = {
                'type': 'optimized_backbone_only',
                'path_id': path_data.path_id,
                'backbone_utilization': 1.0,
                'total_length': len(remaining_path),
                'optimization_used': 'direct_access',
                'load_factor': path_data.get_load_factor()
            }
            return remaining_path, structure
        
        # 规划接入路径
        if not self.path_planner:
            return None, {}
        
        try:
            access_result = self.path_planner.plan_path(
                vehicle_id=vehicle_id or "optimized_access",
                start=current_position,
                goal=interface_pos,
                use_backbone=False
            )
            
            if access_result:
                # 处理不同的返回格式
                if hasattr(access_result, 'path'):
                    access_path = access_result.path
                elif isinstance(access_result, tuple):
                    access_path = access_result[0]
                else:
                    access_path = access_result
                
                if access_path:
                    # 合并路径
                    complete_path = access_path[:-1] + remaining_path
                    
                    structure = {
                        'type': 'optimized_interface_assisted',
                        'path_id': path_data.path_id,
                        'access_path': access_path,
                        'backbone_path': remaining_path,
                        'backbone_utilization': len(remaining_path) / len(complete_path),
                        'total_length': len(complete_path),
                        'optimization_used': 'smart_interface_selection',
                        'load_factor': path_data.get_load_factor(),
                        'congestion_factor': best_option['congestion_factor']
                    }
                    
                    # 增加使用计数
                    path_data.increment_usage()
                    
                    return complete_path, structure
        
        except Exception as e:
            print(f"    接入路径规划失败: {e}")
        
        return None, {}
    
    def _assign_vehicle_to_path(self, vehicle_id: str, path_id: str):
        """分配车辆到路径"""
        # 如果车辆已分配到其他路径，先移除
        if vehicle_id in self.vehicle_path_assignments:
            old_path_id = self.vehicle_path_assignments[vehicle_id]
            if old_path_id in self.bidirectional_paths:
                self.bidirectional_paths[old_path_id].remove_vehicle(vehicle_id)
        
        # 分配到新路径
        self.vehicle_path_assignments[vehicle_id] = path_id
        if path_id in self.bidirectional_paths:
            self.bidirectional_paths[path_id].add_vehicle(vehicle_id)
            
            # 记录负载历史
            current_load = self.bidirectional_paths[path_id].get_load_factor()
            self.path_load_history[path_id].append(current_load)
            
            # 限制历史记录长度
            if len(self.path_load_history[path_id]) > 100:
                self.path_load_history[path_id] = self.path_load_history[path_id][-50:]
    
    def release_vehicle_from_path(self, vehicle_id: str):
        """从路径释放车辆"""
        if vehicle_id in self.vehicle_path_assignments:
            path_id = self.vehicle_path_assignments[vehicle_id]
            
            if path_id in self.bidirectional_paths:
                self.bidirectional_paths[path_id].remove_vehicle(vehicle_id)
            
            del self.vehicle_path_assignments[vehicle_id]
            
            # 释放接口预留
            for interface_id in self.backbone_interfaces:
                self.interface_manager.release_interface(interface_id, vehicle_id)
    
    def find_alternative_backbone_paths(self, target_type: str, target_id: int, 
                                      exclude_path_id: str = None) -> List:
        """查找备选骨干路径"""
        alternatives = []
        
        for path_id, path_data in self.bidirectional_paths.items():
            if path_id == exclude_path_id:
                continue
                
            if ((path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)):
                
                # 检查路径负载是否在可接受范围内
                if path_data.get_load_factor() < self.config['path_switching_threshold']:
                    alternatives.append(path_data)
        
        # 按质量和负载排序
        alternatives.sort(key=lambda p: (p.get_load_factor(), -p.quality))
        
        return alternatives
    
    def _direct_planning_fallback(self, current_position: Tuple, 
                                 target_type: str, target_id: int) -> Optional[Tuple]:
        """直接规划回退"""
        if not self.path_planner:
            return None
        
        try:
            target_position = self._get_target_position(target_type, target_id)
            if not target_position:
                return None
            
            result = self.path_planner.plan_path(
                vehicle_id="direct_fallback",
                start=current_position,
                goal=target_position,
                use_backbone=False
            )
            
            if result:
                if hasattr(result, 'path'):
                    path = result.path
                elif isinstance(result, tuple):
                    path = result[0]
                else:
                    path = result
                
                if path:
                    structure = {
                        'type': 'direct_fallback',
                        'backbone_utilization': 0.0,
                        'total_length': len(path)
                    }
                    
                    print(f"  ✅ 直接规划回退成功: 长度{len(path)}")
                    return path, structure
        
        except Exception as e:
            print(f"    直接规划回退失败: {e}")
        
        return None
    
    def _get_target_position(self, target_type: str, target_id: int) -> Optional[Tuple]:
        """获取目标位置"""
        if target_type in self.special_points:
            points = self.special_points[target_type]
            if 0 <= target_id < len(points):
                return points[target_id]['position']
        return None
    
    def _ensure_3d_point(self, point) -> Tuple[float, float, float]:
        """确保点坐标为3D"""
        if not point:
            return (0.0, 0.0, 0.0)
        elif len(point) >= 3:
            return (float(point[0]), float(point[1]), float(point[2]))
        elif len(point) == 2:
            return (float(point[0]), float(point[1]), 0.0)
        else:
            return (0.0, 0.0, 0.0)
    
    def _calculate_distance(self, p1: Tuple, p2: Tuple) -> float:
        """计算两点间距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _calculate_path_length(self, path: List[Tuple]) -> float:
        """计算路径总长度"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += self._calculate_distance(path[i], path[i + 1])
        return length
    
    def update_load_balancing(self, time_delta: float):
        """更新负载均衡"""
        current_time = time.time()
        
        # 清理过期的接口预留
        self.interface_manager.cleanup_expired_reservations(current_time)
        
        # 更新接口使用统计
        for interface_id, interface_info in self.backbone_interfaces.items():
            if interface_id in self.interface_manager.reservations:
                interface_info['reservation_count'] = 1
            else:
                interface_info['reservation_count'] = 0
    
    def get_network_status(self) -> Dict:
        """获取网络状态"""
        return {
            'bidirectional_paths': len(self.bidirectional_paths),
            'total_interfaces': len(self.backbone_interfaces),
            'generation_stats': self.stats,
            'special_points': {
                'loading': len(self.special_points['loading']),
                'unloading': len(self.special_points['unloading']),
                'parking': len(self.special_points['parking'])
            },
            'path_combinations': {
                'loading_to_unloading': self.stats['loading_to_unloading'],
                'loading_to_parking': self.stats['loading_to_parking'],
                'unloading_to_parking': self.stats['unloading_to_parking']
            },
            'load_balancing': {
                'active_vehicle_assignments': len(self.vehicle_path_assignments),
                'average_path_utilization': self._calculate_average_path_utilization(),
                'interface_reservations': len(self.interface_manager.reservations)
            }
        }
    
    def _calculate_average_path_utilization(self) -> float:
        """计算平均路径利用率"""
        if not self.bidirectional_paths:
            return 0.0
        
        total_utilization = sum(path.get_load_factor() for path in self.bidirectional_paths.values())
        return total_utilization / len(self.bidirectional_paths)
    
    def debug_network_info(self):
        """调试网络信息"""
        print("=== 增强骨干网络调试信息 ===")
        print(f"双向路径数量: {len(self.bidirectional_paths)}")
        print(f"活跃车辆分配: {len(self.vehicle_path_assignments)}")
        print(f"接口预留: {len(self.interface_manager.reservations)}")
        print(f"平均路径利用率: {self._calculate_average_path_utilization():.2%}")
        
        # 显示高负载路径
        high_load_paths = []
        for path_id, path_data in self.bidirectional_paths.items():
            load_factor = path_data.get_load_factor()
            if load_factor > 0.5:
                high_load_paths.append((path_id, load_factor))
        
        if high_load_paths:
            print(f"\n高负载路径 ({len(high_load_paths)} 条):")
            for path_id, load_factor in sorted(high_load_paths, key=lambda x: x[1], reverse=True):
                print(f"  {path_id}: {load_factor:.1%} 负载")

SimplifiedBackboneNetwork = OptimizedBackboneNetwork