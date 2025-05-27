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

class OptimizedBackboneNetwork:
    """优化版骨干路径网络 - 修复完整路径组合生成"""
    
    def __init__(self, env):
        self.env = env
        self.path_planner = None
        
        # 核心数据结构 - 双向路径
        self.bidirectional_paths = {}  # {path_id: BiDirectionalPath}
        self.special_points = {'loading': [], 'unloading': [], 'parking': []}
        
        # 接口系统（简化版）
        self.backbone_interfaces = {}
        self.path_interfaces = defaultdict(list)
        
        # 路径查找索引
        self.connection_index = {}  # {(type_a, id_a, type_b, id_b): path_id}
        
        # 优化配置
        self.config = {
            'primary_quality_threshold': 0.7,    # 主要质量阈值
            'fallback_quality_threshold': 0.4,   # 回退质量阈值
            'max_planning_time_per_path': 20.0,  # 单路径最大规划时间
            'enable_progressive_fallback': True,  # 启用渐进回退
            'interface_spacing': 8,
            'retry_with_relaxed_params': True     # 允许参数放宽重试
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
            'unloading_to_parking': 0
        }
        
        print("初始化优化版骨干路径网络（完整路径组合）")
    
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
            print(f"  混合A*成功: {self.stats['astar_success']}")
            print(f"  RRT成功: {self.stats['rrt_success']}")
            print(f"  直线回退: {self.stats['direct_fallback']}")
            
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
        """
        生成完整的双向路径组合 - 修复版
        确保生成：装载点↔卸载点、装载点↔停车场、卸载点↔停车场
        """
        if not self.path_planner:
            print("❌ 未设置路径规划器")
            return 0
        
        success_count = 0
        path_pairs = []
        
        # 定义需要连接的点类型组合 - 明确指定所有组合
        connection_types = [
            ('loading', 'unloading'),   # 装载点 ↔ 卸载点
            ('loading', 'parking'),     # 装载点 ↔ 停车场
            ('unloading', 'parking')    # 卸载点 ↔ 停车场
        ]
        
        print("生成骨干路径连接：")
        for type_a, type_b in connection_types:
            count_a = len(self.special_points[type_a])
            count_b = len(self.special_points[type_b])
            
            # 跳过空的点类型
            if count_a == 0 or count_b == 0:
                print(f"  跳过 {type_a} ↔ {type_b} (缺少点)")
                continue
                
            print(f"  {type_a} ({count_a}个) ↔ {type_b} ({count_b}个)")
        
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
            print(f"\n[{i}/{len(path_pairs)}] 生成路径: {point_a['type'][0].upper()}{point_a['id']} ↔ {point_b['type'][0].upper()}{point_b['id']} ({connection_type})")
            
            path_result = self._generate_single_bidirectional_path(point_a, point_b)
            
            if path_result:
                success_count += 1
                
                # 更新连接类型统计
                self.stats[connection_type] += 1
                
                print(f"  ✅ 成功: 长度{len(path_result.forward_path)}, "
                      f"质量{path_result.quality:.2f}, "
                      f"规划器: {path_result.planner_used}")
            else:
                print(f"  ❌ 失败")
        
        return success_count
    
    def _generate_single_bidirectional_path(self, point_a: Dict, point_b: Dict) -> Optional[BiDirectionalPath]:
        """
        生成单条双向路径 - 渐进回退策略
        """
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
            
            print(f"    尝试策略: {strategy['name']}")
            
            try:
                # 尝试双向规划，选择更好的方向
                result_ab = self._plan_with_strategy(start_pos, end_pos, strategy, f"{path_id}_AB")
                result_ba = self._plan_with_strategy(end_pos, start_pos, strategy, f"{path_id}_BA")
                
                # 选择更好的结果
                best_result = None
                best_direction = None
                
                if result_ab and result_ba:
                    if result_ab[1] >= result_ba[1]:  # 比较质量
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
                print(f"      策略失败: {e}")
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
                    
                    # 质量检查
                    if quality >= strategy['quality_threshold']:
                        return (result.path, quality)
            
        except Exception as e:
            print(f"        规划异常: {e}")
        
        return None
    
    def _reverse_path(self, path: List[Tuple]) -> List[Tuple]:
        """反转路径方向"""
        if not path:
            return []
        
        reversed_path = []
        for point in reversed(path):
            if len(point) >= 3:
                # 反转角度
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
                
                # 简化接口存储
                self.backbone_interfaces[interface_id] = {
                    'id': interface_id,
                    'position': forward_path[i],
                    'path_id': path_id,
                    'path_index': i,
                    'is_occupied': False
                }
                
                self.path_interfaces[path_id].append(interface_id)
                interface_count += 1
                total_interfaces += 1
            
            print(f"  路径 {path_id}: 生成 {interface_count} 个接口")
        
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
        
        print(f"建立连接索引: {len(self.connection_index)} 个连接")
    
    def get_path_from_position_to_target(self, current_position: Tuple, 
                                       target_type: str, target_id: int) -> Optional[Tuple]:
        """
        优化的路径查找 - 利用双向路径复用
        """
        print(f"查找路径: {current_position} -> {target_type}_{target_id}")
        
        # 查找所有到达目标的双向路径
        candidate_paths = []
        
        for path_id, path_data in self.bidirectional_paths.items():
            # 检查是否连接到目标
            if ((path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id) or
                (path_data.point_b['type'] == target_type and path_data.point_b['id'] == target_id)):
                
                candidate_paths.append(path_data)
        
        if not candidate_paths:
            print(f"  没有找到到 {target_type}_{target_id} 的骨干路径")
            return self._direct_planning_fallback(current_position, target_type, target_id)
        
        # 找到最佳路径
        best_option = self._find_best_bidirectional_option(current_position, candidate_paths, target_type, target_id)
        
        if not best_option:
            return self._direct_planning_fallback(current_position, target_type, target_id)
        
        # 构建完整路径
        complete_path, structure = self._build_complete_bidirectional_path(current_position, best_option)
        
        if complete_path:
            print(f"  ✅ 双向骨干路径成功: 总长度{len(complete_path)}")
            return complete_path, structure
        
        return None
    
    def _find_best_bidirectional_option(self, current_position: Tuple, 
                                      candidate_paths: List, target_type: str, target_id: int) -> Optional[Dict]:
        """找到最佳双向路径选项"""
        best_option = None
        min_total_distance = float('inf')
        
        for path_data in candidate_paths:
            # 确定使用方向
            if path_data.point_a['type'] == target_type and path_data.point_a['id'] == target_id:
                # 需要B->A方向
                backbone_path = path_data.reverse_path
                target_point = path_data.point_a['position']
            else:
                # 需要A->B方向
                backbone_path = path_data.forward_path
                target_point = path_data.point_b['position']
            
            # 找到最近的接口点
            best_interface_index = 0
            min_interface_distance = float('inf')
            
            for i in range(0, len(backbone_path), self.config['interface_spacing']):
                interface_pos = backbone_path[i]
                distance = self._calculate_distance(current_position, interface_pos)
                
                if distance < min_interface_distance:
                    min_interface_distance = distance
                    best_interface_index = i
            
            # 计算总距离
            remaining_path = backbone_path[best_interface_index:]
            backbone_distance = self._calculate_path_length(remaining_path)
            total_distance = min_interface_distance + backbone_distance
            
            if total_distance < min_total_distance:
                min_total_distance = total_distance
                best_option = {
                    'path_data': path_data,
                    'backbone_path': backbone_path,
                    'interface_index': best_interface_index,
                    'interface_position': backbone_path[best_interface_index],
                    'remaining_path': remaining_path,
                    'access_distance': min_interface_distance,
                    'total_distance': total_distance
                }
        
        return best_option
    
    def _build_complete_bidirectional_path(self, current_position: Tuple, 
                                         best_option: Dict) -> Tuple[Optional[List], Dict]:
        """构建完整的双向路径"""
        interface_pos = best_option['interface_position']
        remaining_path = best_option['remaining_path']
        
        # 如果距离接口很近，直接使用骨干路径
        if best_option['access_distance'] < 3.0:
            structure = {
                'type': 'bidirectional_backbone_only',
                'path_id': best_option['path_data'].path_id,
                'backbone_utilization': 1.0,
                'total_length': len(remaining_path)
            }
            return remaining_path, structure
        
        # 规划接入路径
        if not self.path_planner:
            return None, {}
        
        try:
            access_result = self.path_planner.plan_path(
                vehicle_id="bidirectional_access",
                start=current_position,
                goal=interface_pos,
                use_backbone=False
            )
            
            if access_result and hasattr(access_result, 'path') and access_result.path:
                # 合并路径
                complete_path = access_result.path[:-1] + remaining_path
                
                structure = {
                    'type': 'bidirectional_interface_assisted',
                    'path_id': best_option['path_data'].path_id,
                    'access_path': access_result.path,
                    'backbone_path': remaining_path,
                    'backbone_utilization': len(remaining_path) / len(complete_path),
                    'total_length': len(complete_path)
                }
                
                # 增加使用计数
                best_option['path_data'].increment_usage()
                
                return complete_path, structure
            elif access_result and isinstance(access_result, list):
                # 兼容返回list的情况
                complete_path = access_result[:-1] + remaining_path
                
                structure = {
                    'type': 'bidirectional_interface_assisted',
                    'path_id': best_option['path_data'].path_id,
                    'access_path': access_result,
                    'backbone_path': remaining_path,
                    'backbone_utilization': len(remaining_path) / len(complete_path),
                    'total_length': len(complete_path)
                }
                
                best_option['path_data'].increment_usage()
                return complete_path, structure
        
        except Exception as e:
            print(f"    接入路径规划失败: {e}")
        
        return None, {}
    
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
            
            if result and hasattr(result, 'path') and result.path:
                structure = {
                    'type': 'direct_fallback',
                    'backbone_utilization': 0.0,
                    'total_length': len(result.path)
                }
                
                print(f"  ✅ 直接规划回退成功: 长度{len(result.path)}")
                return result.path, structure
            elif result and isinstance(result, list):
                structure = {
                    'type': 'direct_fallback',
                    'backbone_utilization': 0.0,
                    'total_length': len(result)
                }
                
                print(f"  ✅ 直接规划回退成功: 长度{len(result)}")
                return result, structure
        
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
    
    def get_network_status(self) -> Dict:
        """获取网络状态 - 增强版本"""
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
            }
        }
    
    def debug_network_info(self):
        """调试网络信息 - 增强版本"""
        print("=== 完整骨干网络调试信息 ===")
        print(f"双向路径数量: {len(self.bidirectional_paths)}")
        
        # 按类型分组显示
        connection_groups = {
            'loading_to_unloading': [],
            'loading_to_parking': [],
            'unloading_to_parking': []
        }
        
        for path_id, path_data in self.bidirectional_paths.items():
            type_a = path_data.point_a['type']
            type_b = path_data.point_b['type']
            
            if (type_a == 'loading' and type_b == 'unloading') or (type_a == 'unloading' and type_b == 'loading'):
                connection_groups['loading_to_unloading'].append((path_id, path_data))
            elif (type_a == 'loading' and type_b == 'parking') or (type_a == 'parking' and type_b == 'loading'):
                connection_groups['loading_to_parking'].append((path_id, path_data))
            elif (type_a == 'unloading' and type_b == 'parking') or (type_a == 'parking' and type_b == 'unloading'):
                connection_groups['unloading_to_parking'].append((path_id, path_data))
        
        for group_name, paths in connection_groups.items():
            print(f"\n{group_name.replace('_', ' ').title()} ({len(paths)} 条):")
            for path_id, path_data in paths:
                print(f"  路径 {path_id}:")
                print(f"    连接: {path_data.point_a['type']}_{path_data.point_a['id']} ↔ {path_data.point_b['type']}_{path_data.point_b['id']}")
                print(f"    长度: {len(path_data.forward_path)} 点, {path_data.length:.1f} 距离")
                print(f"    质量: {path_data.quality:.2f}")
                print(f"    规划器: {path_data.planner_used}")
                print(f"    使用次数: {path_data.usage_count}")
        
        print(f"\n生成统计:")
        print(f"  总路径对: {self.stats['total_path_pairs']}")
        print(f"  成功路径: {self.stats['successful_paths']}")
        print(f"  混合A*: {self.stats['astar_success']}")
        print(f"  RRT: {self.stats['rrt_success']}")
        print(f"  直线回退: {self.stats['direct_fallback']}")
        print(f"  装载点↔卸载点: {self.stats['loading_to_unloading']}")
        print(f"  装载点↔停车场: {self.stats['loading_to_parking']}")
        print(f"  卸载点↔停车场: {self.stats['unloading_to_parking']}")

# 向后兼容性
SimplifiedBackboneNetwork = OptimizedBackboneNetwork