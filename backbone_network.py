import math
import time
import threading
import numpy as np
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Optional, Any  # 添加缺失的导入
from RRT import OptimizedRRTPlanner

class BackboneInterface:
    """骨干路径接口点"""
    def __init__(self, interface_id, position, direction, backbone_path_id, 
                 path_index, access_difficulty=0.0):
        self.interface_id = interface_id
        self.position = position  # (x, y, theta)
        self.direction = direction  # 接口进入方向
        self.backbone_path_id = backbone_path_id
        self.path_index = path_index  # 在骨干路径中的索引
        self.access_difficulty = access_difficulty  # 接入难度评估
        self.usage_count = 0  # 使用次数统计
        self.is_occupied = False  # 是否被占用
        self.occupied_by = None  # 被哪个车辆占用
        self.reservation_time = None  # 预约时间
        
    def reserve(self, vehicle_id, duration=30):
        """预约接口"""
        self.is_occupied = True
        self.occupied_by = vehicle_id
        self.reservation_time = time.time() + duration
        
    def release(self):
        """释放接口"""
        self.is_occupied = False
        self.occupied_by = None
        self.reservation_time = None
        
    def is_available(self):
        """检查接口是否可用"""
        if not self.is_occupied:
            return True
        
        # 检查预约是否过期
        if self.reservation_time and time.time() > self.reservation_time:
            self.release()
            return True
            
        return False


class EnhancedBackboneInterface(BackboneInterface):
    """增强的骨干路径接口 - 支持混合A*预处理"""
    
    def __init__(self, interface_id, position, direction, backbone_path_id, 
                 path_index, access_difficulty=0.0):
        super().__init__(interface_id, position, direction, backbone_path_id, 
                        path_index, access_difficulty)
        
        # 新增混合A*集成属性
        self.astar_sampling_weight = 1.0  # 混合A*采样权重
        self.accessibility_score = 0.5  # 可达性评分
        self.usage_efficiency = 0.0     # 使用效率
        self.last_quality_score = 0.0   # 最近路径质量
        
        # 性能统计
        self.astar_cache_hits = 0
        self.total_planning_attempts = 0
        self.average_planning_time = 0.0
        
        # 区域影响
        self.influence_radius = 15.0    # 影响半径
        self.sampling_hotspot = False   # 是否为采样热点
    
    def update_astar_statistics(self, planning_time, path_quality, cache_hit=False):
        """更新混合A*相关统计信息"""
        self.total_planning_attempts += 1
        if cache_hit:
            self.astar_cache_hits += 1
        
        # 更新平均规划时间
        alpha = 0.1  # 学习率
        if self.average_planning_time == 0:
            self.average_planning_time = planning_time
        else:
            self.average_planning_time = (1-alpha) * self.average_planning_time + alpha * planning_time
        
        # 更新质量评分
        self.last_quality_score = path_quality
        
        # 更新使用效率
        if self.total_planning_attempts > 0:
            self.usage_efficiency = self.astar_cache_hits / self.total_planning_attempts
    
    def calculate_sampling_priority(self):
        """计算混合A*采样优先级"""
        # 基础权重
        priority = self.astar_sampling_weight
        
        # 可达性加权
        priority *= (0.5 + self.accessibility_score)
        
        # 使用频率加权（使用越多优先级越高，但有上限）
        usage_factor = min(2.0, 1.0 + self.usage_count * 0.1)
        priority *= usage_factor
        
        # 效率加权
        if self.usage_efficiency > 0.7:
            priority *= 1.2  # 高效接口优先
        elif self.usage_efficiency < 0.3:
            priority *= 0.8  # 低效接口降权
        
        return priority
    
    def get_influence_region(self):
        """获取接口影响区域（用于混合A*采样）"""
        return {
            'center': (self.position[0], self.position[1]),
            'radius': self.influence_radius,
            'priority': self.calculate_sampling_priority(),
            'direction_bias': self.direction,
            'quality_hint': self.last_quality_score
        }


class SimplifiedBackbonePathNetwork:
    """
    完整的简化骨干路径网络 - 带接口系统优化版
    支持混合A*和RRT规划器
    """
    
    def __init__(self, env):
        self.env = env
        self.backbone_paths = {}  # 骨干路径字典 {path_id: path_data}
        self.special_points = {   # 特殊点分类
            'loading': [],
            'unloading': [],
            'parking': []
        }
        
        # 骨干接口系统
        self.backbone_interfaces = {}  # {interface_id: BackboneInterface}
        self.path_interfaces = defaultdict(list)  # {path_id: [interface_ids]}
        self.interface_spacing = 10  # 接口间距（路径点数）
        
        # 空间索引用于快速查找接口
        self.interface_spatial_index = {}  # 简化的空间索引
        
        # 路径查找索引
        self.paths_to_target = defaultdict(list)  # {(target_type, target_id): [path_ids]}
        self.paths_from_source = defaultdict(list)  # {(source_type, source_id): [path_ids]}
        
        # 规划器配置 - 支持多种规划器
        self.planner = None
        self.planner_type = "hybrid_astar"  # 默认使用混合A*
        self.available_planners = ["hybrid_astar", "rrt", "auto"]  # 可用的规划器类型
        
        # 性能统计
        self.stats = {
            'total_paths': 0,
            'total_interfaces': 0,
            'interface_usage': defaultdict(int),
            'generation_time': 0,
            'average_path_length': 0,
            'path_usage_count': defaultdict(int),
            'total_usage': 0,
            'planner_performance': defaultdict(dict)  # 规划器性能统计
        }
        
        # 混合A*集成增强
        self.astar_integration = {
            'preprocessing_enabled': True,
            'adaptive_sampling': True,
            'quality_feedback': True,
            'cache_coordination': True
        }
        
        # 预处理数据
        self.sampling_regions = {}      # 混合A*采样区域
        self.path_quality_map = {}      # 路径质量映射
        self.access_heatmap = None      # 可达性热力图
        
        # 性能缓存
        self.planner_ref = None         # 规划器引用
        self.path_cache_stats = {
            'total_requests': 0,
            'cache_hits': 0,
            'quality_improvements': 0
        }
        
        print("初始化带接口系统的骨干路径网络（支持混合A*）")
    
    def set_planner_type(self, planner_type: str) -> bool:
        """设置规划器类型"""
        if planner_type in self.available_planners:
            old_type = self.planner_type
            self.planner_type = planner_type
            
            # 重新创建规划器
            self.planner = self._create_planner()
            
            print(f"规划器类型从 {old_type} 切换到 {planner_type}")
            return True
        else:
            print(f"不支持的规划器类型: {planner_type}")
            return False
    
    def set_planner(self, planner):
        """设置外部规划器"""
        self.planner_ref = planner
        if planner:
            # 建立双向连接
            if hasattr(planner, 'set_backbone_network'):
                planner.set_backbone_network(self)
            self._initialize_planner_integration()
    
    def _initialize_planner_integration(self):
        """初始化规划器集成"""
        print("初始化规划器深度集成...")
        
        # 预处理接口区域
        self._preprocess_sampling_regions()
        
        # 构建可达性热力图
        self._build_accessibility_heatmap()
        
        # 启用质量反馈循环
        self._setup_quality_feedback()
        
        print(f"规划器集成完成: {len(self.sampling_regions)}个采样区域")
    
    def _preprocess_sampling_regions(self):
        """预处理规划器采样区域"""
        self.sampling_regions.clear()
        
        if not hasattr(self, 'backbone_interfaces'):
            return
        
        for interface_id, interface in self.backbone_interfaces.items():
            # 升级为增强接口
            if not isinstance(interface, EnhancedBackboneInterface):
                enhanced_interface = self._upgrade_interface(interface)
                self.backbone_interfaces[interface_id] = enhanced_interface
                interface = enhanced_interface
            
            # 计算接口影响区域
            region = interface.get_influence_region()
            self.sampling_regions[interface_id] = region
        
        # 添加骨干路径中点作为采样区域
        self._add_backbone_midpoint_regions()
    
    def _upgrade_interface(self, old_interface):
        """升级接口为增强版本"""
        enhanced = EnhancedBackboneInterface(
            old_interface.interface_id,
            old_interface.position,
            old_interface.direction,
            old_interface.backbone_path_id,
            old_interface.path_index,
            old_interface.access_difficulty
        )
        
        # 传递统计数据
        enhanced.usage_count = old_interface.usage_count
        enhanced.is_occupied = old_interface.is_occupied
        enhanced.occupied_by = old_interface.occupied_by
        enhanced.reservation_time = old_interface.reservation_time
        
        return enhanced
    
    def _add_backbone_midpoint_regions(self):
        """添加骨干路径中点作为采样区域"""
        for path_id, path_data in self.backbone_paths.items():
            path = path_data.get('path', [])
            if len(path) > 10:  # 只处理较长的路径
                # 在路径中点添加采样区域
                mid_index = len(path) // 2
                mid_point = path[mid_index]
                
                region_id = f"{path_id}_midpoint"
                self.sampling_regions[region_id] = {
                    'center': (mid_point[0], mid_point[1]),
                    'radius': 12.0,
                    'priority': 0.8,
                    'direction_bias': mid_point[2] if len(mid_point) > 2 else 0,
                    'quality_hint': path_data.get('quality', 0.5)
                }
    
    def _build_accessibility_heatmap(self):
        """构建可达性热力图"""
        if not self.env:
            return
        
        # 简化的热力图：基于距离障碍物的距离
        grid_size = 10  # 热力图网格大小
        width_cells = self.env.width // grid_size
        height_cells = self.env.height // grid_size
        
        self.access_heatmap = np.zeros((width_cells, height_cells))
        
        for i in range(width_cells):
            for j in range(height_cells):
                # 计算网格中心点
                center_x = i * grid_size + grid_size // 2
                center_y = j * grid_size + grid_size // 2
                
                # 计算可达性评分
                accessibility = self._calculate_point_accessibility(center_x, center_y)
                self.access_heatmap[i, j] = accessibility
    
    def _calculate_point_accessibility(self, x, y):
        """计算点的可达性评分"""
        # 基于周围障碍物密度
        obstacle_count = 0
        total_cells = 0
        check_radius = 5
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x, check_y = x + dx, y + dy
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height):
                    total_cells += 1
                    if hasattr(self.env, 'grid') and self.env.grid[check_x, check_y] == 1:
                        obstacle_count += 1
        
        if total_cells == 0:
            return 0
        
        return 1.0 - (obstacle_count / total_cells)
    
    def _setup_quality_feedback(self):
        """设置质量反馈循环"""
        # 这里可以设置质量反馈机制
        pass
    
    def generate_backbone_network(self, quality_threshold=0.4, interface_spacing=8):
        """
        生成骨干路径网络并创建接口点 - 完整版本
        """
        self.interface_spacing = interface_spacing
        
        start_time = time.time()
        print("开始生成骨干路径网络...")
        
        try:
            # 1. 读取和分类特殊点
            self._load_special_points()
            print(f"特殊点统计: 装载点{len(self.special_points['loading'])}个, "
                  f"卸载点{len(self.special_points['unloading'])}个, "
                  f"停车点{len(self.special_points['parking'])}个")
            
            # 2. 创建规划器
            if not self.planner:
                self.planner = self._create_planner()
                
            if not self.planner:
                print("无法创建路径规划器，骨干网络生成失败")
                return False
            
            # 3. 生成骨干路径
            self._generate_backbone_paths(quality_threshold)
            
            # 4. 生成骨干接口点
            self._generate_backbone_interfaces_simplified()
            
            # 5. 建立查找索引
            self._build_path_indexes()
            self._build_interface_spatial_index()
            
            # 6. 如果有规划器集成，初始化相关系统
            if self.planner_ref:
                self._initialize_planner_integration()
            
            # 7. 统计信息
            generation_time = time.time() - start_time
            self.stats['generation_time'] = generation_time
            self.stats['total_paths'] = len(self.backbone_paths)
            self.stats['total_interfaces'] = len(self.backbone_interfaces)
            
            if self.backbone_paths:
                total_length = sum(path_data['length'] for path_data in self.backbone_paths.values())
                self.stats['average_path_length'] = total_length / len(self.backbone_paths)
            
            print(f"骨干路径网络生成完成!")
            print(f"- 总路径数: {len(self.backbone_paths)}")
            print(f"- 总接口数: {len(self.backbone_interfaces)}")
            print(f"- 接口间距: {self.interface_spacing} 个路径点")
            print(f"- 生成耗时: {generation_time:.2f}秒")
            print(f"- 平均路径长度: {self.stats['average_path_length']:.1f}")
            print(f"- 使用规划器: {self.planner_type}")
            
            return True
            
        except Exception as e:
            print(f"生成骨干路径网络失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _generate_backbone_interfaces_simplified(self):
        """简化的接口生成方法 - 在每条骨干路径上均匀分布接口"""
        print("正在生成骨干路径接口点...")
        
        total_interfaces = 0
        
        for path_id, path_data in self.backbone_paths.items():
            path = path_data['path']
            path_length = len(path)
            
            if path_length < 2:
                print(f"⚠️ 路径 {path_id} 太短({path_length}个点)，跳过接口生成")
                continue
                
            interfaces_for_path = []
            interface_count = 0
            
            # 从路径起点开始，每隔interface_spacing个点设置一个接口
            for i in range(0, path_length, self.interface_spacing):
                if i >= path_length:
                    break
                    
                # 计算接口方向
                direction = self._calculate_interface_direction(path, i)
                
                # 创建增强接口
                interface_id = f"{path_id}_if_{interface_count}"
                interface = EnhancedBackboneInterface(
                    interface_id=interface_id,
                    position=path[i],
                    direction=direction,
                    backbone_path_id=path_id,
                    path_index=i,
                    access_difficulty=self._evaluate_interface_access_difficulty(path, i)
                )
                
                # 计算可达性评分
                interface.accessibility_score = self._calculate_point_accessibility(
                    int(path[i][0]), int(path[i][1])
                )
                
                # 存储接口
                self.backbone_interfaces[interface_id] = interface
                interfaces_for_path.append(interface_id)
                total_interfaces += 1
                interface_count += 1
            
            # 确保路径终点也有接口（如果终点不在间距点上）
            last_index = path_length - 1
            if last_index > 0 and last_index % self.interface_spacing != 0:
                last_interface_id = f"{path_id}_if_end"
                direction = self._calculate_interface_direction(path, last_index)
                last_interface = EnhancedBackboneInterface(
                    interface_id=last_interface_id,
                    position=path[last_index],
                    direction=direction,
                    backbone_path_id=path_id,
                    path_index=last_index,
                    access_difficulty=self._evaluate_interface_access_difficulty(path, last_index)
                )
                
                last_interface.accessibility_score = self._calculate_point_accessibility(
                    int(path[last_index][0]), int(path[last_index][1])
                )
                
                self.backbone_interfaces[last_interface_id] = last_interface
                interfaces_for_path.append(last_interface_id)
                total_interfaces += 1
            
            self.path_interfaces[path_id] = interfaces_for_path
            print(f"   路径 {path_id}: 生成 {len(interfaces_for_path)} 个接口")
        
        print(f"成功生成 {total_interfaces} 个骨干接口点")
    
    def _calculate_interface_direction(self, path, index):
        """计算接口的方向角"""
        if index < len(path) - 1:
            # 使用当前点到下一点的方向
            dx = path[index + 1][0] - path[index][0]
            dy = path[index + 1][1] - path[index][1]
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                return math.atan2(dy, dx)
        
        # 如果是最后一个点或者方向向量为零，使用点本身的朝向
        return path[index][2] if len(path[index]) > 2 else 0.0
    
    def _evaluate_interface_access_difficulty(self, path, index):
        """评估接口的接入难度"""
        difficulty = 0.0
        
        # 基于周围障碍物密度
        x, y = int(path[index][0]), int(path[index][1])
        obstacle_count = 0
        search_radius = 5
        
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                check_x, check_y = x + dx, y + dy
                if (0 <= check_x < self.env.width and 
                    0 <= check_y < self.env.height and
                    hasattr(self.env, 'grid') and
                    self.env.grid[check_x, check_y] == 1):
                    obstacle_count += 1
        
        difficulty += obstacle_count * 0.1
        
        # 基于路径曲率（转弯越急难度越高）
        if index > 0 and index < len(path) - 1:
            curvature = self._calculate_path_curvature(path, index)
            difficulty += curvature * 5
        
        return difficulty
    
    def _calculate_path_curvature(self, path, index):
        """计算路径在指定点的曲率"""
        if index <= 0 or index >= len(path) - 1:
            return 0.0
        
        p1 = path[index - 1]
        p2 = path[index]
        p3 = path[index + 1]
        
        # 使用三点法计算曲率
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        len_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
        len_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
        
        if len_v1 < 0.001 or len_v2 < 0.001:
            return 0.0
        
        cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len_v1 * len_v2)
        cos_angle = max(-1, min(1, cos_angle))
        angle = math.acos(cos_angle)
        
        # 曲率 = 角度变化 / 平均段长
        avg_length = (len_v1 + len_v2) / 2
        return angle / (avg_length + 0.001)
    
    def _build_interface_spatial_index(self):
        """建立接口的空间索引"""
        self.interface_spatial_index = {}
        
        # 简化的网格索引
        grid_size = 20  # 网格大小
        
        for interface_id, interface in self.backbone_interfaces.items():
            x, y = interface.position[0], interface.position[1]
            grid_x = int(x // grid_size)
            grid_y = int(y // grid_size)
            grid_key = (grid_x, grid_y)
            
            if grid_key not in self.interface_spatial_index:
                self.interface_spatial_index[grid_key] = []
            
            self.interface_spatial_index[grid_key].append(interface_id)
    
    def find_nearest_interface(self, position, target_type, target_id, max_distance=50, debug=True):
        """找到最近的可用骨干接口"""
        if debug:
            print(f"🔍 查找从 {position} 到 {target_type}_{target_id} 的接口")
        
        # 找到所有通向目标的骨干路径
        target_key = (target_type, target_id)
        target_paths = self.paths_to_target.get(target_key, [])
        
        if not target_paths:
            if debug:
                print(f"❌ 没有找到通向 {target_type}_{target_id} 的骨干路径")
            return None
        
        # 收集候选接口
        candidate_interfaces = []
        
        for path_data in target_paths:
            path_id = path_data['id']
            
            if path_id not in self.path_interfaces:
                continue
                
            for interface_id in self.path_interfaces[path_id]:
                interface = self.backbone_interfaces[interface_id]
                
                if not interface.is_available():
                    continue
                
                distance = self._calculate_distance(position, interface.position)
                if distance <= max_distance:
                    candidate_interfaces.append((interface, distance, path_id))
        
        if not candidate_interfaces:
            if debug:
                print(f"❌ 在距离 {max_distance} 内没有找到可用接口")
            return None
        
        # 选择最佳接口
        best_interface = min(candidate_interfaces, key=lambda x: x[1])[0]
        
        if debug:
            print(f"✅ 选择接口: {best_interface.interface_id}")
        
        return best_interface
    
    def get_complete_path_via_interface_enhanced(self, start, target_type, target_id, hints=None):
        """增强版路径获取 - 支持多种规划器"""
        # 获取基础路径
        base_result = self.get_path_from_position_to_target_via_interface(
            start, target_type, target_id
        )
        
        if not base_result or not base_result[0]:
            return base_result
        
        path, structure = base_result
        
        # 如果有提示信息，进行路径优化
        if hints and self.planner_ref:
            optimized_path = self._apply_planner_hints(path, hints)
            if optimized_path:
                path = optimized_path
                structure['planner_optimized'] = True
                structure['optimizer_type'] = self.planner_type
        
        return path, structure
    
    def _apply_planner_hints(self, path, hints):
        """应用规划器提示优化路径"""
        try:
            if 'smoothing_suggested' in hints:
                return self._smooth_path_with_planner(path)
            
            if 'density_adjustment' in hints:
                return self._adjust_path_density_smart(path, hints['target_density'])
        
        except Exception as e:
            print(f"规划器提示应用失败: {e}")
        
        return path
    
    def _smooth_path_with_planner(self, path):
        """使用规划器优化器平滑路径"""
        if self.planner_ref and hasattr(self.planner_ref, '_adaptive_smoothing'):
            return self.planner_ref._adaptive_smoothing(path)
        return path
    
    def _adjust_path_density_smart(self, path, target_density):
        """智能调整路径密度"""
        if self.planner_ref and hasattr(self.planner_ref, '_adjust_path_density'):
            return self.planner_ref._adjust_path_density(path, target_density)
        return path
    
    def get_sampling_guidance_for_planner(self, start, goal):
        """为规划器提供采样引导信息"""
        guidance = {
            'priority_regions': [],
            'avoid_regions': [],
            'backbone_hints': [],
            'interface_targets': []
        }
        
        # 添加相关的采样区域
        for region_id, region in self.sampling_regions.items():
            relevance = self._calculate_region_relevance(region, start, goal)
            
            if relevance > 0.3:
                guidance['priority_regions'].append({
                    'region': region,
                    'relevance': relevance,
                    'id': region_id
                })
        
        # 排序并限制数量
        guidance['priority_regions'].sort(key=lambda x: x['relevance'], reverse=True)
        guidance['priority_regions'] = guidance['priority_regions'][:10]
        
        return guidance
    
    def _calculate_region_relevance(self, region, start, goal):
        """计算区域与起终点的相关性"""
        center = region['center']
        
        # 计算到起点和终点的距离
        dist_to_start = math.sqrt((center[0] - start[0])**2 + (center[1] - start[1])**2)
        dist_to_goal = math.sqrt((center[0] - goal[0])**2 + (center[1] - goal[1])**2)
        
        # 计算起终点直线距离
        direct_distance = math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
        
        # 相关性基于区域是否在合理的路径范围内
        max_detour = direct_distance * 1.5
        total_distance = dist_to_start + dist_to_goal
        
        if total_distance <= max_detour:
            base_relevance = 1.0 - (total_distance - direct_distance) / (max_detour - direct_distance)
            priority_weight = region.get('priority', 1.0)
            return base_relevance * priority_weight
        
        return 0.0
    
    def get_path_from_position_to_target_via_interface(self, current_position, target_type, target_id):
        """
        通过骨干接口获取从当前位置到目标的完整路径
        """
        # 1. 查找最佳骨干接口
        best_interface = self.find_nearest_interface(current_position, target_type, target_id)
        
        if not best_interface:
            print(f"未找到到 {target_type}_{target_id} 的可用骨干接口")
            return None, None
        
        # 2. 预约接口
        best_interface.reserve("vehicle_temp", duration=60)
        
        # 3. 规划从当前位置到接口的接入路径
        interface_position = best_interface.position
        
        # 如果当前位置就在接口附近，直接使用骨干路径
        if self._calculate_distance(current_position, interface_position) < 3.0:
            print(f"当前位置接近骨干接口 {best_interface.interface_id}，直接使用骨干路径")
            
            # 获取从接口到目标的骨干路径段
            backbone_segment = self._get_backbone_segment_from_interface(best_interface, target_type, target_id)
            
            return backbone_segment, {
                'type': 'backbone_only',
                'interface_id': best_interface.interface_id,
                'backbone_path_id': best_interface.backbone_path_id,
                'backbone_utilization': 1.0,
                'access_length': 0,
                'backbone_length': len(backbone_segment) if backbone_segment else 0,
                'total_length': len(backbone_segment) if backbone_segment else 0,
                'planner_used': self.planner_type
            }
        
        # 4. 规划接入路径
        print(f"规划到骨干接口 {best_interface.interface_id} 的接入路径")
        access_path = self.planner.plan_path(current_position, interface_position, max_iterations=3000)
        
        if not access_path or len(access_path) < 2:
            print("接入路径规划失败")
            best_interface.release()  # 释放接口预约
            return None, None
        
        # 5. 获取骨干路径段
        backbone_segment = self._get_backbone_segment_from_interface(best_interface, target_type, target_id)
        
        if not backbone_segment:
            print("获取骨干路径段失败")
            best_interface.release()
            return None, None
        
        # 6. 拼接路径
        complete_path = self._merge_paths(access_path, backbone_segment)
        
        if not complete_path:
            print("路径拼接失败")
            best_interface.release()
            return None, None
        
        # 7. 构建路径结构信息
        structure = {
            'type': 'interface_assisted',
            'interface_id': best_interface.interface_id,
            'backbone_path_id': best_interface.backbone_path_id,
            'access_path': access_path,
            'backbone_path': backbone_segment,
            'backbone_utilization': len(backbone_segment) / len(complete_path),
            'access_length': len(access_path),
            'backbone_length': len(backbone_segment),
            'total_length': len(complete_path),
            'planner_used': self.planner_type
        }
        
        # 8. 更新使用统计
        best_interface.usage_count += 1
        self.stats['interface_usage'][best_interface.interface_id] += 1
        self.stats['total_usage'] += 1
        
        # 9. 更新规划器性能统计
        if self.planner_type not in self.stats['planner_performance']:
            self.stats['planner_performance'][self.planner_type] = {
                'total_usage': 0,
                'success_count': 0,
                'average_path_length': 0
            }
        
        planner_stats = self.stats['planner_performance'][self.planner_type]
        planner_stats['total_usage'] += 1
        planner_stats['success_count'] += 1
        planner_stats['average_path_length'] = (
            (planner_stats['average_path_length'] * (planner_stats['success_count'] - 1) + 
             len(complete_path)) / planner_stats['success_count']
        )
        
        print(f"接口辅助路径生成成功: 总长度{len(complete_path)}, "
              f"骨干利用率{structure['backbone_utilization']:.2f}, "
              f"使用接口{best_interface.interface_id}, "
              f"规划器{self.planner_type}")
        
        return complete_path, structure
    
    def update_path_feedback(self, path, planning_time, quality_score, used_cache=False):
        """更新路径反馈信息"""
        if used_cache:
            self.path_cache_stats['cache_hits'] += 1
        
        self.path_cache_stats['total_requests'] += 1
        
        # 更新相关接口的统计信息
        used_interfaces = self._identify_path_interfaces(path)
        
        for interface_id in used_interfaces:
            if interface_id in self.backbone_interfaces:
                interface = self.backbone_interfaces[interface_id]
                if hasattr(interface, 'update_astar_statistics'):
                    interface.update_astar_statistics(planning_time, quality_score, used_cache)
    
    def _identify_path_interfaces(self, path):
        """识别路径使用的接口"""
        used_interfaces = []
        
        if not path:
            return used_interfaces
        
        # 检查路径点是否接近接口
        for interface_id, interface in self.backbone_interfaces.items():
            interface_pos = interface.position
            
            for path_point in path[::5]:  # 每隔5个点检查一次
                distance = math.sqrt(
                    (path_point[0] - interface_pos[0])**2 + 
                    (path_point[1] - interface_pos[1])**2
                )
                
                if distance < 5.0:  # 如果路径接近接口
                    used_interfaces.append(interface_id)
                    break
        
        return used_interfaces
    
    def get_planner_performance_stats(self):
        """获取规划器相关的性能统计"""
        stats = {
            'cache_stats': self.path_cache_stats.copy(),
            'interface_performance': {},
            'sampling_region_count': len(self.sampling_regions),
            'heatmap_available': self.access_heatmap is not None,
            'current_planner': self.planner_type,
            'planner_performance': self.stats['planner_performance'].copy()
        }
        
        # 接口性能统计
        for interface_id, interface in self.backbone_interfaces.items():
            if hasattr(interface, 'total_planning_attempts'):
                stats['interface_performance'][interface_id] = {
                    'usage_count': interface.usage_count,
                    'planning_attempts': interface.total_planning_attempts,
                    'cache_hit_rate': interface.usage_efficiency,
                    'avg_planning_time': interface.average_planning_time,
                    'last_quality': interface.last_quality_score,
                    'accessibility_score': getattr(interface, 'accessibility_score', 0.5)
                }
        
        return stats
    
    def _create_planner(self):
        """创建规划器 - 支持多种类型"""
        try:
            if self.planner_type == "hybrid_astar":
                # 优先使用混合A*规划器
                from astar import HybridAStarPlanner
                planner = HybridAStarPlanner(
                    self.env,
                    vehicle_length=6.0,
                    vehicle_width=3.0,
                    turning_radius=8.0,
                    step_size=2.0,
                    angle_resolution=30
                )
                
                # 设置双向引用
                planner.set_backbone_network(self)
                print(f"✅ 创建混合A*规划器成功")
                return planner
                
            elif self.planner_type == "rrt":
                # 使用RRT规划器
                planner = OptimizedRRTPlanner(
                    self.env,
                    vehicle_length=6.0,
                    vehicle_width=3.0,
                    turning_radius=8.0,
                    step_size=0.8
                )
                
                # 设置双向引用（如果支持）
                if hasattr(planner, 'set_backbone_network'):
                    planner.set_backbone_network(self)
                
                print(f"✅ 创建RRT规划器成功")
                return planner
                
            elif self.planner_type == "auto":
                # 自动选择：优先混合A*，回退到RRT
                try:
                    from astar import HybridAStarPlanner
                    planner = HybridAStarPlanner(
                        self.env,
                        vehicle_length=6.0,
                        vehicle_width=3.0,
                        turning_radius=8.0,
                        step_size=2.0
                    )
                    planner.set_backbone_network(self)
                    self.planner_type = "hybrid_astar"  # 更新实际使用的类型
                    print(f"✅ 自动选择：创建混合A*规划器成功")
                    return planner
                except Exception as e:
                    print(f"⚠️ 混合A*创建失败，回退到RRT: {e}")
                    planner = OptimizedRRTPlanner(
                        self.env,
                        vehicle_length=6.0,
                        vehicle_width=3.0,
                        turning_radius=8.0,
                        step_size=0.8
                    )
                    if hasattr(planner, 'set_backbone_network'):
                        planner.set_backbone_network(self)
                    self.planner_type = "rrt"  # 更新实际使用的类型
                    print(f"✅ 自动选择：创建RRT规划器成功")
                    return planner
                    
        except Exception as e:
            print(f"⚠️ 创建 {self.planner_type} 规划器失败: {e}")
            
            # 最后的回退方案
            try:
                from RRT import RRTPlanner
                planner = RRTPlanner(
                    self.env,
                    vehicle_length=6.0,
                    vehicle_width=3.0,
                    turning_radius=8.0,
                    step_size=0.8
                )
                self.planner_type = "rrt_basic"
                print(f"✅ 回退：创建基础RRT规划器成功")
                return planner
            except Exception as e2:
                print(f"❌ 所有规划器创建失败: {e2}")
                return None
    
    # 以下方法保持原有逻辑，从原始代码复制...
    def _load_special_points(self):
        """载入和分类特殊点"""
        # 装载点
        self.special_points['loading'] = []
        for i, point in enumerate(self.env.loading_points):
            self.special_points['loading'].append({
                'id': i,
                'type': 'loading',
                'position': self._ensure_3d_point(point),
                'capacity': 5  # 默认容量
            })
        
        # 卸载点
        self.special_points['unloading'] = []
        for i, point in enumerate(self.env.unloading_points):
            self.special_points['unloading'].append({
                'id': i,
                'type': 'unloading', 
                'position': self._ensure_3d_point(point),
                'capacity': 5
            })
        
        # 停车点
        self.special_points['parking'] = []
        parking_areas = getattr(self.env, 'parking_areas', [])
        for i, point in enumerate(parking_areas):
            self.special_points['parking'].append({
                'id': i,
                'type': 'parking',
                'position': self._ensure_3d_point(point),
                'capacity': 10
            })
    
    def _generate_backbone_paths(self, quality_threshold):
        """生成特殊点之间的骨干路径"""
        path_count = 0
        
        print("生成装载点 ↔ 卸载点路径...")
        # 装载点 → 卸载点 (双向)
        for loading_point in self.special_points['loading']:
            for unloading_point in self.special_points['unloading']:
                # 正向路径
                path_id = f"L{loading_point['id']}_to_U{unloading_point['id']}"
                if self._generate_single_path(loading_point, unloading_point, path_id, quality_threshold):
                    path_count += 1
                
                # 反向路径
                reverse_path_id = f"U{unloading_point['id']}_to_L{loading_point['id']}"
                if self._generate_single_path(unloading_point, loading_point, reverse_path_id, quality_threshold):
                    path_count += 1
        
        print("生成装载点 ↔ 停车点路径...")
        # 装载点 → 停车点 (双向)
        for loading_point in self.special_points['loading']:
            for parking_point in self.special_points['parking']:
                # 正向路径
                path_id = f"L{loading_point['id']}_to_P{parking_point['id']}"
                if self._generate_single_path(loading_point, parking_point, path_id, quality_threshold):
                    path_count += 1
                
                # 反向路径
                reverse_path_id = f"P{parking_point['id']}_to_L{loading_point['id']}"
                if self._generate_single_path(parking_point, loading_point, reverse_path_id, quality_threshold):
                    path_count += 1
        
        print("生成卸载点 ↔ 停车点路径...")
        # 卸载点 → 停车点 (双向)
        for unloading_point in self.special_points['unloading']:
            for parking_point in self.special_points['parking']:
                # 正向路径
                path_id = f"U{unloading_point['id']}_to_P{parking_point['id']}"
                if self._generate_single_path(unloading_point, parking_point, path_id, quality_threshold):
                    path_count += 1
                
                # 反向路径
                reverse_path_id = f"P{parking_point['id']}_to_U{unloading_point['id']}"
                if self._generate_single_path(parking_point, unloading_point, reverse_path_id, quality_threshold):
                    path_count += 1
        
        print(f"成功生成 {path_count} 条骨干路径（使用{self.planner_type}规划器）")
    
    def _generate_single_path(self, start_point, end_point, path_id, quality_threshold):
        """生成单条骨干路径 - 支持多种规划器"""
        try:
            start_pos = start_point['position']
            end_pos = end_point['position']
            
            # 多次尝试增加成功率，根据规划器类型调整参数
            max_attempts = 5
            if self.planner_type == "hybrid_astar":
                max_attempts = 3  # 混合A*通常更稳定，减少尝试次数
            
            for attempt in range(max_attempts):
                if self.planner_type == "hybrid_astar":
                    # 混合A*参数
                    max_iterations = 6000 + attempt * 2000
                    path = self.planner.plan_path(
                        start_pos, end_pos, 
                        agent_id=f"backbone_{path_id}",
                        max_iterations=max_iterations,
                        quality_threshold=quality_threshold
                    )
                else:
                    # RRT参数
                    max_iterations = 4000 + attempt * 1000
                    if hasattr(self.planner, 'plan_path'):
                        # 新版RRT接口
                        path = self.planner.plan_path(
                            start=start_pos, goal=end_pos,
                            agent_id=f"backbone_{path_id}",
                            max_iterations=max_iterations,
                            quality_threshold=quality_threshold
                        )
                    else:
                        # 旧版RRT接口
                        path = self.planner.plan_path(start_pos, end_pos, max_iterations=max_iterations)
                
                if path and len(path) >= 2:
                    # 评估路径质量
                    quality = self._evaluate_path_quality(path)
                    if quality >= quality_threshold:
                        # 存储路径
                        self.backbone_paths[path_id] = {
                            'id': path_id,
                            'start_point': start_point,
                            'end_point': end_point,
                            'path': path,
                            'length': self._calculate_path_length(path),
                            'quality': quality,
                            'usage_count': 0,
                            'created_time': time.time(),
                            'planner_used': self.planner_type
                        }
                        
                        print(f"✅ 路径 {path_id} 生成成功 (尝试 {attempt+1}, "
                              f"质量: {quality:.2f}, 规划器: {self.planner_type})")
                        return True
                    else:
                        print(f"⚠️ 路径 {path_id} 质量不达标: {quality:.2f} < {quality_threshold} "
                              f"(尝试 {attempt+1}, 规划器: {self.planner_type})")
                else:
                    print(f"❌ 路径 {path_id} 规划失败 (尝试 {attempt+1}, 规划器: {self.planner_type})")
            
            return False
            
        except Exception as e:
            print(f"生成路径 {path_id} 失败: {e}")
            return False
    
    def _build_path_indexes(self):
        """建立路径查找索引"""
        self.paths_to_target.clear()
        self.paths_from_source.clear()
        
        print("开始建立路径索引...")
        
        for path_id, path_data in self.backbone_paths.items():
            start_point = path_data['start_point']
            end_point = path_data['end_point']
            
            # 按终点建立索引
            target_key = (end_point['type'], end_point['id'])
            self.paths_to_target[target_key].append(path_data)
            
            # 按起点建立索引
            source_key = (start_point['type'], start_point['id'])
            self.paths_from_source[source_key].append(path_data)
            
            print(f"索引路径 {path_id}: {source_key} -> {target_key}")
        
        print(f"路径索引建立完成，目标索引: {len(self.paths_to_target)} 个")
    
    def find_paths_to_target(self, target_type, target_id):
        """查找到指定目标的所有骨干路径"""
        target_key = (target_type, target_id)
        return self.paths_to_target.get(target_key, [])
    
    def identify_target_point(self, target_position):
        """识别目标位置是否为特殊点"""
        tolerance = 2.0  # 位置容差
        
        # 检查是否为装载点
        for point in self.special_points['loading']:
            if self._calculate_distance(target_position, point['position']) < tolerance:
                return 'loading', point['id']
        
        # 检查是否为卸载点
        for point in self.special_points['unloading']:
            if self._calculate_distance(target_position, point['position']) < tolerance:
                return 'unloading', point['id']
        
        # 检查是否为停车点
        for point in self.special_points['parking']:
            if self._calculate_distance(target_position, point['position']) < tolerance:
                return 'parking', point['id']
        
        return None, None
    
    def _get_backbone_segment_from_interface(self, interface, target_type, target_id):
        """从接口获取到目标的骨干路径段"""
        backbone_path_data = self.backbone_paths.get(interface.backbone_path_id)
        if not backbone_path_data:
            return None
        
        backbone_path = backbone_path_data['path']
        
        # 从接口位置开始到路径终点的段
        if interface.path_index < len(backbone_path):
            return backbone_path[interface.path_index:]
        
        return None
    
    def _merge_paths(self, access_path, backbone_path):
        """合并接入路径和骨干路径"""
        if not access_path or not backbone_path:
            return None
        
        # 移除重复的连接点
        merged_path = list(access_path)
        
        # 如果接入路径的终点和骨干路径的起点很接近，跳过骨干路径的起点
        if (len(access_path) > 0 and len(backbone_path) > 0 and
            self._calculate_distance(access_path[-1], backbone_path[0]) < 1.0):
            merged_path.extend(backbone_path[1:])
        else:
            merged_path.extend(backbone_path)
        
        return merged_path
    
    def _evaluate_path_quality(self, path):
        """评估路径质量"""
        if not path or len(path) < 2:
            return 0.0
        
        # 简化的质量评估
        # 1. 长度效率
        path_length = self._calculate_path_length(path)
        direct_distance = self._calculate_distance(path[0], path[-1])
        
        if direct_distance < 0.1:
            length_efficiency = 1.0
        else:
            length_efficiency = min(1.0, direct_distance / path_length)
        
        # 2. 平滑度
        smoothness = self._evaluate_path_smoothness(path)
        
        # 综合评分
        quality = length_efficiency * 0.6 + smoothness * 0.4
        
        return quality
    
    def _evaluate_path_smoothness(self, path):
        """评估路径平滑度"""
        if len(path) < 3:
            return 1.0
        
        total_angle_change = 0.0
        for i in range(1, len(path) - 1):
            angle_change = self._calculate_angle_change(path[i-1], path[i], path[i+1])
            total_angle_change += angle_change
        
        # 归一化
        avg_angle_change = total_angle_change / max(1, len(path) - 2)
        smoothness = math.exp(-avg_angle_change * 2)
        
        return min(1.0, smoothness)
    
    def _calculate_angle_change(self, p1, p2, p3):
        """计算角度变化"""
        v1 = [p2[0] - p1[0], p2[1] - p1[1]]
        v2 = [p3[0] - p2[0], p3[1] - p2[1]]
        
        len1 = math.sqrt(v1[0]**2 + v1[1]**2)
        len2 = math.sqrt(v2[0]**2 + v2[1]**2)
        
        if len1 < 0.001 or len2 < 0.001:
            return 0.0
        
        cos_angle = (v1[0]*v2[0] + v1[1]*v2[1]) / (len1 * len2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        
        return math.acos(cos_angle)
    
    def _calculate_distance(self, pos1, pos2):
        """计算两点间距离"""
        x1 = pos1[0] if len(pos1) > 0 else 0
        y1 = pos1[1] if len(pos1) > 1 else 0
        x2 = pos2[0] if len(pos2) > 0 else 0
        y2 = pos2[1] if len(pos2) > 1 else 0
        
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def _calculate_path_length(self, path):
        """计算路径长度"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += self._calculate_distance(path[i], path[i+1])
        
        return length
    
    def _ensure_3d_point(self, point):
        """确保点坐标有三个元素"""
        if not point:
            return (0, 0, 0)
        if len(point) >= 3:
            return (point[0], point[1], point[2])
        elif len(point) == 2:
            return (point[0], point[1], 0)
        else:
            return (0, 0, 0)
    
    def release_interface(self, interface_id):
        """释放接口"""
        if interface_id in self.backbone_interfaces:
            self.backbone_interfaces[interface_id].release()
    
    def get_save_data(self) -> Dict:
        """获取保存数据"""
        return {
            'total_paths': len(self.backbone_paths),
            'total_interfaces': len(self.backbone_interfaces),
            'interface_spacing': self.interface_spacing,
            'planner_type': self.planner_type,  # 新增规划器类型
            'generation_stats': self.stats.copy(),
            'astar_integration': self.astar_integration.copy(),  # 新增混合A*集成状态
            'paths_summary': {
                path_id: {
                    'id': path_id,
                    'start_point_type': path_data["start_point"]["type"],
                    'start_point_id': path_data["start_point"]["id"],
                    'end_point_type': path_data["end_point"]["type"],
                    'end_point_id': path_data["end_point"]["id"],
                    'length': path_data.get("length", 0),
                    'quality': path_data.get("quality", 0),
                    'usage_count': path_data.get("usage_count", 0),
                    'path_points_count': len(path_data.get("path", [])),
                    'created_time': path_data.get("created_time", 0),
                    'planner_used': path_data.get("planner_used", "unknown")  # 新增规划器信息
                }
                for path_id, path_data in self.backbone_paths.items()
            },
            'interface_states': {
                interface_id: {
                    'interface_id': interface.interface_id,
                    'position': interface.position,
                    'direction': interface.direction,
                    'backbone_path_id': interface.backbone_path_id,
                    'path_index': interface.path_index,
                    'access_difficulty': interface.access_difficulty,
                    'usage_count': interface.usage_count,
                    'is_occupied': interface.is_occupied,
                    'occupied_by': interface.occupied_by,
                    'reservation_time': interface.reservation_time,
                    # 增强接口的额外属性
                    'accessibility_score': getattr(interface, 'accessibility_score', 0.5),
                    'astar_sampling_weight': getattr(interface, 'astar_sampling_weight', 1.0),
                    'usage_efficiency': getattr(interface, 'usage_efficiency', 0.0),
                    'influence_radius': getattr(interface, 'influence_radius', 15.0)
                }
                for interface_id, interface in self.backbone_interfaces.items()
            },
            'path_cache_stats': self.path_cache_stats.copy()
        }
    
    def restore_from_save_data(self, save_data: Dict):
        """从保存数据恢复状态"""
        try:
            # 恢复基本设置
            self.interface_spacing = save_data.get('interface_spacing', 10)
            
            # 恢复规划器类型
            saved_planner_type = save_data.get('planner_type', 'hybrid_astar')
            if saved_planner_type in self.available_planners:
                self.planner_type = saved_planner_type
            
            # 恢复统计信息
            if 'generation_stats' in save_data:
                self.stats.update(save_data['generation_stats'])
            
            # 恢复混合A*集成状态
            if 'astar_integration' in save_data:
                self.astar_integration.update(save_data['astar_integration'])
            
            # 恢复接口状态
            interface_states = save_data.get('interface_states', {})
            for interface_id, state_data in interface_states.items():
                if interface_id in self.backbone_interfaces:
                    interface = self.backbone_interfaces[interface_id]
                    
                    # 恢复基本状态
                    interface.usage_count = state_data.get('usage_count', 0)
                    interface.is_occupied = state_data.get('is_occupied', False)
                    interface.occupied_by = state_data.get('occupied_by')
                    interface.reservation_time = state_data.get('reservation_time')
                    
                    # 恢复增强属性
                    if hasattr(interface, 'accessibility_score'):
                        interface.accessibility_score = state_data.get('accessibility_score', 0.5)
                    if hasattr(interface, 'astar_sampling_weight'):
                        interface.astar_sampling_weight = state_data.get('astar_sampling_weight', 1.0)
                    if hasattr(interface, 'usage_efficiency'):
                        interface.usage_efficiency = state_data.get('usage_efficiency', 0.0)
                    if hasattr(interface, 'influence_radius'):
                        interface.influence_radius = state_data.get('influence_radius', 15.0)
            
            # 恢复路径缓存统计
            if 'path_cache_stats' in save_data:
                self.path_cache_stats.update(save_data['path_cache_stats'])
            
            print(f"骨干网络状态已恢复 (规划器: {self.planner_type})")
            
        except Exception as e:
            print(f"恢复骨干网络状态失败: {e}")
    
    def clear_all_data(self):
        """清除所有数据（重置时使用）"""
        self.backbone_paths.clear()
        self.backbone_interfaces.clear()
        self.path_interfaces.clear()
        self.interface_spatial_index.clear()
        self.paths_to_target.clear()
        self.paths_from_source.clear()
        self.sampling_regions.clear()
        self.path_quality_map.clear()
        
        # 重置统计信息
        self.stats = {
            'total_paths': 0,
            'total_interfaces': 0,
            'interface_usage': defaultdict(int),
            'generation_time': 0,
            'average_path_length': 0,
            'path_usage_count': defaultdict(int),
            'total_usage': 0,
            'planner_performance': defaultdict(dict)
        }
        
        self.path_cache_stats = {
            'total_requests': 0,
            'cache_hits': 0,
            'quality_improvements': 0
        }
        
        print("已清除所有骨干网络数据")
    
    def get_network_health_status(self) -> Dict:
        """获取网络健康状态"""
        total_interfaces = len(self.backbone_interfaces)
        available_interfaces = sum(
            1 for interface in self.backbone_interfaces.values() 
            if interface.is_available()
        )
        
        total_usage = sum(
            interface.usage_count for interface in self.backbone_interfaces.values()
        )
        
        return {
            'total_paths': len(self.backbone_paths),
            'total_interfaces': total_interfaces,
            'available_interfaces': available_interfaces,
            'interface_availability_rate': available_interfaces / max(1, total_interfaces),
            'total_usage': total_usage,
            'average_usage_per_interface': total_usage / max(1, total_interfaces),
            'network_utilization': min(1.0, total_usage / max(1, total_interfaces * 10)),
            'current_planner': self.planner_type,
            'planner_integration_active': self.planner_ref is not None,
            'cache_hit_rate': (
                self.path_cache_stats['cache_hits'] / 
                max(1, self.path_cache_stats['total_requests'])
            )
        }
    
    def debug_network_status(self):
        """调试网络状态"""
        print("=== 骨干网络调试信息 ===")
        print(f"骨干路径数量: {len(self.backbone_paths)}")
        print(f"当前规划器: {self.planner_type}")
        
        for path_id, path_data in self.backbone_paths.items():
            print(f"路径 {path_id}:")
            print(f"  起点: {path_data['start_point']['type']}_{path_data['start_point']['id']}")
            print(f"  终点: {path_data['end_point']['type']}_{path_data['end_point']['id']}")
            print(f"  路径长度: {len(path_data.get('path', []))} 个点")
            print(f"  接口数量: {len(self.path_interfaces.get(path_id, []))}")
            print(f"  规划器: {path_data.get('planner_used', 'unknown')}")
        
        print(f"\n接口总数: {len(self.backbone_interfaces)}")
        print(f"特殊点数量:")
        print(f"  装载点: {len(self.special_points.get('loading', []))}")
        print(f"  卸载点: {len(self.special_points.get('unloading', []))}")
        
        # 显示到目标的路径索引
        print(f"\n路径到目标索引:")
        for target_key, paths in self.paths_to_target.items():
            print(f"  {target_key}: {len(paths)} 条路径")
    
    # ===== 保持向后兼容性的属性和方法 =====
    
    @property
    def paths(self):
        """兼容原始接口"""
        return self.backbone_paths
    
    @property
    def connections(self):
        """兼容原始接口 - 返回空字典"""
        return {}
    
    # 兼容性方法
    def get_path_from_position_to_target(self, current_position, target_type, target_id):
        """兼容原有接口，内部调用新的接口系统"""
        return self.get_path_from_position_to_target_via_interface(
            current_position, target_type, target_id
        )
    
    # 兼容RRT命名的方法
    def get_sampling_guidance_for_rrt(self, start, goal):
        """兼容RRT接口的方法"""
        return self.get_sampling_guidance_for_planner(start, goal)
    
    def set_rrt_planner(self, planner):
        """兼容RRT接口的方法"""
        return self.set_planner(planner)
    
    def get_rrt_performance_stats(self):
        """兼容RRT接口的方法"""
        return self.get_planner_performance_stats()
    def register_vehicle_path_enhanced(self, vehicle_id: str, path: List, 
                                     path_structure: Dict = None, 
                                     start_time: float = 0, speed: float = 1.0,
                                     quality_score: float = 0.5) -> bool:
        """增强版车辆路径注册"""
        if not path or len(path) < 2:
            return False
        self.state_lock = threading.RLock()
        with self.state_lock:
            # 移除旧路径
            self.release_vehicle_path(vehicle_id)
            
            # 评估路径质量
            if quality_score <= 0:
                quality_score = self.quality_evaluator.evaluate_path(path)
            
            # 准备增强路径信息
            path_info = {
                'path': path,
                'structure': path_structure or {},
                'start_time': start_time,
                'speed': speed,
                'registered_time': time.time(),
                'quality_score': quality_score,
                'planner_used': path_structure.get('planner_used', 'unknown') if path_structure else 'unknown',
                'backbone_utilization': path_structure.get('backbone_utilization', 0.0) if path_structure else 0.0,
                'current_position': path[0],
                'target_position': path[-1],
                'progress': 0.0,
                'estimated_completion_time': start_time + len(path) * 2.0 / speed
            }
            
            # 处理接口预约
            self._handle_interface_reservations(vehicle_id, path_structure)
            
            # 注册路径
            self.active_paths[vehicle_id] = path_info
            
            # 添加到时空预留表
            self._add_to_spacetime_reservation(vehicle_id, path, start_time, speed)
            
            # 记录历史
            self.path_history[vehicle_id].append({
                'timestamp': time.time(),
                'path_length': len(path),
                'quality': quality_score,
                'planner': path_info['planner_used']
            })
            
            # 限制历史长度
            if len(self.path_history[vehicle_id]) > 20:
                self.path_history[vehicle_id].popleft()
            
            print(f"增强路径注册: 车辆{vehicle_id}, 长度{len(path)}, "
                  f"质量{quality_score:.2f}, 规划器{path_info['planner_used']}")
            
            return True
    def active_paths(self) -> List[str]:
        """获取当前活跃的骨干路径ID列表
        
        Returns:
            List[str]: 活跃的骨干路径ID列表，按使用频率排序
        """
        # 根据使用次数筛选活跃路径
        active_paths = []
        for path_id, usage_count in self.stats['path_usage_count'].items():
            if usage_count > 0:  # 只返回使用过的路径
                active_paths.append(path_id)
        
        # 按使用频率降序排序
        active_paths.sort(key=lambda x: self.stats['path_usage_count'][x], reverse=True)
        
        return active_paths
    def release_vehicle_path(self, vehicle_id: str) -> bool:
        """释放车辆路径"""
        with self.state_lock:
            if vehicle_id not in self.active_paths:
                return False
            
            # 释放接口预约
            with self.interface_lock:
                for interface_id, reservations in self.interface_reservations.items():
                    self.interface_reservations[interface_id] = [
                        res for res in reservations if res[0] != vehicle_id
                    ]
            
            # 移除时空预留
            expired_reservations = []
            for key, vehicles in self.path_reservations.items():
                if vehicle_id in vehicles:
                    vehicles.remove(vehicle_id)
                    if not vehicles:
                        expired_reservations.append(key)
            
            for key in expired_reservations:
                del self.path_reservations[key]
            
            # 移除活跃路径
            del self.active_paths[vehicle_id]
            
            return True
    
# 保持向后兼容性
OptimizedBackbonePathNetwork = SimplifiedBackbonePathNetwork
BackbonePathNetwork = SimplifiedBackbonePathNetwork