"""
main_debug.py - 增强调度系统调试版
简单的可视化界面，用于载入地图、设置骨干路径、一键启动观察全流程
"""

import os
import sys
import time
import json
import math
import threading
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import numpy as np
from typing import Dict, List, Tuple, Optional

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入增强组件
try:
    from test import EnhancedMineSchedulingSystem
    from vehicle_scheduler import TaskPriority
    from environment import OptimizedOpenPitMineEnv
    ENHANCED_AVAILABLE = True
    print("✅ 增强组件加载成功")
except ImportError as e:
    print(f"⚠️ 增强组件加载失败: {e}")
    ENHANCED_AVAILABLE = False

class VisualDebugger:
    """可视化调试器"""
    
    def __init__(self):
        # 系统组件
        self.env = None
        self.system = None
        
        # 可视化设置
        self.fig = None
        self.ax = None
        self.animation = None
        
        # 数据存储
        self.vehicle_plots = {}
        self.path_plots = {}
        self.backbone_plots = {}
        self.conflict_plots = {}
        
        # 性能监控
        self.performance_data = {
            'time': [],
            'efficiency': [],
            'throughput': [],
            'conflicts': [],
            'backbone_utilization': []
        }
        
        # 状态
        self.is_running = False
        self.update_thread = None
        
        # 配置
        self.config = {
            'update_interval': 0.5,  # 更新间隔（秒）
            'max_history': 100,      # 最大历史数据点
            'vehicle_size': 8,       # 车辆显示大小
            'path_width': 2,         # 路径线宽
            'backbone_width': 3      # 骨干路径线宽
        }
        
        print("🎯 可视化调试器初始化完成")
    
    def create_test_environment(self, width=200, height=200, num_vehicles=6):
        """创建测试环境"""
        print(f"🏗️ 创建测试环境 ({width}x{height}, {num_vehicles}车辆)")
        
        # 创建环境
        self.env = OptimizedOpenPitMineEnv()
        
        # 设置基本参数
        self.env.width = width
        self.env.height = height
        
        # 创建简单的障碍物网格
        self.env.grid = np.zeros((width, height), dtype=int)
        
        # 添加一些障碍物区域
        obstacles = [
            (60, 80, 40, 20),   # (x, y, width, height)
            (120, 60, 30, 40),
            (40, 140, 50, 30),
            (150, 120, 25, 35)
        ]
        
        for x, y, w, h in obstacles:
            self.env.grid[x:x+w, y:y+h] = 1
        
        # 设置特殊点
        self.env.loading_points = [
            (30, 30, 0),
            (170, 30, 0)
        ]
        
        self.env.unloading_points = [
            (30, 170, 0),
            (170, 170, 0)
        ]
        
        self.env.parking_areas = [
            (100, 100, 0)
        ]
        
        # 创建车辆
        self.env.vehicles = {}
        for i in range(num_vehicles):
            angle = 2 * math.pi * i / num_vehicles
            x = 100 + 30 * math.cos(angle)
            y = 100 + 30 * math.sin(angle)
            
            self.env.vehicles[f'vehicle_{i}'] = {
                'position': (x, y, angle),
                'max_load': 100,
                'load': 0,
                'status': 'idle',
                'completed_cycles': 0,
                'path': None,
                'progress': 0.0
            }
        
        # 存储环境到文件（可选）
        self.save_environment('debug_environment.json')
        
        print(f"✅ 测试环境创建完成: {len(self.env.loading_points)}装载点, "
              f"{len(self.env.unloading_points)}卸载点, {len(self.env.vehicles)}车辆")
        
        return True
    
    def load_environment_from_file(self, file_path):
        """从文件加载环境"""
        if not os.path.exists(file_path):
            print(f"❌ 环境文件不存在: {file_path}")
            return False
        
        try:
            self.env = OptimizedOpenPitMineEnv()
            if self.env.load_from_file(file_path):
                print(f"✅ 环境加载成功: {file_path}")
                return True
            else:
                print(f"❌ 环境加载失败: {file_path}")
                return False
        except Exception as e:
            print(f"❌ 加载环境时出错: {e}")
            return False
    
    def save_environment(self, file_path):
        """保存环境到文件"""
        try:
            env_data = {
                'width': self.env.width,
                'height': self.env.height,
                'loading_points': self.env.loading_points,
                'unloading_points': self.env.unloading_points,
                'parking_areas': getattr(self.env, 'parking_areas', []),
                'vehicles': dict(self.env.vehicles),
                'grid': self.env.grid.tolist() if hasattr(self.env, 'grid') else []
            }
            
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(env_data, f, indent=2, ensure_ascii=False)
            
            print(f"💾 环境已保存到: {file_path}")
            return True
            
        except Exception as e:
            print(f"❌ 保存环境失败: {e}")
            return False
    
    def initialize_system(self, quality_threshold=0.6):
        """初始化增强调度系统"""
        if not self.env:
            print("❌ 请先加载环境")
            return False
        
        if not ENHANCED_AVAILABLE:
            print("❌ 增强组件不可用")
            return False
        
        print("🚀 初始化增强调度系统...")
        
        try:
            # 创建系统
            self.system = EnhancedMineSchedulingSystem(self.env)
            
            # 初始化
            if not self.system.initialize_system(quality_threshold):
                print("❌ 系统初始化失败")
                return False
            
            print("✅ 增强调度系统初始化成功")
            return True
            
        except Exception as e:
            print(f"❌ 系统初始化错误: {e}")
            return False
    
    def setup_visualization(self):
        """设置可视化界面"""
        print("🎨 设置可视化界面...")
        
        # 创建图形窗口
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('增强调度系统实时监控', fontsize=16, fontweight='bold')
        
        # 创建子图
        gs = self.fig.add_gridspec(2, 3, height_ratios=[3, 1], width_ratios=[2, 1, 1])
        
        # 主地图视图
        self.ax = self.fig.add_subplot(gs[0, :2])
        self.ax.set_title('矿场实时状态', fontsize=14)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # 性能监控图
        self.perf_ax = self.fig.add_subplot(gs[0, 2])
        self.perf_ax.set_title('系统效率', fontsize=12)
        self.perf_ax.set_ylim(0, 1)
        
        # 统计信息区域
        self.stats_ax = self.fig.add_subplot(gs[1, :])
        self.stats_ax.axis('off')
        
        # 设置地图范围
        if self.env:
            self.ax.set_xlim(0, self.env.width)
            self.ax.set_ylim(0, self.env.height)
        
        # 绘制静态元素
        self._draw_static_elements()
        
        # 添加控制按钮
        self._add_control_buttons()
        
        print("✅ 可视化界面设置完成")
    
    def _draw_static_elements(self):
        """绘制静态元素"""
        if not self.env:
            return
        
        # 绘制障碍物
        if hasattr(self.env, 'grid'):
            obstacle_x, obstacle_y = np.where(self.env.grid == 1)
            self.ax.scatter(obstacle_x, obstacle_y, c='gray', s=4, alpha=0.6, marker='s')
        
        # 绘制装载点
        for i, point in enumerate(self.env.loading_points):
            circle = patches.Circle((point[0], point[1]), 8, 
                                  facecolor='green', alpha=0.3, edgecolor='darkgreen', linewidth=2)
            self.ax.add_patch(circle)
            self.ax.text(point[0], point[1]-15, f'装载{i+1}', 
                        ha='center', va='center', fontweight='bold', color='darkgreen')
        
        # 绘制卸载点
        for i, point in enumerate(self.env.unloading_points):
            square = patches.Rectangle((point[0]-8, point[1]-8), 16, 16,
                                     facecolor='orange', alpha=0.3, edgecolor='darkorange', linewidth=2)
            self.ax.add_patch(square)
            self.ax.text(point[0], point[1]-20, f'卸载{i+1}', 
                        ha='center', va='center', fontweight='bold', color='darkorange')
        
        # 绘制停车区
        if hasattr(self.env, 'parking_areas'):
            for i, point in enumerate(self.env.parking_areas):
                diamond = patches.RegularPolygon((point[0], point[1]), numvertices=4, radius=10,
                                            facecolor='blue', alpha=0.3, edgecolor='darkblue', linewidth=2)
                self.ax.add_patch(diamond)
                self.ax.text(point[0], point[1]-18, f'停车{i+1}', 
                            ha='center', va='center', fontweight='bold', color='darkblue')
    
    def _add_control_buttons(self):
        """添加控制按钮"""
        # 按钮位置
        button_y = 0.02
        button_height = 0.04
        button_width = 0.08
        
        # 启动/停止按钮
        start_ax = plt.axes([0.1, button_y, button_width, button_height])
        self.start_button = Button(start_ax, '启动系统')
        self.start_button.on_clicked(self._on_start_clicked)
        
        # 停止按钮
        stop_ax = plt.axes([0.2, button_y, button_width, button_height])
        self.stop_button = Button(stop_ax, '停止系统')
        self.stop_button.on_clicked(self._on_stop_clicked)
        
        # 分配任务按钮
        task_ax = plt.axes([0.3, button_y, button_width, button_height])
        self.task_button = Button(task_ax, '分配任务')
        self.task_button.on_clicked(self._on_assign_task_clicked)
        
        # 速度滑块
        speed_ax = plt.axes([0.5, button_y, 0.2, button_height])
        self.speed_slider = Slider(speed_ax, '速度', 0.1, 5.0, valinit=1.0)
        self.speed_slider.on_changed(self._on_speed_changed)
    
    def _on_start_clicked(self, event):
        """启动按钮点击事件"""
        if self.system and not self.is_running:
            if self.system.start_system():
                self.is_running = True
                self._start_update_thread()
                print("🚀 系统已启动")
    
    def _on_stop_clicked(self, event):
        """停止按钮点击事件"""
        if self.system and self.is_running:
            self.is_running = False
            self.system.stop_system()
            print("🛑 系统已停止")
    
    def _on_assign_task_clicked(self, event):
        """分配任务按钮点击事件"""
        if self.system and self.is_running:
            success = self.system.assign_priority_mission(priority=TaskPriority.HIGH)
            if success:
                print("✅ 高优先级任务已分配")
            else:
                print("❌ 任务分配失败")
    
    def _on_speed_changed(self, val):
        """速度滑块变化事件"""
        self.config['update_interval'] = 1.0 / val
        print(f"⚡ 更新速度调整为: {val:.1f}x")
    
    def _start_update_thread(self):
        """启动更新线程"""
        def update_loop():
            last_update = time.time()
            
            while self.is_running:
                current_time = time.time()
                delta_time = current_time - last_update
                
                if delta_time >= self.config['update_interval']:
                    # 更新系统
                    if self.system:
                        self.system.update_system(delta_time)
                    
                    # 收集性能数据
                    self._collect_performance_data()
                    
                    last_update = current_time
                
                time.sleep(0.1)  # 短暂休眠
        
        self.update_thread = threading.Thread(target=update_loop, daemon=True)
        self.update_thread.start()
    
    def _collect_performance_data(self):
        """收集性能数据"""
        if not self.system:
            return
        
        try:
            status = self.system.get_system_status()
            current_time = time.time()
            
            # 添加数据点
            self.performance_data['time'].append(current_time)
            self.performance_data['efficiency'].append(
                status['system_metrics']['overall_efficiency']
            )
            self.performance_data['throughput'].append(
                status['system_metrics']['system_throughput']
            )
            self.performance_data['backbone_utilization'].append(
                status['system_metrics']['backbone_utilization']
            )
            
            # 获取冲突数量
            if hasattr(self.system, 'traffic_manager'):
                traffic_stats = self.system.traffic_manager.get_statistics()
                conflict_count = traffic_stats.get('total_conflicts', 0)
                self.performance_data['conflicts'].append(conflict_count)
            else:
                self.performance_data['conflicts'].append(0)
            
            # 限制数据长度
            max_len = self.config['max_history']
            for key in self.performance_data:
                if len(self.performance_data[key]) > max_len:
                    self.performance_data[key] = self.performance_data[key][-max_len:]
        
        except Exception as e:
            print(f"性能数据收集错误: {e}")
    
    def update_visualization(self, frame):
        """更新可视化（动画回调）"""
        try:
            # 清除动态元素
            self._clear_dynamic_elements()
            
            # 绘制骨干网络
            self._draw_backbone_network()
            
            # 绘制车辆和路径
            self._draw_vehicles_and_paths()
            
            # 绘制冲突
            self._draw_conflicts()
            
            # 更新性能图表
            self._update_performance_plot()
            
            # 更新统计信息
            self._update_statistics_display()
        
        except Exception as e:
            print(f"可视化更新错误: {e}")
    
    def _clear_dynamic_elements(self):
        """清除动态元素"""
        # 清除车辆
        for plots in self.vehicle_plots.values():
            for plot in plots:
                if plot in self.ax.collections or plot in self.ax.patches:
                    plot.remove()
        self.vehicle_plots.clear()
        
        # 清除路径
        for plot in self.path_plots.values():
            if hasattr(plot, 'remove'):
                plot.remove()
        self.path_plots.clear()
        
        # 清除冲突
        for plot in self.conflict_plots.values():
            if hasattr(plot, 'remove'):
                plot.remove()
        self.conflict_plots.clear()
    
    def _draw_backbone_network(self):
        """绘制骨干网络"""
        if not self.system or not hasattr(self.system, 'backbone_network'):
            return
        
        backbone = self.system.backbone_network
        if not backbone or not hasattr(backbone, 'bidirectional_paths'):
            return
        
        for path_id, path_data in backbone.bidirectional_paths.items():
            if not path_data.forward_path or len(path_data.forward_path) < 2:
                continue
            
            # 提取坐标
            x_coords = [p[0] for p in path_data.forward_path]
            y_coords = [p[1] for p in path_data.forward_path]
            
            # 根据负载设置颜色
            load_factor = path_data.get_load_factor()
            if load_factor > 0.8:
                color = 'red'
                alpha = 0.8
            elif load_factor > 0.5:
                color = 'orange'
                alpha = 0.6
            else:
                color = 'blue'
                alpha = 0.4
            
            # 绘制路径
            line = self.ax.plot(x_coords, y_coords, color=color, alpha=alpha,
                               linewidth=self.config['backbone_width'], 
                               linestyle='-', label=f'骨干路径 {path_id}')[0]
            
            self.backbone_plots[path_id] = line
            
            # 绘制接口点
            if hasattr(backbone, 'backbone_interfaces'):
                interface_spacing = backbone.config.get('interface_spacing', 8)
                for i in range(0, len(path_data.forward_path), interface_spacing):
                    if i < len(path_data.forward_path):
                        point = path_data.forward_path[i]
                        circle = patches.Circle((point[0], point[1]), 2, 
                                              facecolor='cyan', alpha=0.6,
                                              edgecolor='darkblue', linewidth=1)
                        self.ax.add_patch(circle)
    
    def _draw_vehicles_and_paths(self):
        """绘制车辆和路径"""
        if not self.env:
            return
        
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            position = vehicle_data.get('position', (0, 0, 0))
            status = vehicle_data.get('status', 'idle')
            load = vehicle_data.get('load', 0)
            max_load = vehicle_data.get('max_load', 100)
            
            # 车辆颜色
            status_colors = {
                'idle': 'gray',
                'moving': 'blue',
                'loading': 'green',
                'unloading': 'orange',
                'planning': 'purple',
                'waiting': 'red'
            }
            color = status_colors.get(status, 'gray')
            
            # 绘制车辆
            vehicle_circle = patches.Circle((position[0], position[1]), 
                                          self.config['vehicle_size']/2,
                                          facecolor=color, alpha=0.7,
                                          edgecolor='black', linewidth=1)
            self.ax.add_patch(vehicle_circle)
            
            # 绘制负载指示器
            if max_load > 0:
                load_ratio = load / max_load
                load_height = self.config['vehicle_size'] * load_ratio
                load_rect = patches.Rectangle(
                    (position[0] - 2, position[1] - self.config['vehicle_size']/2),
                    4, load_height,
                    facecolor='yellow', alpha=0.8,
                    edgecolor='black', linewidth=1
                )
                self.ax.add_patch(load_rect)
            
            # 绘制方向指示
            if len(position) >= 3:
                theta = position[2]
                arrow_length = self.config['vehicle_size']
                dx = arrow_length * math.cos(theta)
                dy = arrow_length * math.sin(theta)
                
                arrow = patches.FancyArrowPatch(
                    (position[0], position[1]),
                    (position[0] + dx, position[1] + dy),
                    arrowstyle='->', mutation_scale=15,
                    color='black', alpha=0.8, linewidth=2
                )
                self.ax.add_patch(arrow)
            
            # 绘制车辆标签
            self.ax.text(position[0], position[1] + self.config['vehicle_size'],
                        f'{vehicle_id}\n{status}',
                        ha='center', va='bottom', fontsize=8,
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))
            
            # 绘制车辆路径
            vehicle_path = vehicle_data.get('path')
            if vehicle_path and len(vehicle_path) > 1:
                path_x = [p[0] for p in vehicle_path]
                path_y = [p[1] for p in vehicle_path]
                
                line = self.ax.plot(path_x, path_y, color=color, alpha=0.5,
                                   linewidth=self.config['path_width'], 
                                   linestyle='--')[0]
                
                self.path_plots[vehicle_id] = line
            
            # 存储车辆图形元素
            self.vehicle_plots[vehicle_id] = [vehicle_circle, load_rect if max_load > 0 else None]
    
    def _draw_conflicts(self):
        """绘制冲突"""
        if not self.system or not hasattr(self.system, 'traffic_manager'):
            return
        
        try:
            conflicts = self.system.traffic_manager.detect_all_conflicts()
            
            for i, conflict in enumerate(conflicts[:10]):  # 最多显示10个冲突
                x, y = conflict.location
                
                # 冲突严重程度决定颜色
                if conflict.severity > 0.8:
                    color = 'red'
                    size = 15
                elif conflict.severity > 0.5:
                    color = 'orange'
                    size = 12
                else:
                    color = 'yellow'
                    size = 10
                
                # 绘制冲突标记
                conflict_marker = patches.RegularPolygon((x, y), 6, size,
                                                       facecolor=color, alpha=0.7,
                                                       edgecolor='darkred', linewidth=2)
                self.ax.add_patch(conflict_marker)
                
                # 冲突标签
                self.ax.text(x, y + 20, f'冲突{i+1}\n{conflict.conflict_type.value}',
                            ha='center', va='bottom', fontsize=8,
                            bbox=dict(boxstyle='round,pad=0.2', facecolor=color, alpha=0.8))
                
                self.conflict_plots[f'conflict_{i}'] = conflict_marker
        
        except Exception as e:
            print(f"冲突绘制错误: {e}")
    
    def _update_performance_plot(self):
        """更新性能图表"""
        if not self.performance_data['time']:
            return
        
        try:
            self.perf_ax.clear()
            self.perf_ax.set_title('系统效率', fontsize=12)
            self.perf_ax.set_ylim(0, 1)
            
            # 获取时间数据（相对时间）
            if self.performance_data['time']:
                start_time = self.performance_data['time'][0]
                relative_times = [(t - start_time) / 60 for t in self.performance_data['time']]  # 转换为分钟
                
                # 绘制效率曲线
                if self.performance_data['efficiency']:
                    self.perf_ax.plot(relative_times, self.performance_data['efficiency'],
                                     'b-', linewidth=2, label='效率')
                
                # 绘制骨干利用率
                if self.performance_data['backbone_utilization']:
                    self.perf_ax.plot(relative_times, self.performance_data['backbone_utilization'],
                                     'g-', linewidth=2, label='骨干利用率')
                
                self.perf_ax.set_xlabel('时间 (分钟)')
                self.perf_ax.set_ylabel('比率')
                self.perf_ax.legend(loc='upper right')
                self.perf_ax.grid(True, alpha=0.3)
        
        except Exception as e:
            print(f"性能图表更新错误: {e}")
    
    def _update_statistics_display(self):
        """更新统计信息显示"""
        if not self.system:
            return
        
        try:
            self.stats_ax.clear()
            self.stats_ax.axis('off')
            
            # 获取系统状态
            status = self.system.get_system_status()
            
            # 构建统计文本
            stats_text = []
            
            # 基本信息
            runtime = status.get('runtime', 0)
            stats_text.append(f"运行时间: {runtime/60:.1f} 分钟")
            
            # 系统指标
            metrics = status.get('system_metrics', {})
            stats_text.append(f"系统效率: {metrics.get('overall_efficiency', 0):.1%}")
            stats_text.append(f"骨干利用率: {metrics.get('backbone_utilization', 0):.1%}")
            stats_text.append(f"吞吐量: {metrics.get('system_throughput', 0):.1f} 任务/小时")
            stats_text.append(f"车辆空闲率: {metrics.get('vehicle_idle_ratio', 0):.1%}")
            
            # 组件统计
            component_stats = status.get('component_stats', {})
            
            # 调度器统计
            scheduler_stats = component_stats.get('scheduler', {})
            real_time = scheduler_stats.get('real_time', {})
            stats_text.append(f"活跃车辆: {real_time.get('active_vehicles', 0)}")
            stats_text.append(f"完成任务: {scheduler_stats.get('completed_tasks', 0)}")
            stats_text.append(f"失败任务: {scheduler_stats.get('failed_tasks', 0)}")
            
            # 交通统计
            traffic_stats = component_stats.get('traffic', {})
            stats_text.append(f"已解决冲突: {traffic_stats.get('resolved_conflicts', 0)}")
            stats_text.append(f"停车操作: {traffic_stats.get('parking_maneuvers', 0)}")
            
            # 骨干网络统计
            backbone_stats = component_stats.get('backbone', {})
            stats_text.append(f"骨干路径: {backbone_stats.get('bidirectional_paths', 0)} 条")
            
            # 显示统计文本
            text_content = '\n'.join(stats_text)
            self.stats_ax.text(0.02, 0.98, text_content, transform=self.stats_ax.transAxes,
                              fontsize=10, verticalalignment='top',
                              bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.8))
            
            # 添加性能历史摘要
            if self.performance_data['efficiency']:
                recent_efficiency = self.performance_data['efficiency'][-10:]
                avg_efficiency = sum(recent_efficiency) / len(recent_efficiency)
                
                summary_text = f"近期平均效率: {avg_efficiency:.1%}"
                if len(self.performance_data['efficiency']) > 1:
                    trend = "↗️" if self.performance_data['efficiency'][-1] > self.performance_data['efficiency'][-2] else "↘️"
                    summary_text += f" {trend}"
                
                self.stats_ax.text(0.6, 0.98, summary_text, transform=self.stats_ax.transAxes,
                                  fontsize=12, verticalalignment='top', fontweight='bold',
                                  bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
        
        except Exception as e:
            print(f"统计显示更新错误: {e}")
    
    def start_visualization(self):
        """启动可视化"""
        print("🎬 启动实时可视化...")
        
        # 设置动画
        self.animation = FuncAnimation(self.fig, self.update_visualization,
                                     interval=500, blit=False, cache_frame_data=False)
        
        # 显示窗口
        plt.tight_layout()
        plt.show()
    
    def run_complete_demo(self):
        """运行完整演示"""
        print("🎯 开始完整系统演示...")
        
        try:
            # 1. 创建或加载环境
            print("\n📋 第1步: 环境准备")
            env_file = 'debug_environment.json'
            
            if os.path.exists(env_file):
                print(f"发现现有环境文件: {env_file}")
                choice = input("使用现有环境文件? (y/n): ").strip().lower()
                if choice == 'y':
                    if not self.load_environment_from_file(env_file):
                        print("加载失败，创建新环境...")
                        self.create_test_environment()
                else:
                    self.create_test_environment()
            else:
                self.create_test_environment()
            
            # 2. 初始化系统
            print("\n🚀 第2步: 系统初始化")
            quality = float(input("输入骨干网络质量阈值 (0.1-1.0, 默认0.6): ") or "0.6")
            
            if not self.initialize_system(quality):
                print("❌ 系统初始化失败，退出演示")
                return False
            
            # 3. 设置可视化
            print("\n🎨 第3步: 可视化设置")
            self.setup_visualization()
            
            # 4. 启动可视化
            print("\n🎬 第4步: 启动实时监控")
            print("=" * 50)
            print("📖 操作指南:")
            print("  - 点击 '启动系统' 开始调度")
            print("  - 点击 '分配任务' 添加高优先级任务")
            print("  - 使用速度滑块调整更新频率")
            print("  - 关闭窗口结束演示")
            print("=" * 50)
            
            self.start_visualization()
            
            return True
            
        except KeyboardInterrupt:
            print("\n🛑 用户中断演示")
            return True
        except Exception as e:
            print(f"\n❌ 演示过程中发生错误: {e}")
            return False
        finally:
            # 清理
            if self.system and self.is_running:
                self.is_running = False
                self.system.stop_system()
            print("🎯 演示结束")

def main():
    """主函数"""
    print("🌟 增强调度系统可视化调试器")
    print("=" * 50)
    
    if not ENHANCED_AVAILABLE:
        print("❌ 增强组件不可用，请检查导入路径")
        return
    
    # 创建调试器
    debugger = VisualDebugger()
    
    # 运行演示
    debugger.run_complete_demo()

if __name__ == "__main__":
    main()