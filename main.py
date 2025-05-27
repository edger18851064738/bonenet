#!/usr/bin/env python3
"""
main.py - 优化版露天矿多车协同调度系统
优化可视化：简洁美观，功能完整
"""

import sys
import os
import math
import time
import json
from typing import Dict, List, Tuple, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPropertyAnimation, QEasingCurve
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QGroupBox, QGridLayout, QTableWidget, QTableWidgetItem,
    QGraphicsScene, QGraphicsView, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsPathItem, QTabWidget, QFrame,
    QSlider, QCheckBox, QLCDNumber, QScrollArea
)
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter, QPainterPath, QFont, QPixmap, QIcon

# 导入优化后的组件
try:
    from optimized_backbone_network import OptimizedBackboneNetwork
    from optimized_planner_config import EnhancedPathPlannerWithConfig
    OPTIMIZED_COMPONENTS_AVAILABLE = True
    print("✅ 优化组件加载成功")
except ImportError:
    print("⚠️ 优化组件不可用，使用原始组件")
    from path_planner import SimplifiedPathPlanner as EnhancedPathPlannerWithConfig
    OPTIMIZED_COMPONENTS_AVAILABLE = False

from traffic_manager import OptimizedTrafficManager
from vehicle_scheduler import SimplifiedVehicleScheduler
from environment import OptimizedOpenPitMineEnv


class MinimalMapView(QGraphicsView):
    """简洁美观的地图视图 - 优化视觉效果"""
    
    def __init__(self):
        super().__init__()
        
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        
        # 数据引用
        self.env = None
        self.backbone_network = None
        
        # 显示选项
        self.show_backbone = True
        self.show_paths = True
        self.show_interfaces = False
        self.show_grid = False
        
        # 图形项存储
        self.vehicle_items = {}
        self.path_items = {}
        self.backbone_items = []
        self.interface_items = []
        
        # 优化的颜色方案
        self.colors = {
            'background': QColor(248, 249, 250),
            'obstacle': QColor(108, 117, 125),
            'obstacle_border': QColor(73, 80, 87),
            'loading': QColor(40, 167, 69),
            'unloading': QColor(220, 53, 69),
            'parking': QColor(0, 123, 255),
            'vehicle': [
                QColor(220, 53, 69),   # 红
                QColor(40, 167, 69),   # 绿  
                QColor(0, 123, 255),   # 蓝
                QColor(108, 117, 125), # 灰
                QColor(255, 193, 7),   # 黄
                QColor(111, 66, 193),  # 紫
            ],
            'backbone_high': QColor(40, 167, 69),    # 高质量
            'backbone_mid': QColor(0, 123, 255),     # 中等质量  
            'backbone_low': QColor(255, 193, 7),     # 低质量
            'interface': QColor(111, 66, 193),
            'text': QColor(73, 80, 87)
        }
        
        # 视觉增强
        self.setStyleSheet("""
            QGraphicsView {
                border: 1px solid #dee2e6;
                border-radius: 4px;
                background-color: #f8f9fa;
            }
        """)
    
    def set_environment(self, env):
        """设置环境"""
        self.env = env
        if env:
            self.scene.setSceneRect(0, 0, env.width, env.height)
        self.redraw_environment()
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.redraw_environment()
    
    def toggle_backbone_display(self, show: bool):
        """切换骨干网络显示"""
        self.show_backbone = show
        self.redraw_environment()
    
    def toggle_interfaces_display(self, show: bool):
        """切换接口显示"""
        self.show_interfaces = show
        self.redraw_environment()
    
    def redraw_environment(self):
        """重绘环境 - 简洁美观的设计"""
        self.scene.clear()
        self.vehicle_items.clear()
        self.path_items.clear()
        self.backbone_items.clear()
        self.interface_items.clear()
        
        if not self.env:
            return
        
        # 绘制背景
        bg = QGraphicsRectItem(0, 0, self.env.width, self.env.height)
        bg.setBrush(QBrush(self.colors['background']))
        bg.setPen(QPen(Qt.NoPen))
        self.scene.addItem(bg)
        
        # 绘制网格（如果启用）
        if self.show_grid:
            self._draw_minimal_grid()
        
        # 绘制障碍物 - 简洁设计
        self._draw_minimal_obstacles()
        
        # 绘制关键点 - 小尺寸设计
        self._draw_minimal_special_points()
        
        # 绘制骨干网络
        if self.show_backbone and self.backbone_network:
            self._draw_minimal_backbone_network()
        
        # 绘制车辆
        self._draw_minimal_vehicles()
        
        self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
    
    def _draw_minimal_grid(self):
        """绘制简洁网格"""
        grid_size = 50  # 增大网格间距
        pen = QPen(QColor(0, 0, 0, 15))  # 极淡的网格线
        pen.setWidth(0)  # 最细线条
        
        # 垂直线
        for x in range(0, int(self.env.width), grid_size):
            self.scene.addLine(x, 0, x, self.env.height, pen)
        
        # 水平线
        for y in range(0, int(self.env.height), grid_size):
            self.scene.addLine(0, y, self.env.width, y, pen)
    
    def _draw_minimal_obstacles(self):
        """绘制简洁障碍物"""
        for x, y in self.env.obstacle_points:
            # 更小的障碍物尺寸
            obstacle = QGraphicsRectItem(x-0.3, y-0.3, 0.6, 0.6)
            obstacle.setBrush(QBrush(self.colors['obstacle']))
            obstacle.setPen(QPen(self.colors['obstacle_border'], 0.2))
            self.scene.addItem(obstacle)
    
    def _draw_minimal_special_points(self):
        """绘制小尺寸的关键点"""
        # 装载点 - 小圆形
        for i, point in enumerate(self.env.loading_points):
            item = QGraphicsEllipseItem(point[0]-3, point[1]-3, 6, 6)
            item.setBrush(QBrush(self.colors['loading']))
            item.setPen(QPen(self.colors['loading'].darker(130), 1))
            self.scene.addItem(item)
            
            # 小标签
            text = self.scene.addText(f"L{i+1}", QFont("Arial", 7))
            text.setPos(point[0]+4, point[1]-8)
            text.setDefaultTextColor(self.colors['text'])
        
        # 卸载点 - 小方形
        for i, point in enumerate(self.env.unloading_points):
            item = QGraphicsRectItem(point[0]-3, point[1]-3, 6, 6)
            item.setBrush(QBrush(self.colors['unloading']))
            item.setPen(QPen(self.colors['unloading'].darker(130), 1))
            self.scene.addItem(item)
            
            text = self.scene.addText(f"U{i+1}", QFont("Arial", 7))
            text.setPos(point[0]+4, point[1]-8)
            text.setDefaultTextColor(self.colors['text'])
        
        # 停车点 - 小菱形
        for i, point in enumerate(self.env.parking_areas):
            item = self._create_small_diamond(point[0], point[1], 3, self.colors['parking'])
            self.scene.addItem(item)
            
            text = self.scene.addText(f"P{i+1}", QFont("Arial", 7))
            text.setPos(point[0]+4, point[1]-8)
            text.setDefaultTextColor(self.colors['text'])
    
    def _create_small_diamond(self, x, y, size, color):
        """创建小菱形"""
        path = QPainterPath()
        path.moveTo(x, y - size)
        path.lineTo(x + size, y)
        path.lineTo(x, y + size)
        path.lineTo(x - size, y)
        path.closeSubpath()
        
        item = QGraphicsPathItem(path)
        item.setBrush(QBrush(color))
        item.setPen(QPen(color.darker(130), 1))
        return item
    
    def _draw_minimal_backbone_network(self):
        """绘制简洁的骨干网络"""
        if not hasattr(self.backbone_network, 'bidirectional_paths'):
            # 兼容原版本
            if hasattr(self.backbone_network, 'backbone_paths'):
                self._draw_legacy_backbone()
            return
        
        # 绘制优化版双向路径
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            forward_path = path_data.forward_path
            if len(forward_path) < 2:
                continue
            
            # 主路径
            painter_path = QPainterPath()
            painter_path.moveTo(forward_path[0][0], forward_path[0][1])
            
            for point in forward_path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            path_item = QGraphicsPathItem(painter_path)
            
            # 根据质量设置颜色 - 更细的线条
            quality = path_data.quality
            if quality > 0.8:
                color = self.colors['backbone_high']
            elif quality > 0.6:
                color = self.colors['backbone_mid']
            else:
                color = self.colors['backbone_low']
            
            pen = QPen(color, 0.8)  # 更细的线条
            pen.setCapStyle(Qt.RoundCap)
            path_item.setPen(pen)
            self.scene.addItem(path_item)
            self.backbone_items.append(path_item)
            
            # 绘制接口点（如果启用）
            if self.show_interfaces:
                self._draw_minimal_interfaces(path_data)
    
    def _draw_legacy_backbone(self):
        """绘制传统骨干网络"""
        for path_id, path_data in self.backbone_network.backbone_paths.items():
            path = path_data.get('path', [])
            if len(path) < 2:
                continue
            
            painter_path = QPainterPath()
            painter_path.moveTo(path[0][0], path[0][1])
            
            for point in path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            path_item = QGraphicsPathItem(painter_path)
            pen = QPen(self.colors['backbone_mid'], 0.8)
            path_item.setPen(pen)
            self.scene.addItem(path_item)
            self.backbone_items.append(path_item)
    
    def _draw_minimal_interfaces(self, path_data):
        """绘制小尺寸接口"""
        forward_path = path_data.forward_path
        spacing = 12  # 增大接口间距，减少视觉混乱
        
        for i in range(0, len(forward_path), spacing):
            if i >= len(forward_path):
                break
            
            point = forward_path[i]
            interface_item = QGraphicsEllipseItem(point[0]-1, point[1]-1, 2, 2)
            interface_item.setBrush(QBrush(self.colors['interface']))
            interface_item.setPen(QPen(Qt.NoPen))
            self.scene.addItem(interface_item)
            self.interface_items.append(interface_item)
    
    def _draw_minimal_vehicles(self):
        """绘制简洁的车辆"""
        if not self.env or not self.env.vehicles:
            return
        
        for i, (vehicle_id, vehicle_data) in enumerate(self.env.vehicles.items()):
            pos = vehicle_data.get('position', (0, 0, 0))
            status = vehicle_data.get('status', 'idle')
            
            # 选择颜色
            color = self.colors['vehicle'][i % len(self.colors['vehicle'])]
            
            # 车辆主体 - 更小的尺寸
            vehicle_item = QGraphicsEllipseItem(pos[0]-2.5, pos[1]-2.5, 5, 5)
            vehicle_item.setBrush(QBrush(color))
            vehicle_item.setPen(QPen(color.darker(130), 1))
            self.scene.addItem(vehicle_item)
            self.vehicle_items[vehicle_id] = vehicle_item
            
            # 方向指示器 - 更短更细
            if len(pos) > 2:
                theta = pos[2]
                dx = 4 * math.cos(theta)
                dy = 4 * math.sin(theta)
                
                arrow_line = self.scene.addLine(
                    pos[0], pos[1], pos[0] + dx, pos[1] + dy,
                    QPen(color.darker(150), 1.5, Qt.SolidLine, Qt.RoundCap)
                )
            
            # 状态指示 - 更小的圆环
            status_colors = {
                'idle': QColor(108, 117, 125),
                'moving': QColor(40, 167, 69),
                'loading': QColor(255, 193, 7),
                'unloading': QColor(220, 53, 69),
                'planning': QColor(111, 66, 193)
            }
            
            status_color = status_colors.get(status, QColor(108, 117, 125))
            status_ring = QGraphicsEllipseItem(pos[0]-3.5, pos[1]-3.5, 7, 7)
            status_ring.setBrush(QBrush(Qt.NoBrush))
            status_ring.setPen(QPen(status_color, 1))
            self.scene.addItem(status_ring)
            
            # 小标签
            text = self.scene.addText(str(vehicle_id), QFont("Arial", 6, QFont.Bold))
            text.setPos(pos[0]+4, pos[1]-10)
            text.setDefaultTextColor(self.colors['text'])
            
            # 绘制路径
            if self.show_paths and 'path' in vehicle_data and vehicle_data['path']:
                self._draw_minimal_vehicle_path(vehicle_id, vehicle_data['path'], color)
    
    def _draw_minimal_vehicle_path(self, vehicle_id, path, color):
        """绘制简洁的车辆路径"""
        if len(path) < 2:
            return
        
        painter_path = QPainterPath()
        painter_path.moveTo(path[0][0], path[0][1])
        
        for point in path[1:]:
            painter_path.lineTo(point[0], point[1])
        
        path_item = QGraphicsPathItem(painter_path)
        pen = QPen(color, 0.8)  # 很细的路径线
        pen.setStyle(Qt.DashLine)
        pen.setCapStyle(Qt.RoundCap)
        path_item.setPen(pen)
        self.scene.addItem(path_item)
        self.path_items[vehicle_id] = path_item
    
    def update_vehicles(self):
        """更新车辆显示"""
        if not self.env:
            return
        
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            if vehicle_id in self.vehicle_items:
                pos = vehicle_data.get('position', (0, 0, 0))
                item = self.vehicle_items[vehicle_id]
                item.setRect(pos[0]-2.5, pos[1]-2.5, 5, 5)
    
    def wheelEvent(self, event):
        """鼠标滚轮缩放"""
        factor = 1.1 if event.angleDelta().y() > 0 else 1/1.1
        self.scale(factor, factor)


class CompactPerformanceMonitor(QWidget):
    """紧凑的性能监控组件"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        
        # 数据存储
        self.backbone_stats = {}
        self.planner_stats = {}
        self.traffic_stats = {}
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 骨干网络性能 - 紧凑布局
        backbone_group = QGroupBox("🌐 骨干网络")
        backbone_layout = QGridLayout()
        backbone_layout.setSpacing(4)
        
        # 使用小尺寸LCD
        self.backbone_paths_lcd = QLCDNumber(2)
        self.backbone_paths_lcd.setMaximumHeight(30)
        self.backbone_paths_lcd.setStyleSheet("QLCDNumber { color: #28a745; }")
        
        self.backbone_success_rate = QLabel("0%")
        self.backbone_avg_quality = QLabel("0.00")
        self.backbone_gen_time = QLabel("0.00s")
        
        # 紧凑的字体
        small_font = QFont()
        small_font.setPointSize(8)
        for widget in [self.backbone_success_rate, self.backbone_avg_quality, self.backbone_gen_time]:
            widget.setFont(small_font)
        
        backbone_layout.addWidget(QLabel("路径:"), 0, 0)
        backbone_layout.addWidget(self.backbone_paths_lcd, 0, 1)
        backbone_layout.addWidget(QLabel("成功率:"), 1, 0)
        backbone_layout.addWidget(self.backbone_success_rate, 1, 1)
        backbone_layout.addWidget(QLabel("质量:"), 2, 0)
        backbone_layout.addWidget(self.backbone_avg_quality, 2, 1)
        backbone_layout.addWidget(QLabel("时间:"), 3, 0)
        backbone_layout.addWidget(self.backbone_gen_time, 3, 1)
        
        backbone_group.setLayout(backbone_layout)
        layout.addWidget(backbone_group)
        
        # 交通管理性能 - 紧凑版
        traffic_group = QGroupBox("🚦 交通状态")
        traffic_layout = QGridLayout()
        traffic_layout.setSpacing(4)
        
        self.active_vehicles_lcd = QLCDNumber(2)
        self.active_vehicles_lcd.setMaximumHeight(25)
        self.active_vehicles_lcd.setStyleSheet("QLCDNumber { color: #007bff; }")
        
        self.conflicts_lcd = QLCDNumber(2)
        self.conflicts_lcd.setMaximumHeight(25)
        self.conflicts_lcd.setStyleSheet("QLCDNumber { color: #dc3545; }")
        
        traffic_layout.addWidget(QLabel("活跃:"), 0, 0)
        traffic_layout.addWidget(self.active_vehicles_lcd, 0, 1)
        traffic_layout.addWidget(QLabel("冲突:"), 1, 0)
        traffic_layout.addWidget(self.conflicts_lcd, 1, 1)
        
        traffic_group.setLayout(traffic_layout)
        layout.addWidget(traffic_group)
        
        layout.addStretch()
    
    def update_backbone_stats(self, stats):
        """更新骨干网络统计"""
        self.backbone_stats = stats
        
        if 'bidirectional_paths' in stats:
            self.backbone_paths_lcd.display(stats['bidirectional_paths'])
        elif 'total_paths' in stats:
            self.backbone_paths_lcd.display(stats['total_paths'])
        
        if 'generation_stats' in stats:
            gen_stats = stats['generation_stats']
            
            # 计算成功率
            if 'total_path_pairs' in gen_stats and gen_stats['total_path_pairs'] > 0:
                success_rate = gen_stats['successful_paths'] / gen_stats['total_path_pairs']
                self.backbone_success_rate.setText(f"{success_rate:.0%}")
            
            # 生成时间
            if 'generation_time' in gen_stats:
                self.backbone_gen_time.setText(f"{gen_stats['generation_time']:.1f}s")
    
    def update_traffic_stats(self, active_vehicles, conflicts, resolved):
        """更新交通统计"""
        self.active_vehicles_lcd.display(active_vehicles)
        self.conflicts_lcd.display(conflicts)


class CompactControlPanel(QWidget):
    """紧凑的控制面板"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 文件操作 - 紧凑版
        file_group = QGroupBox("📁 文件")
        file_layout = QVBoxLayout()
        file_layout.setSpacing(4)
        
        self.file_label = QLabel("未选择文件")
        self.file_label.setStyleSheet("QLabel { color: #6c757d; font-size: 10px; }")
        
        # 紧凑的按钮
        self.browse_btn = QPushButton("📂 选择地图")
        self.load_btn = QPushButton("📥 加载")
        
        for btn in [self.browse_btn, self.load_btn]:
            btn.setMaximumHeight(30)
            self._compact_button_style(btn)
        
        self.browse_btn.clicked.connect(self.main_window.browse_file)
        self.load_btn.clicked.connect(self.main_window.load_environment)
        
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(self.browse_btn)
        file_layout.addWidget(self.load_btn)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # 骨干网络配置 - 紧凑版
        backbone_group = QGroupBox("🌐 骨干网络")
        backbone_layout = QGridLayout()
        backbone_layout.setSpacing(4)
        
        backbone_layout.addWidget(QLabel("质量:"), 0, 0)
        self.quality_spin = QDoubleSpinBox()
        self.quality_spin.setRange(0.1, 1.0)
        self.quality_spin.setSingleStep(0.1)
        self.quality_spin.setValue(0.6)
        self.quality_spin.setMaximumHeight(25)
        backbone_layout.addWidget(self.quality_spin, 0, 1)
        
        self.generate_btn = QPushButton("🚀 生成")
        self.generate_btn.setMaximumHeight(30)
        self._compact_button_style(self.generate_btn, "#6f42c1")
        self.generate_btn.clicked.connect(self.main_window.generate_backbone_network)
        backbone_layout.addWidget(self.generate_btn, 1, 0, 1, 2)
        
        self.backbone_stats_label = QLabel("等待生成...")
        self.backbone_stats_label.setStyleSheet("QLabel { color: #6c757d; font-size: 9px; }")
        backbone_layout.addWidget(self.backbone_stats_label, 2, 0, 1, 2)
        
        backbone_group.setLayout(backbone_layout)
        layout.addWidget(backbone_group)
        
        # 调度控制 - 紧凑版
        schedule_group = QGroupBox("🚛 调度")
        schedule_layout = QVBoxLayout()
        schedule_layout.setSpacing(4)
        
        self.assign_all_btn = QPushButton("📋 分配所有")
        self.assign_single_btn = QPushButton("🎯 分配单个")
        
        for btn in [self.assign_all_btn, self.assign_single_btn]:
            btn.setMaximumHeight(28)
            self._compact_button_style(btn, "#fd7e14")
        
        self.assign_all_btn.clicked.connect(self.main_window.assign_all_vehicles)
        self.assign_single_btn.clicked.connect(self.main_window.assign_single_vehicle)
        
        schedule_layout.addWidget(self.assign_all_btn)
        schedule_layout.addWidget(self.assign_single_btn)
        schedule_group.setLayout(schedule_layout)
        layout.addWidget(schedule_group)
        
        # 仿真控制 - 紧凑版
        sim_group = QGroupBox("⚡ 仿真")
        sim_layout = QVBoxLayout()
        sim_layout.setSpacing(4)
        
        button_layout = QHBoxLayout()
        button_layout.setSpacing(2)
        
        self.start_btn = QPushButton("▶️")
        self.pause_btn = QPushButton("⏸️")
        self.reset_btn = QPushButton("🔄")
        
        for btn in [self.start_btn, self.pause_btn, self.reset_btn]:
            btn.setMaximumHeight(28)
            btn.setMaximumWidth(35)
            self._compact_button_style(btn)
        
        self.start_btn.clicked.connect(self.main_window.start_simulation)
        self.pause_btn.clicked.connect(self.main_window.pause_simulation)
        self.reset_btn.clicked.connect(self.main_window.reset_simulation)
        
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.pause_btn)
        button_layout.addWidget(self.reset_btn)
        
        # 仿真速度 - 紧凑版
        speed_layout = QHBoxLayout()
        speed_layout.setSpacing(4)
        
        speed_layout.addWidget(QLabel("速度:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.speed_slider.setMaximumHeight(20)
        self.speed_label = QLabel("1.0x")
        self.speed_label.setMinimumWidth(30)
        
        self.speed_slider.valueChanged.connect(self._update_speed_label)
        
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_label)
        
        # 紧凑的进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximumHeight(15)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #dee2e6;
                border-radius: 3px;
                text-align: center;
                font-size: 8px;
            }
            QProgressBar::chunk {
                background-color: #007bff;
                border-radius: 2px;
            }
        """)
        
        sim_layout.addLayout(button_layout)
        sim_layout.addLayout(speed_layout)
        sim_layout.addWidget(self.progress_bar)
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # 显示选项 - 紧凑版
        display_group = QGroupBox("👁️ 显示")
        display_layout = QVBoxLayout()
        display_layout.setSpacing(2)
        
        self.show_backbone_cb = QCheckBox("骨干网络")
        self.show_interfaces_cb = QCheckBox("接口点")
        self.show_paths_cb = QCheckBox("车辆路径")
        
        for cb in [self.show_backbone_cb, self.show_interfaces_cb, self.show_paths_cb]:
            cb.setStyleSheet("QCheckBox { font-size: 10px; }")
        
        self.show_backbone_cb.setChecked(True)
        self.show_paths_cb.setChecked(True)
        
        self.show_backbone_cb.toggled.connect(self.main_window.toggle_backbone_display)
        self.show_interfaces_cb.toggled.connect(self.main_window.toggle_interfaces_display)
        self.show_paths_cb.toggled.connect(self.main_window.toggle_paths_display)
        
        display_layout.addWidget(self.show_backbone_cb)
        display_layout.addWidget(self.show_interfaces_cb)
        display_layout.addWidget(self.show_paths_cb)
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        layout.addStretch()
    
    def _compact_button_style(self, button, color="#007bff"):
        """紧凑按钮样式"""
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                padding: 4px 8px;
                border-radius: 3px;
                font-size: 10px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {self._darken_color(color)};
            }}
            QPushButton:pressed {{
                background-color: {self._darken_color(color, 0.8)};
            }}
            QPushButton:disabled {{
                background-color: #6c757d;
                color: #adb5bd;
            }}
        """)
    
    def _darken_color(self, color, factor=0.9):
        """颜色加深"""
        color = QColor(color)
        return f"rgb({int(color.red()*factor)}, {int(color.green()*factor)}, {int(color.blue()*factor)})"
    
    def _update_speed_label(self, value):
        """更新速度标签"""
        speed = value / 5.0
        self.speed_label.setText(f"{speed:.1f}x")
        if hasattr(self.main_window, 'set_simulation_speed'):
            self.main_window.set_simulation_speed(speed)


class CompactStatusPanel(QWidget):
    """紧凑的状态面板"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        
        # 实时状态 - 紧凑版
        real_time_group = QGroupBox("📊 实时状态")
        real_time_layout = QGridLayout()
        real_time_layout.setSpacing(4)
        
        # 小尺寸LCD
        self.vehicles_lcd = QLCDNumber(2)
        self.vehicles_lcd.setMaximumHeight(25)
        self.vehicles_lcd.setStyleSheet("QLCDNumber { color: #28a745; }")
        
        self.tasks_lcd = QLCDNumber(3)
        self.tasks_lcd.setMaximumHeight(25)
        self.tasks_lcd.setStyleSheet("QLCDNumber { color: #007bff; }")
        
        real_time_layout.addWidget(QLabel("车辆:"), 0, 0)
        real_time_layout.addWidget(self.vehicles_lcd, 0, 1)
        real_time_layout.addWidget(QLabel("任务:"), 1, 0)
        real_time_layout.addWidget(self.tasks_lcd, 1, 1)
        
        real_time_group.setLayout(real_time_layout)
        layout.addWidget(real_time_group)
        
        # 性能监控
        self.performance_monitor = CompactPerformanceMonitor()
        layout.addWidget(self.performance_monitor)
        
        # 紧凑的车辆表格
        vehicle_group = QGroupBox("🚛 车辆")
        vehicle_layout = QVBoxLayout()
        vehicle_layout.setSpacing(4)
        
        self.vehicle_table = QTableWidget()
        self.vehicle_table.setColumnCount(3)
        self.vehicle_table.setHorizontalHeaderLabels(["ID", "状态", "位置"])
        self.vehicle_table.setMaximumHeight(150)
        self.vehicle_table.setAlternatingRowColors(True)
        
        # 设置表格样式
        self.vehicle_table.setStyleSheet("""
            QTableWidget {
                font-size: 9px;
                gridline-color: #dee2e6;
            }
            QHeaderView::section {
                background-color: #f8f9fa;
                padding: 2px;
                border: 1px solid #dee2e6;
                font-size: 9px;
                font-weight: bold;
            }
        """)
        
        vehicle_layout.addWidget(self.vehicle_table)
        vehicle_group.setLayout(vehicle_layout)
        layout.addWidget(vehicle_group)
        
        # 紧凑日志
        log_group = QGroupBox("📝 日志")
        log_layout = QVBoxLayout()
        log_layout.setSpacing(4)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(120)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                color: #495057;
                border: 1px solid #dee2e6;
                border-radius: 3px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 8px;
            }
        """)
        
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        layout.addStretch()
    
    def update_status(self, stats):
        """更新状态显示"""
        if not stats:
            return
        
        real_time = stats.get('real_time', {})
        active = real_time.get('active_vehicles', 0)
        self.vehicles_lcd.display(active)
        
        completed = stats.get('completed_tasks', 0)
        self.tasks_lcd.display(completed)
    
    def update_vehicle_table(self, vehicles):
        """更新车辆表格"""
        if not vehicles:
            return
        
        self.vehicle_table.setRowCount(len(vehicles))
        
        status_map = {
            'idle': '🟢',
            'moving': '🔵',
            'loading': '🟡',
            'unloading': '🔴',
            'planning': '🟣'
        }
        
        for row, (vehicle_id, vehicle_data) in enumerate(vehicles.items()):
            # ID列
            self.vehicle_table.setItem(row, 0, QTableWidgetItem(str(vehicle_id)))
            
            # 状态列
            status = vehicle_data.get('status', 'idle')
            status_text = status_map.get(status, status)
            self.vehicle_table.setItem(row, 1, QTableWidgetItem(status_text))
            
            # 位置列
            pos = vehicle_data.get('position', (0, 0, 0))
            pos_str = f"({pos[0]:.0f},{pos[1]:.0f})"
            self.vehicle_table.setItem(row, 2, QTableWidgetItem(pos_str))
    
    def add_log(self, message, level="info"):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        
        level_styles = {
            "error": ("🔴", "#dc3545"),
            "warning": ("🟡", "#ffc107"), 
            "success": ("🟢", "#28a745"),
            "info": ("ℹ️", "#007bff")
        }
        
        icon, color = level_styles.get(level, ("ℹ️", "#495057"))
        
        formatted_message = f'<span style="color: {color}; font-size: 8px;">[{timestamp}] {icon} {message}</span>'
        
        self.log_text.append(formatted_message)
        
        # 自动滚动到底部
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class OptimizedMineGUI(QMainWindow):
    """优化版主窗口 - 简洁美观的界面设计"""
    
    def __init__(self):
        super().__init__()
        
        # 系统组件
        self.env = None
        self.backbone_network = None
        self.path_planner = None
        self.vehicle_scheduler = None
        self.traffic_manager = None
        
        # 状态
        self.is_simulating = False
        self.simulation_time = 0
        self.simulation_speed = 1.0
        self.map_file_path = None
        
        self.init_ui()
        
        # 定时器
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)
        
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.simulation_step)
        
        self.stats_timer = QTimer(self)
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(2000)
        
        self.performance_timer = QTimer(self)
        self.performance_timer.timeout.connect(self.update_performance_stats)
        self.performance_timer.start(5000)
        
        # 添加状态栏
        self.status_label = QLabel("系统就绪")
        self.statusBar().addWidget(self.status_label)
        self.statusBar().setStyleSheet("QStatusBar { font-size: 10px; }")
        
        # 优化组件提示
        if OPTIMIZED_COMPONENTS_AVAILABLE:
            self.status_label.setText("✅ 优化组件已加载 - 系统就绪")
        else:
            self.status_label.setText("⚠️ 使用兼容模式 - 系统就绪")
    
    def init_ui(self):
        """初始化简洁界面"""
        self.setWindowTitle("露天矿调度系统 - 简洁版")
        self.setGeometry(100, 100, 1200, 800)
        
        # 简洁的样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #ffffff;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #dee2e6;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 4px;
                font-size: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(6, 6, 6, 6)
        
        # 左侧控制面板 - 更窄
        self.control_panel = CompactControlPanel(self)
        self.control_panel.setMaximumWidth(220)
        main_layout.addWidget(self.control_panel)
        
        # 中央地图视图
        self.map_view = MinimalMapView()
        main_layout.addWidget(self.map_view, 1)
        
        # 右侧状态面板 - 更窄
        self.status_panel = CompactStatusPanel()
        self.status_panel.setMaximumWidth(280)
        main_layout.addWidget(self.status_panel)
        
        # 创建菜单
        self.create_menu_bar()
        
        self.enable_controls(False)
    
    def create_menu_bar(self):
        """创建简洁菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu('文件')
        
        open_action = file_menu.addAction('打开地图')
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.browse_file)
        
        file_menu.addSeparator()
        
        exit_action = file_menu.addAction('退出')
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        
        # 仿真菜单
        sim_menu = menubar.addMenu('仿真')
        
        start_action = sim_menu.addAction('开始')
        start_action.setShortcut('F5')
        start_action.triggered.connect(self.start_simulation)
        
        pause_action = sim_menu.addAction('暂停')
        pause_action.setShortcut('F6')
        pause_action.triggered.connect(self.pause_simulation)
        
        reset_action = sim_menu.addAction('重置')
        reset_action.setShortcut('F7')
        reset_action.triggered.connect(self.reset_simulation)
        
        # 视图菜单
        view_menu = menubar.addMenu('视图')
        
        zoom_fit_action = view_menu.addAction('适应窗口')
        zoom_fit_action.setShortcut('Ctrl+0')
        zoom_fit_action.triggered.connect(self.zoom_fit)
    
    # 文件操作
    def browse_file(self):
        """浏览文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开地图文件", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            self.map_file_path = file_path
            filename = os.path.basename(file_path)
            self.control_panel.file_label.setText(filename)
            self.control_panel.file_label.setStyleSheet("QLabel { color: #28a745; font-size: 10px; }")
            self.status_panel.add_log(f"已选择地图文件: {filename}")
    
    def load_environment(self):
        """加载环境"""
        if not self.map_file_path:
            QMessageBox.warning(self, "警告", "请先选择地图文件")
            return
        
        try:
            self.status_panel.add_log("正在加载环境...", "info")
            self.status_label.setText("正在加载环境...")
            
            # 创建环境
            self.env = OptimizedOpenPitMineEnv()
            if not self.env.load_from_file(self.map_file_path):
                raise Exception("环境加载失败")
            
            # 设置到地图视图
            self.map_view.set_environment(self.env)
            
            # 创建系统组件
            self.create_system_components()
            
            self.status_panel.add_log("环境加载成功", "success")
            self.status_label.setText("环境已加载")
            self.enable_controls(True)
            
            # 显示环境信息
            summary = self.env.get_environment_summary()
            info_text = f"地图尺寸: {summary['dimensions']}, 车辆: {summary['vehicles']}辆"
            self.status_panel.add_log(f"环境信息: {info_text}", "info")
            
        except Exception as e:
            error_msg = f"加载环境失败: {str(e)}"
            self.status_panel.add_log(error_msg, "error")
            self.status_label.setText("加载失败")
            QMessageBox.critical(self, "错误", f"加载环境失败:\n{str(e)}")
    
    def create_system_components(self):
        """创建系统组件"""
        try:
            # 创建增强路径规划器
            self.path_planner = EnhancedPathPlannerWithConfig(self.env)
            self.status_panel.add_log("✅ 路径规划器初始化完成", "success")
            
            # 创建优化骨干网络
            self.backbone_network = OptimizedBackboneNetwork(self.env)
            self.backbone_network.set_path_planner(self.path_planner)
            
            # 创建交通管理器
            self.traffic_manager = OptimizedTrafficManager(
                self.env, self.backbone_network, self.path_planner
            )
            
            # 创建车辆调度器
            self.vehicle_scheduler = SimplifiedVehicleScheduler(
                self.env, self.path_planner, self.backbone_network, self.traffic_manager
            )
            
            # 设置组件间引用
            if hasattr(self.path_planner, 'set_backbone_network'):
                self.path_planner.set_backbone_network(self.backbone_network)
            
            # 初始化车辆状态
            self.vehicle_scheduler.initialize_vehicles()
            
            # 创建默认任务模板
            if self.env.loading_points and self.env.unloading_points:
                self.vehicle_scheduler.create_enhanced_mission_template("default")
            
            component_info = f"组件初始化完成 - 优化模式: {OPTIMIZED_COMPONENTS_AVAILABLE}"
            self.status_panel.add_log(component_info, "success")
            
        except Exception as e:
            error_msg = f"组件初始化失败: {str(e)}"
            self.status_panel.add_log(error_msg, "error")
            raise
    
    # 骨干网络操作
    def generate_backbone_network(self):
        """生成骨干网络"""
        if not self.env or not self.backbone_network:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            self.status_panel.add_log("开始生成骨干网络...", "info")
            self.status_label.setText("正在生成骨干网络...")
            
            quality_threshold = self.control_panel.quality_spin.value()
            
            success = self.backbone_network.generate_backbone_network(
                quality_threshold=quality_threshold
            )
            
            if success:
                # 更新组件引用
                if hasattr(self.path_planner, 'set_backbone_network'):
                    self.path_planner.set_backbone_network(self.backbone_network)
                if self.traffic_manager:
                    self.traffic_manager.set_backbone_network(self.backbone_network)
                if self.vehicle_scheduler:
                    self.vehicle_scheduler.set_backbone_network(self.backbone_network)
                
                # 更新地图显示
                self.map_view.set_backbone_network(self.backbone_network)
                
                # 获取统计信息
                status = self.backbone_network.get_network_status()
                
                if 'bidirectional_paths' in status:
                    path_count = status['bidirectional_paths']
                    stats_text = f"双向路径: {path_count}"
                else:
                    path_count = status.get('total_paths', 0)
                    stats_text = f"路径: {path_count}"
                
                self.control_panel.backbone_stats_label.setText(stats_text)
                self.control_panel.backbone_stats_label.setStyleSheet("QLabel { color: #28a745; font-size: 9px; }")
                
                self.status_label.setText("骨干网络生成成功")
                self.status_panel.add_log(f"骨干网络生成成功: {stats_text}", "success")
            else:
                self.status_panel.add_log("骨干网络生成失败", "error")
                self.status_label.setText("生成失败")
                QMessageBox.critical(self, "错误", "骨干网络生成失败")
        
        except Exception as e:
            error_msg = f"生成骨干网络失败: {str(e)}"
            self.status_panel.add_log(error_msg, "error")
            self.status_label.setText("生成异常")
            QMessageBox.critical(self, "错误", f"生成骨干网络失败:\n{str(e)}")
    
    # 调度操作
    def assign_all_vehicles(self):
        """批量分配任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        assigned_count = 0
        for vehicle_id in self.env.vehicles.keys():
            if self.vehicle_scheduler.assign_mission_intelligently(vehicle_id):
                assigned_count += 1
        
        success_msg = f"已为 {assigned_count} 个车辆分配任务"
        self.status_panel.add_log(success_msg, "success")
    
    def assign_single_vehicle(self):
        """分配单个车辆任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        # 简单实现：分配给第一个空闲车辆
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            if vehicle_data.get('status') == 'idle':
                if self.vehicle_scheduler.assign_mission_intelligently(vehicle_id):
                    msg = f"已为车辆 {vehicle_id} 分配任务"
                    self.status_panel.add_log(msg, "success")
                    return
        
        self.status_panel.add_log("没有找到空闲车辆", "warning")
    
    # 仿真控制
    def start_simulation(self):
        """开始仿真"""
        if not self.env:
            return
        
        self.is_simulating = True
        self.control_panel.start_btn.setEnabled(False)
        self.control_panel.pause_btn.setEnabled(True)
        
        # 根据速度设置定时器间隔
        interval = int(200 / self.simulation_speed)
        self.sim_timer.start(interval)
        
        self.status_panel.add_log("仿真已开始", "success")
        self.status_label.setText("仿真运行中...")
    
    def pause_simulation(self):
        """暂停仿真"""
        self.is_simulating = False
        self.control_panel.start_btn.setEnabled(True)
        self.control_panel.pause_btn.setEnabled(False)
        
        self.sim_timer.stop()
        
        self.status_panel.add_log("仿真已暂停", "info")
        self.status_label.setText("仿真已暂停")
    
    def reset_simulation(self):
        """重置仿真"""
        if self.is_simulating:
            self.pause_simulation()
        
        if self.env:
            self.env.reset()
        
        if self.vehicle_scheduler:
            self.vehicle_scheduler.initialize_vehicles()
        
        self.simulation_time = 0
        self.map_view.redraw_environment()
        self.control_panel.progress_bar.setValue(0)
        
        self.status_panel.add_log("仿真已重置", "info")
        self.status_label.setText("仿真已重置")
    
    def set_simulation_speed(self, speed):
        """设置仿真速度"""
        self.simulation_speed = speed
        if self.is_simulating:
            interval = int(200 / speed)
            self.sim_timer.setInterval(interval)
    
    def simulation_step(self):
        """仿真步骤"""
        if not self.is_simulating or not self.env:
            return
        
        time_step = 0.2 * self.simulation_speed
        self.simulation_time += time_step
        
        # 更新环境
        self.env.current_time = self.simulation_time
        
        # 更新调度器
        if self.vehicle_scheduler:
            try:
                self.vehicle_scheduler.update(time_step)
            except Exception as e:
                self.status_panel.add_log(f"调度器更新错误: {e}", "error")
        
        # 更新交通管理器
        if self.traffic_manager:
            try:
                self.traffic_manager.update(time_step)
            except Exception as e:
                self.status_panel.add_log(f"交通管理器更新错误: {e}", "error")
        
        # 更新进度条
        max_time = 1800  # 30分钟
        progress = min(100, int(self.simulation_time * 100 / max_time))
        self.control_panel.progress_bar.setValue(progress)
        
        if progress >= 100:
            self.pause_simulation()
            self.status_panel.add_log("仿真完成", "success")
    
    # 显示控制
    def toggle_backbone_display(self, show: bool):
        """切换骨干网络显示"""
        self.map_view.toggle_backbone_display(show)
    
    def toggle_interfaces_display(self, show: bool):
        """切换接口显示"""
        self.map_view.toggle_interfaces_display(show)
    
    def toggle_paths_display(self, show: bool):
        """切换路径显示"""
        self.map_view.show_paths = show
        self.map_view.redraw_environment()
    
    # 显示更新
    def update_display(self):
        """更新显示"""
        if not self.env:
            return
        
        self.map_view.update_vehicles()
    
    def update_statistics(self):
        """更新统计信息"""
        if not self.vehicle_scheduler:
            return
        
        try:
            stats = self.vehicle_scheduler.get_comprehensive_stats()
            self.status_panel.update_status(stats)
            self.status_panel.update_vehicle_table(self.env.vehicles)
            
            # 更新冲突计数
            conflicts_count = 0
            if self.traffic_manager:
                try:
                    conflicts = self.traffic_manager.detect_all_conflicts()
                    conflicts_count = len(conflicts)
                except Exception:
                    pass
            
            # 更新性能监控器的交通统计
            if hasattr(self.status_panel, 'performance_monitor'):
                active_vehicles = len([v for v in self.env.vehicles.values() 
                                     if v.get('status') != 'idle'])
                resolved_conflicts = 0
                
                self.status_panel.performance_monitor.update_traffic_stats(
                    active_vehicles, conflicts_count, resolved_conflicts
                )
        
        except Exception as e:
            self.status_panel.add_log(f"统计更新错误: {e}", "warning")
    
    def update_performance_stats(self):
        """更新性能统计"""
        try:
            # 更新骨干网络统计
            if self.backbone_network:
                backbone_stats = self.backbone_network.get_network_status()
                self.status_panel.performance_monitor.update_backbone_stats(backbone_stats)
        
        except Exception as e:
            self.status_panel.add_log(f"性能统计更新错误: {e}", "warning")
    
    # 工具方法
    def enable_controls(self, enabled):
        """启用/禁用控件"""
        self.control_panel.start_btn.setEnabled(enabled)
        self.control_panel.reset_btn.setEnabled(enabled)
        self.control_panel.generate_btn.setEnabled(enabled)
        self.control_panel.assign_all_btn.setEnabled(enabled)
        self.control_panel.assign_single_btn.setEnabled(enabled)
    
    def zoom_fit(self):
        """适应窗口大小"""
        if self.env:
            self.map_view.fitInView(self.map_view.scene.sceneRect(), Qt.KeepAspectRatio)
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.is_simulating:
            reply = QMessageBox.question(
                self, '确认退出', '仿真正在运行，确定要退出吗？',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.No:
                event.ignore()
                return
        
        # 停止所有定时器
        self.update_timer.stop()
        self.sim_timer.stop()
        self.stats_timer.stop()
        self.performance_timer.stop()
        
        # 关闭组件
        if self.path_planner and hasattr(self.path_planner, 'shutdown'):
            self.path_planner.shutdown()
        
        if self.traffic_manager and hasattr(self.traffic_manager, 'shutdown'):
            self.traffic_manager.shutdown()
        
        if self.vehicle_scheduler and hasattr(self.vehicle_scheduler, 'shutdown'):
            self.vehicle_scheduler.shutdown()
        
        self.status_panel.add_log("系统已关闭", "info")
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("露天矿调度系统")
    app.setApplicationVersion("简洁版")
    app.setOrganizationName("优化设计")
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    # 加载字体
    try:
        import platform
        if platform.system() == "Windows":
            app.setFont(QFont("Microsoft YaHei", 8))
        else:
            app.setFont(QFont("Arial", 8))
    except:
        pass
    
    window = OptimizedMineGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()