#!/usr/bin/env python3
"""
improved_mine_gui.py - 改进版露天矿多车协同调度系统GUI
修复递归问题，完善骨干路径可视化，展示冲突消解能力
"""

import sys
import os
import math
import time
import json
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPointF, QRectF
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QGroupBox, QGridLayout, QTableWidget, QTableWidgetItem,
    QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QDockWidget,
    QGraphicsRectItem, QGraphicsPathItem, QTabWidget, QFrame,
    QSlider, QCheckBox, QGraphicsItemGroup, QGraphicsPolygonItem, 
    QGraphicsLineItem, QGraphicsTextItem, QAction, QMenuBar, QMenu
)
from PyQt5.QtGui import (
    QPen, QBrush, QColor, QPainter, QPainterPath, QFont, QIcon,
    QPolygonF, QTransform
)

# 导入系统组件
try:
    from optimized_backbone_network import OptimizedBackboneNetwork
    from optimized_planner_config import EnhancedPathPlannerWithConfig
    from traffic_manager import OptimizedTrafficManagerWithECBS
    from vehicle_scheduler import EnhancedVehicleScheduler
    from environment import OptimizedOpenPitMineEnv
    COMPONENTS_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ 组件导入失败: {e}")
    COMPONENTS_AVAILABLE = False
    sys.exit(1)

# 配色方案
COLORS = {
    'bg': QColor(45, 47, 57),
    'surface': QColor(55, 58, 71),
    'primary': QColor(66, 135, 245),
    'success': QColor(16, 185, 129),
    'warning': QColor(245, 158, 11),
    'error': QColor(239, 68, 68),
    'text': QColor(229, 231, 235),
    'muted': QColor(156, 163, 175),
    'border': QColor(75, 85, 99),
    'backbone': QColor(147, 51, 234),  # 紫色用于骨干路径
    'interface': QColor(59, 130, 246), # 蓝色用于接口节点
    'conflict': QColor(239, 68, 68),  # 红色用于冲突
}

# 车辆状态颜色
VEHICLE_COLORS = {
    'idle': QColor(156, 163, 175),
    'loading': QColor(16, 185, 129),
    'unloading': QColor(245, 158, 11),
    'moving': QColor(66, 135, 245),
    'waiting': QColor(168, 85, 247),
    'planning': QColor(244, 63, 94),
    'coordinating': QColor(34, 197, 94),
}


class BackboneInterfaceItem(QGraphicsItemGroup):
    """骨干网络接口节点图形项"""
    
    def __init__(self, interface_id: str, position: Tuple[float, float], 
                 path_id: str, index: int):
        super().__init__()
        self.interface_id = interface_id
        self.position = position
        self.path_id = path_id
        self.index = index
        
        # 创建节点圆形
        self.node = QGraphicsEllipseItem(-1, -1, 2, 2, self)
        self.node.setBrush(QBrush(COLORS['interface']))
        self.node.setPen(QPen(COLORS['interface'].darker(150), 0.5))
        self.node.setZValue(15)
        
        # 创建标签
        self.label = QGraphicsTextItem(f"I{index}", self)
        self.label.setDefaultTextColor(COLORS['text'])
        self.label.setFont(QFont("Arial", 1))
        self.label.setPos(-10, -20)
        self.label.setZValue(16)
        
        # 设置位置
        self.setPos(position[0], position[1])
        
        # 默认隐藏标签
        self.label.setVisible(False)
    
    def set_highlighted(self, highlighted: bool):
        """设置高亮状态"""
        if highlighted:
            self.node.setBrush(QBrush(COLORS['warning']))
            self.node.setPen(QPen(COLORS['warning'].darker(150), 1))
            self.label.setVisible(True)
        else:
            self.node.setBrush(QBrush(COLORS['interface']))
            self.node.setPen(QPen(COLORS['interface'].darker(150), 1))
            self.label.setVisible(False)


class BackbonePathItem(QGraphicsPathItem):
    """骨干路径图形项"""
    
    def __init__(self, path_id: str, path_data: Any):
        super().__init__()
        self.path_id = path_id
        self.path_data = path_data
        self.setZValue(-20)  # 在底层
        
        # 创建路径
        self.update_path()
        
        # 设置样式
        self.update_style()
    
    def update_path(self):
        """更新路径"""
        if not self.path_data.forward_path or len(self.path_data.forward_path) < 2:
            return
        
        painter_path = QPainterPath()
        painter_path.moveTo(self.path_data.forward_path[0][0], 
                           self.path_data.forward_path[0][1])
        
        for point in self.path_data.forward_path[1:]:
            painter_path.lineTo(point[0], point[1])
        
        self.setPath(painter_path)
    
    def update_style(self):
        """更新样式（根据负载）"""
        load_factor = self.path_data.get_load_factor()
        quality = self.path_data.quality
        
        # 根据负载设置颜色和宽度
        if load_factor > 0.8:
            color = COLORS['error']
            width = 3.0
        elif load_factor > 0.5:
            color = COLORS['warning']
            width = 2.5
        else:
            color = COLORS['backbone']
            width = 2.0
        
        # 根据质量调整透明度
        color.setAlphaF(0.5 + quality * 0.5)
        
        pen = QPen(color, width)
        pen.setCapStyle(Qt.RoundCap)
        pen.setStyle(Qt.DashLine if quality < 0.6 else Qt.SolidLine)
        self.setPen(pen)
    
    def set_highlighted(self, highlighted: bool):
        """设置高亮状态"""
        if highlighted:
            pen = self.pen()
            pen.setWidth(pen.width() + 1)
            pen.setStyle(Qt.SolidLine)
            color = pen.color()
            color.setAlphaF(1.0)
            pen.setColor(color)
            self.setPen(pen)
        else:
            self.update_style()


class ConflictMarker(QGraphicsItemGroup):
    """冲突标记"""
    
    def __init__(self, conflict_id: str, position: Tuple[float, float], 
                 severity: float = 1.0):
        super().__init__()
        self.conflict_id = conflict_id
        self.position = position
        self.severity = severity
        
        # 冲突区域（半透明圆）
        radius = 10 + severity * 10
        self.area = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2, self)
        self.area.setBrush(QBrush(QColor(239, 68, 68, 50)))
        self.area.setPen(QPen(Qt.NoPen))
        self.area.setZValue(25)
        
        # 冲突中心标记
        self.marker = QGraphicsEllipseItem(-1, -1, 2, 2, self)
        self.marker.setBrush(QBrush(COLORS['conflict']))
        self.marker.setPen(QPen(COLORS['conflict'].darker(150), 1))
        self.marker.setZValue(26)
        
        # 脉冲动画效果
        self.pulse_timer = QTimer()
        self.pulse_timer.timeout.connect(self._pulse_animation)
        self.pulse_timer.start(50)
        self.pulse_phase = 0
        
        self.setPos(position[0], position[1])
    
    def _pulse_animation(self):
        """脉冲动画"""
        self.pulse_phase = (self.pulse_phase + 0.1) % (2 * math.pi)
        scale = 1.0 + 0.2 * math.sin(self.pulse_phase)
        self.marker.setScale(scale)
        
        # 更新透明度
        alpha = int(50 + 30 * math.sin(self.pulse_phase))
        color = QColor(239, 68, 68, alpha)
        self.area.setBrush(QBrush(color))
    
    def cleanup(self):
        """清理资源"""
        self.pulse_timer.stop()


class EnhancedVehicleItem(QGraphicsItemGroup):
    """增强的车辆图形项"""
    
    def __init__(self, vehicle_id: str, vehicle_data):
        super().__init__()
        self.vehicle_id = vehicle_id
        self.vehicle_data = vehicle_data
        self.position = vehicle_data.get('position', (0, 0, 0))
        self.update_lock = False  # 防止递归更新
        
        # 车辆形状
        self.body = QGraphicsPolygonItem(self)
        self.body.setZValue(10)
        
        # 方向指示器
        self.direction = QGraphicsLineItem(self)
        self.direction.setZValue(11)
        
        # ID标签
        self.label = QGraphicsTextItem(str(vehicle_id), self)
        self.label.setDefaultTextColor(COLORS['text'])
        self.label.setFont(QFont("Arial", 1, QFont.Bold))
        self.label.setZValue(12)
        
        # 状态指示器
        self.status_indicator = QGraphicsEllipseItem(self)
        self.status_indicator.setZValue(13)
        
        # 安全矩形
        self.safety_rect = QGraphicsRectItem(self)
        self.safety_rect.setZValue(5)
        self.safety_rect.setVisible(False)
        
        # 冲突警告标记
        self.conflict_warning = QGraphicsEllipseItem(self)
        self.conflict_warning.setZValue(14)
        self.conflict_warning.setVisible(False)
        
        self.update_appearance()
        self.update_position()
    
    def update_appearance(self):
        """更新外观"""
        if self.update_lock:
            return
            
        status = self.vehicle_data.get('status', 'idle')
        color = VEHICLE_COLORS.get(status, VEHICLE_COLORS['idle'])
        
        # 车身
        self.body.setBrush(QBrush(color))
        self.body.setPen(QPen(color.darker(150), 1))
        
        # 方向线
        self.direction.setPen(QPen(COLORS['text'], 1))
        
        # 状态指示器
        self.status_indicator.setBrush(QBrush(color.lighter(150)))
        self.status_indicator.setPen(QPen(Qt.NoPen))
        self.status_indicator.setRect(-1, -1, 2, 2)
        
        # 安全矩形
        self._update_safety_rect()
        
        # 冲突警告
        self._update_conflict_warning()
    
    def _update_safety_rect(self):
        """更新安全矩形"""
        safety_params = self.vehicle_data.get('safety_params', {})
        if safety_params:
            if hasattr(safety_params, 'length'):
                length = safety_params.length + safety_params.safety_margin * 2
                width = safety_params.width + safety_params.safety_margin * 2
            else:
                length = safety_params.get('length', 6.0) + safety_params.get('safety_margin', 1.5) * 2
                width = safety_params.get('width', 3.0) + safety_params.get('safety_margin', 1.5) * 2
            
            self.safety_rect.setRect(-length/2, -width/2, length, width)
            self.safety_rect.setBrush(QBrush(Qt.NoBrush))
            self.safety_rect.setPen(QPen(QColor(255, 255, 0, 80), 1, Qt.DashLine))
    
    def _update_conflict_warning(self):
        """更新冲突警告"""
        if hasattr(self.vehicle_data, 'in_conflict') and self.vehicle_data.in_conflict:
            self.conflict_warning.setVisible(True)
            self.conflict_warning.setRect(-8, -8, 16, 16)
            self.conflict_warning.setBrush(QBrush(QColor(239, 68, 68, 100)))
            self.conflict_warning.setPen(QPen(COLORS['error'], 1))
        else:
            self.conflict_warning.setVisible(False)
    
    def update_position(self):
        """更新位置"""
        if self.update_lock:
            return
            
        x, y = self.position[0], self.position[1]
        theta = self.position[2] if len(self.position) > 2 else 0
        
        # 车辆形状
        length, width = 6.0, 3.0
        points = [
            QPointF(length/2, 0),
            QPointF(length/3, width/2),
            QPointF(-length/2, width/2),
            QPointF(-length/2, -width/2),
            QPointF(length/3, -width/2)
        ]
        
        # 应用变换
        transform = QTransform()
        transform.translate(x, y)
        transform.rotate(math.degrees(theta))
        
        polygon = QPolygonF()
        for point in points:
            polygon.append(transform.map(point))
        
        self.body.setPolygon(polygon)
        
        # 更新其他组件位置
        self.label.setPos(x - 15, y - 25)
        self.status_indicator.setPos(x, y)
        self.safety_rect.setPos(x, y)
        self.safety_rect.setRotation(math.degrees(theta))
        self.conflict_warning.setPos(x, y)
        
        # 方向线
        line_length = 4
        end_x = x + line_length * math.cos(theta)
        end_y = y + line_length * math.sin(theta)
        self.direction.setLine(x, y, end_x, end_y)
    
    def update_data(self, vehicle_data):
        """更新数据（防止递归）"""
        if self.update_lock:
            return
            
        self.update_lock = True
        try:
            self.vehicle_data = vehicle_data
            self.position = vehicle_data.get('position', self.position)
            self.update_position()
            self.update_appearance()
        finally:
            self.update_lock = False
    
    def set_safety_rect_visible(self, visible: bool):
        """设置安全矩形可见性"""
        self.safety_rect.setVisible(visible)


class ImprovedMineScene(QGraphicsScene):
    """改进的矿场场景"""
    
    def __init__(self):
        super().__init__()
        self.env = None
        self.backbone_network = None
        self.traffic_manager = None
        
        # 图形项容器
        self.vehicle_items = {}
        self.path_items = {}
        self.backbone_path_items = {}
        self.interface_items = {}
        self.conflict_markers = {}
        
        # 显示选项
        self.show_paths = True
        self.show_backbone = True
        self.show_interfaces = True
        self.show_conflicts = True
        self.show_safety_rects = False
        
        # 更新控制
        self.update_in_progress = False
        
        self.setSceneRect(0, 0, 500, 500)
    
    def set_environment(self, env):
        """设置环境"""
        self.env = env
        if env:
            self.setSceneRect(0, 0, env.width, env.height)
        
        self.clear()
        self._reset_items()
        
        if env:
            self.draw_environment()
    
    def _reset_items(self):
        """重置图形项"""
        # 清理冲突标记的定时器
        for marker in self.conflict_markers.values():
            marker.cleanup()
        
        self.vehicle_items.clear()
        self.path_items.clear()
        self.backbone_path_items.clear()
        self.interface_items.clear()
        self.conflict_markers.clear()
    
    def draw_environment(self):
        """绘制环境"""
        if not self.env:
            return
        
        # 背景
        bg = QGraphicsRectItem(0, 0, self.env.width, self.env.height)
        bg.setBrush(QBrush(COLORS['bg']))
        bg.setPen(QPen(Qt.NoPen))
        bg.setZValue(-100)
        self.addItem(bg)
        
        # 网格线（可选）
        self._draw_grid()
        
        # 障碍物
        self._draw_obstacles()
        
        # 特殊点
        self._draw_special_points()
        
        # 车辆
        self.draw_vehicles()
    
    def _draw_grid(self):
        """绘制网格线"""
        grid_size = 50
        pen = QPen(COLORS['border'], 0.5, Qt.DotLine)
        
        # 垂直线
        for x in range(0, int(self.env.width) + 1, grid_size):
            line = QGraphicsLineItem(x, 0, x, self.env.height)
            line.setPen(pen)
            line.setZValue(-99)
            self.addItem(line)
        
        # 水平线
        for y in range(0, int(self.env.height) + 1, grid_size):
            line = QGraphicsLineItem(0, y, self.env.width, y)
            line.setPen(pen)
            line.setZValue(-99)
            self.addItem(line)
    
    def _draw_obstacles(self):
        """绘制障碍物"""
        for x, y in self.env.obstacle_points:
            obstacle = QGraphicsRectItem(x, y, 1, 1)
            obstacle.setBrush(QBrush(COLORS['surface']))
            obstacle.setPen(QPen(COLORS['border'], 0.1))
            obstacle.setZValue(-50)
            self.addItem(obstacle)
    
    def _draw_special_points(self):
        """绘制特殊点"""
        # 装载点
        for i, point in enumerate(self.env.loading_points):
            self._draw_special_point(point[0], point[1], COLORS['success'], 
                                   f"L{i+1}", 'circle', 8)
        
        # 卸载点
        for i, point in enumerate(self.env.unloading_points):
            self._draw_special_point(point[0], point[1], COLORS['warning'], 
                                   f"U{i+1}", 'square', 8)
        
        # 停车区
        if hasattr(self.env, 'parking_areas'):
            for i, point in enumerate(self.env.parking_areas):
                self._draw_special_point(point[0], point[1], COLORS['primary'], 
                                       f"P{i+1}", 'diamond', 8)
    
    def _draw_special_point(self, x, y, color, label, shape='circle', size=6):
        """绘制特殊点（改进版）"""
        if shape == 'circle':
            area = QGraphicsEllipseItem(x-size/2, y-size/2, size, size)
        elif shape == 'square':
            area = QGraphicsRectItem(x-size/2, y-size/2, size, size)
        else:  # diamond
            diamond = QPolygonF([
                QPointF(x, y-size/2),
                QPointF(x+size/2, y),
                QPointF(x, y+size/2),
                QPointF(x-size/2, y)
            ])
            area = QGraphicsPolygonItem(diamond)
        
        area.setBrush(QBrush(color.lighter(150)))
        area.setPen(QPen(color, 1))
        area.setZValue(-20)
        self.addItem(area)
        
        # 标签
        text = QGraphicsTextItem(label)
        text.setPos(x-10, y-25)
        text.setDefaultTextColor(color)
        text.setFont(QFont("Arial", 1, QFont.Bold))
        text.setZValue(-10)
        self.addItem(text)
    
    def draw_vehicles(self):
        """绘制车辆"""
        if not self.env:
            return
        
        # 清除现有车辆
        for item in self.vehicle_items.values():
            self.removeItem(item)
        self.vehicle_items.clear()
        
        # 添加车辆
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            vehicle_item = EnhancedVehicleItem(vehicle_id, vehicle_data)
            vehicle_item.set_safety_rect_visible(self.show_safety_rects)
            self.addItem(vehicle_item)
            self.vehicle_items[vehicle_id] = vehicle_item
            
            # 绘制路径
            if self.show_paths and hasattr(vehicle_data, 'path') and vehicle_data.path:
                self._draw_vehicle_path(vehicle_id, vehicle_data.path)
    
    def _draw_vehicle_path(self, vehicle_id: str, path: List[Tuple]):
        """绘制车辆路径"""
        if not path or len(path) < 2:
            return
        
        # 移除旧路径
        if vehicle_id in self.path_items:
            self.removeItem(self.path_items[vehicle_id])
        
        painter_path = QPainterPath()
        painter_path.moveTo(path[0][0], path[0][1])
        
        for point in path[1:]:
            painter_path.lineTo(point[0], point[1])
        
        path_item = QGraphicsPathItem(painter_path)
        pen = QPen(COLORS['primary'], 1, Qt.DashLine)
        pen.setCapStyle(Qt.RoundCap)
        path_item.setPen(pen)
        path_item.setZValue(0)
        
        self.addItem(path_item)
        self.path_items[vehicle_id] = path_item
    
    def update_vehicles(self):
        """更新车辆（防止递归）"""
        if self.update_in_progress or not self.env:
            return
        
        self.update_in_progress = True
        try:
            for vehicle_id, vehicle_data in self.env.vehicles.items():
                if vehicle_id in self.vehicle_items:
                    self.vehicle_items[vehicle_id].update_data(vehicle_data)
                else:
                    vehicle_item = EnhancedVehicleItem(vehicle_id, vehicle_data)
                    vehicle_item.set_safety_rect_visible(self.show_safety_rects)
                    self.addItem(vehicle_item)
                    self.vehicle_items[vehicle_id] = vehicle_item
                
                # 更新路径
                if self.show_paths:
                    if hasattr(vehicle_data, 'path') and vehicle_data.path:
                        self._draw_vehicle_path(vehicle_id, vehicle_data.path)
                    elif vehicle_id in self.path_items:
                        self.removeItem(self.path_items[vehicle_id])
                        del self.path_items[vehicle_id]
        finally:
            self.update_in_progress = False
    
    def update_backbone_visualization(self):
        """更新骨干网络可视化"""
        # 清除旧的
        for item in self.backbone_path_items.values():
            self.removeItem(item)
        self.backbone_path_items.clear()
        
        for item in self.interface_items.values():
            self.removeItem(item)
        self.interface_items.clear()
        
        if not self.backbone_network or not self.show_backbone:
            return
        
        # 绘制骨干路径
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            if not path_data.forward_path or len(path_data.forward_path) < 2:
                continue
            
            # 创建路径项
            path_item = BackbonePathItem(path_id, path_data)
            self.addItem(path_item)
            self.backbone_path_items[path_id] = path_item
            
            # 绘制接口节点
            if self.show_interfaces:
                self._draw_path_interfaces(path_id, path_data)
    
    def _draw_path_interfaces(self, path_id: str, path_data: Any):
        """绘制路径的接口节点"""
        if path_id not in self.backbone_network.path_interfaces:
            return
        
        interfaces = self.backbone_network.path_interfaces[path_id]
        
        for i, interface_id in enumerate(interfaces):
            if interface_id in self.backbone_network.backbone_interfaces:
                interface_info = self.backbone_network.backbone_interfaces[interface_id]
                position = interface_info['position']
                
                interface_item = BackboneInterfaceItem(
                    interface_id, position, path_id, i
                )
                self.addItem(interface_item)
                self.interface_items[interface_id] = interface_item
    
    def update_conflicts(self):
        """更新冲突显示"""
        # 清除旧冲突
        for marker in self.conflict_markers.values():
            marker.cleanup()
            self.removeItem(marker)
        self.conflict_markers.clear()
        
        if not self.show_conflicts or not self.traffic_manager:
            return
        
        # 获取冲突
        try:
            conflicts = self.traffic_manager.detect_all_conflicts()
            
            for i, conflict in enumerate(conflicts[:20]):  # 最多显示20个
                conflict_id = f"conflict_{i}"
                
                # 创建冲突标记
                marker = ConflictMarker(
                    conflict_id,
                    conflict.location,
                    getattr(conflict, 'severity', 1.0)
                )
                
                self.addItem(marker)
                self.conflict_markers[conflict_id] = marker
                
        except Exception as e:
            print(f"更新冲突显示失败: {e}")
    
    def highlight_vehicle_path(self, vehicle_id: str):
        """高亮车辆路径"""
        if vehicle_id in self.path_items:
            path_item = self.path_items[vehicle_id]
            pen = path_item.pen()
            pen.setWidth(1)
            pen.setColor(COLORS['warning'])
            path_item.setPen(pen)
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.update_backbone_visualization()
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.traffic_manager = traffic_manager
    
    def toggle_safety_rects(self, visible: bool):
        """切换安全矩形显示"""
        self.show_safety_rects = visible
        for vehicle_item in self.vehicle_items.values():
            vehicle_item.set_safety_rect_visible(visible)


class ControlPanel(QWidget):
    """控制面板"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 文件操作
        self._create_file_group(layout)
        
        # 系统初始化
        self._create_init_group(layout)
        
        # ECBS协调控制
        self._create_ecbs_group(layout)
        
        # 冲突消解演示
        self._create_conflict_demo_group(layout)
        
        # 任务管理
        self._create_task_group(layout)
        
        # 仿真控制
        self._create_simulation_group(layout)
        
        # 显示选项
        self._create_display_group(layout)
        
        layout.addStretch()
    
    def _create_file_group(self, parent_layout):
        """创建文件操作组"""
        file_group = QGroupBox("文件操作")
        file_layout = QHBoxLayout()
        
        self.file_label = QLabel("未选择文件")
        self.file_label.setStyleSheet("""
            QLabel {
                background-color: rgb(45, 47, 57);
                padding: 6px;
                border: 1px solid rgb(75, 85, 99);
                border-radius: 3px;
                color: rgb(156, 163, 175);
            }
        """)
        
        self.browse_btn = QPushButton("浏览")
        self.load_btn = QPushButton("加载")
        
        file_layout.addWidget(self.file_label, 1)
        file_layout.addWidget(self.browse_btn)
        file_layout.addWidget(self.load_btn)
        
        file_group.setLayout(file_layout)
        parent_layout.addWidget(file_group)
    
    def _create_init_group(self, parent_layout):
        """创建系统初始化组"""
        init_group = QGroupBox("系统初始化")
        init_layout = QVBoxLayout()
        
        # 骨干网络
        backbone_layout = QHBoxLayout()
        backbone_layout.addWidget(QLabel("质量阈值:"))
        
        self.quality_spin = QDoubleSpinBox()
        self.quality_spin.setRange(0.3, 1.0)
        self.quality_spin.setValue(0.6)
        self.quality_spin.setSingleStep(0.1)
        
        self.generate_backbone_btn = QPushButton("生成骨干网络")
        
        backbone_layout.addWidget(self.quality_spin)
        backbone_layout.addWidget(self.generate_backbone_btn)
        
        init_layout.addLayout(backbone_layout)
        init_group.setLayout(init_layout)
        parent_layout.addWidget(init_group)
    
    def _create_ecbs_group(self, parent_layout):
        """创建ECBS协调控制组"""
        ecbs_group = QGroupBox("ECBS多车协调")
        ecbs_layout = QVBoxLayout()
        
        # 协调模式
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("模式:"))
        
        self.coord_mode_combo = QComboBox()
        self.coord_mode_combo.addItems(["批量协调", "实时协调", "周期协调"])
        
        mode_layout.addWidget(self.coord_mode_combo)
        ecbs_layout.addLayout(mode_layout)
        
        # 协调按钮
        coord_btn_layout = QHBoxLayout()
        self.coord_selected_btn = QPushButton("协调选中")
        self.coord_all_btn = QPushButton("协调所有")
        
        coord_btn_layout.addWidget(self.coord_selected_btn)
        coord_btn_layout.addWidget(self.coord_all_btn)
        
        ecbs_layout.addLayout(coord_btn_layout)
        ecbs_group.setLayout(ecbs_layout)
        parent_layout.addWidget(ecbs_group)
    
    def _create_conflict_demo_group(self, parent_layout):
        """创建冲突消解演示组"""
        demo_group = QGroupBox("冲突消解演示")
        demo_layout = QVBoxLayout()
        
        # 演示场景选择
        scenario_layout = QHBoxLayout()
        scenario_layout.addWidget(QLabel("场景:"))
        
        self.scenario_combo = QComboBox()
        self.scenario_combo.addItems([
            "对向冲突",
            "交叉路口冲突",
            "骨干路径拥塞",
            "多车聚集"
        ])
        
        scenario_layout.addWidget(self.scenario_combo)
        demo_layout.addLayout(scenario_layout)
        
        # 演示按钮
        demo_btn_layout = QHBoxLayout()
        self.create_scenario_btn = QPushButton("创建场景")
        self.resolve_conflict_btn = QPushButton("消解冲突")
        
        demo_btn_layout.addWidget(self.create_scenario_btn)
        demo_btn_layout.addWidget(self.resolve_conflict_btn)
        
        demo_layout.addLayout(demo_btn_layout)
        demo_group.setLayout(demo_layout)
        parent_layout.addWidget(demo_group)
    
    def _create_task_group(self, parent_layout):
        """创建任务管理组"""
        task_group = QGroupBox("任务管理")
        task_layout = QVBoxLayout()
        
        # 优先级
        priority_layout = QHBoxLayout()
        priority_layout.addWidget(QLabel("优先级:"))
        
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["低", "普通", "高", "紧急"])
        self.priority_combo.setCurrentIndex(1)
        
        priority_layout.addWidget(self.priority_combo)
        task_layout.addLayout(priority_layout)
        
        # 任务分配
        task_btn_layout = QHBoxLayout()
        self.assign_single_btn = QPushButton("分配单个")
        self.assign_batch_btn = QPushButton("批量分配")
        
        task_btn_layout.addWidget(self.assign_single_btn)
        task_btn_layout.addWidget(self.assign_batch_btn)
        
        task_layout.addLayout(task_btn_layout)
        task_group.setLayout(task_layout)
        parent_layout.addWidget(task_group)
    
    def _create_simulation_group(self, parent_layout):
        """创建仿真控制组"""
        sim_group = QGroupBox("仿真控制")
        sim_layout = QVBoxLayout()
        
        # 控制按钮
        control_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始")
        self.pause_btn = QPushButton("暂停")
        self.reset_btn = QPushButton("重置")
        
        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.pause_btn)
        control_layout.addWidget(self.reset_btn)
        
        sim_layout.addLayout(control_layout)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("速度:"))
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        
        self.speed_label = QLabel("1.0x")
        
        speed_layout.addWidget(self.speed_slider, 1)
        speed_layout.addWidget(self.speed_label)
        
        sim_layout.addLayout(speed_layout)
        
        # 进度条
        self.progress_bar = QProgressBar()
        sim_layout.addWidget(self.progress_bar)
        
        sim_group.setLayout(sim_layout)
        parent_layout.addWidget(sim_group)
    
    def _create_display_group(self, parent_layout):
        """创建显示选项组"""
        display_group = QGroupBox("显示选项")
        display_layout = QVBoxLayout()
        
        self.show_paths_cb = QCheckBox("显示路径")
        self.show_paths_cb.setChecked(True)
        
        self.show_backbone_cb = QCheckBox("显示骨干网络")
        self.show_backbone_cb.setChecked(True)
        
        self.show_interfaces_cb = QCheckBox("显示接口节点")
        self.show_interfaces_cb.setChecked(True)
        
        self.show_conflicts_cb = QCheckBox("显示冲突")
        self.show_conflicts_cb.setChecked(True)
        
        self.show_safety_cb = QCheckBox("显示安全矩形")
        self.show_safety_cb.setChecked(False)
        
        display_layout.addWidget(self.show_paths_cb)
        display_layout.addWidget(self.show_backbone_cb)
        display_layout.addWidget(self.show_interfaces_cb)
        display_layout.addWidget(self.show_conflicts_cb)
        display_layout.addWidget(self.show_safety_cb)
        
        display_group.setLayout(display_layout)
        parent_layout.addWidget(display_group)


class StatusPanel(QWidget):
    """状态面板"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 系统状态
        self._create_system_status_group(layout)
        
        # 冲突管理
        self._create_conflict_status_group(layout)
        
        # ECBS统计
        self._create_ecbs_stats_group(layout)
        
        # 日志输出
        self._create_log_group(layout)
        
        layout.addStretch()
    
    def _create_system_status_group(self, parent_layout):
        """创建系统状态组"""
        system_group = QGroupBox("系统状态")
        system_layout = QGridLayout()
        
        # 车辆统计
        self.active_vehicles_label = QLabel("活跃车辆: 0")
        self.idle_vehicles_label = QLabel("空闲车辆: 0")
        self.total_vehicles_label = QLabel("总车辆: 0")
        
        system_layout.addWidget(self.active_vehicles_label, 0, 0)
        system_layout.addWidget(self.idle_vehicles_label, 0, 1)
        system_layout.addWidget(self.total_vehicles_label, 1, 0, 1, 2)
        
        # 任务统计
        self.completed_tasks_label = QLabel("完成任务: 0")
        self.pending_tasks_label = QLabel("待处理: 0")
        
        system_layout.addWidget(self.completed_tasks_label, 2, 0)
        system_layout.addWidget(self.pending_tasks_label, 2, 1)
        
        # 效率指标
        self.system_efficiency_bar = QProgressBar()
        self.system_efficiency_bar.setTextVisible(True)
        self.system_efficiency_bar.setFormat("系统效率: %p%")
        
        self.backbone_usage_bar = QProgressBar()
        self.backbone_usage_bar.setTextVisible(True)
        self.backbone_usage_bar.setFormat("骨干利用率: %p%")
        
        system_layout.addWidget(self.system_efficiency_bar, 3, 0, 1, 2)
        system_layout.addWidget(self.backbone_usage_bar, 4, 0, 1, 2)
        
        system_group.setLayout(system_layout)
        parent_layout.addWidget(system_group)
    
    def _create_conflict_status_group(self, parent_layout):
        """创建冲突状态组"""
        conflict_group = QGroupBox("冲突管理")
        conflict_layout = QVBoxLayout()
        
        self.current_conflicts_label = QLabel("当前冲突: 0")
        self.resolved_conflicts_label = QLabel("已解决: 0")
        self.prevention_rate_label = QLabel("预防率: 0%")
        
        # 冲突类型分布
        self.conflict_type_label = QLabel("冲突类型:")
        self.temporal_conflicts_label = QLabel("  时空冲突: 0")
        self.backbone_conflicts_label = QLabel("  骨干冲突: 0")
        self.safety_conflicts_label = QLabel("  安全冲突: 0")
        
        conflict_layout.addWidget(self.current_conflicts_label)
        conflict_layout.addWidget(self.resolved_conflicts_label)
        conflict_layout.addWidget(self.prevention_rate_label)
        conflict_layout.addWidget(self.conflict_type_label)
        conflict_layout.addWidget(self.temporal_conflicts_label)
        conflict_layout.addWidget(self.backbone_conflicts_label)
        conflict_layout.addWidget(self.safety_conflicts_label)
        
        conflict_group.setLayout(conflict_layout)
        parent_layout.addWidget(conflict_group)
    
    def _create_ecbs_stats_group(self, parent_layout):
        """创建ECBS统计组"""
        ecbs_group = QGroupBox("ECBS协调统计")
        ecbs_layout = QVBoxLayout()
        
        self.ecbs_coordinations_label = QLabel("协调次数: 0")
        self.ecbs_success_rate_label = QLabel("成功率: 0%")
        self.ecbs_avg_time_label = QLabel("平均耗时: 0.0s")
        self.ecbs_expansions_label = QLabel("平均扩展: 0")
        
        ecbs_layout.addWidget(self.ecbs_coordinations_label)
        ecbs_layout.addWidget(self.ecbs_success_rate_label)
        ecbs_layout.addWidget(self.ecbs_avg_time_label)
        ecbs_layout.addWidget(self.ecbs_expansions_label)
        
        ecbs_group.setLayout(ecbs_layout)
        parent_layout.addWidget(ecbs_group)
    
    def _create_log_group(self, parent_layout):
        """创建日志组"""
        log_group = QGroupBox("系统日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        parent_layout.addWidget(log_group)
    
    def add_log(self, message, level='info'):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        
        color_map = {
            'error': 'red',
            'warning': 'orange', 
            'success': 'green',
            'info': 'white'
        }
        
        color = color_map.get(level, 'white')
        
        html = f'<span style="color: gray">[{timestamp}]</span> <span style="color: {color}">{message}</span>'
        self.log_text.append(html)
        
        # 保持最新消息可见
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )


class ImprovedMineGUI(QMainWindow):
    """改进版露天矿调度系统GUI"""
    
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
        
        # 初始化界面
        self.init_ui()
        
        # 定时器
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 100ms更新一次
        
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.simulation_step)
        
        self.stats_timer = QTimer(self)
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(1000)  # 1秒更新一次
    
    def init_ui(self):
        """初始化界面"""
        self.setWindowTitle("露天矿多车协同调度系统 - 改进版")
        self.setGeometry(100, 100, 1400, 900)
        
        # 设置样式
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {COLORS['bg'].name()};
                color: {COLORS['text'].name()};
            }}
            QGroupBox {{
                font-weight: bold;
                border: 1px solid {COLORS['border'].name()};
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 6px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }}
            QPushButton {{
                background-color: {COLORS['primary'].name()};
                color: white;
                border: none;
                padding: 5px 10px;
                border-radius: 3px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {COLORS['primary'].darker(110).name()};
            }}
            QPushButton:pressed {{
                background-color: {COLORS['primary'].darker(120).name()};
            }}
            QComboBox, QSpinBox, QDoubleSpinBox {{
                background-color: {COLORS['surface'].name()};
                border: 1px solid {COLORS['border'].name()};
                border-radius: 3px;
                padding: 3px;
                color: {COLORS['text'].name()};
            }}
            QProgressBar {{
                border: 1px solid {COLORS['border'].name()};
                border-radius: 3px;
                text-align: center;
                background-color: {COLORS['surface'].name()};
            }}
            QProgressBar::chunk {{
                background-color: {COLORS['success'].name()};
                border-radius: 2px;
            }}
        """)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(8)
        
        # 左侧控制面板
        self.control_panel = ControlPanel()
        self.control_panel.setMaximumWidth(280)
        main_layout.addWidget(self.control_panel)
        
        # 中央视图
        self.scene = ImprovedMineScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setDragMode(QGraphicsView.RubberBandDrag)
        main_layout.addWidget(self.view, 1)
        
        # 右侧状态面板
        self.status_panel = StatusPanel()
        self.status_panel.setMaximumWidth(320)
        main_layout.addWidget(self.status_panel)
        
        # 连接信号
        self.connect_signals()
        
        # 初始日志
        self.status_panel.add_log("系统已启动", 'success')
        self.status_panel.add_log("改进版：修复递归问题，完善骨干路径可视化", 'info')
    
    def connect_signals(self):
        """连接信号"""
        # 文件操作
        self.control_panel.browse_btn.clicked.connect(self.browse_file)
        self.control_panel.load_btn.clicked.connect(self.load_environment)
        
        # 系统初始化
        self.control_panel.generate_backbone_btn.clicked.connect(self.generate_backbone_network)
        
        # ECBS协调
        self.control_panel.coord_selected_btn.clicked.connect(self.coordinate_selected_vehicles)
        self.control_panel.coord_all_btn.clicked.connect(self.coordinate_all_vehicles)
        
        # 冲突消解演示
        self.control_panel.create_scenario_btn.clicked.connect(self.create_conflict_scenario)
        self.control_panel.resolve_conflict_btn.clicked.connect(self.resolve_conflicts_demo)
        
        # 任务管理
        self.control_panel.assign_single_btn.clicked.connect(self.assign_single_task)
        self.control_panel.assign_batch_btn.clicked.connect(self.assign_batch_tasks)
        
        # 仿真控制
        self.control_panel.start_btn.clicked.connect(self.start_simulation)
        self.control_panel.pause_btn.clicked.connect(self.pause_simulation)
        self.control_panel.reset_btn.clicked.connect(self.reset_simulation)
        self.control_panel.speed_slider.valueChanged.connect(self.update_speed)
        
        # 显示选项
        self.control_panel.show_paths_cb.toggled.connect(
            lambda checked: setattr(self.scene, 'show_paths', checked)
        )
        self.control_panel.show_backbone_cb.toggled.connect(self.toggle_backbone_display)
        self.control_panel.show_interfaces_cb.toggled.connect(
            lambda checked: setattr(self.scene, 'show_interfaces', checked)
        )
        self.control_panel.show_conflicts_cb.toggled.connect(
            lambda checked: setattr(self.scene, 'show_conflicts', checked)
        )
        self.control_panel.show_safety_cb.toggled.connect(self.scene.toggle_safety_rects)
    
    def browse_file(self):
        """浏览文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开地图文件", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            self.map_file_path = file_path
            self.control_panel.file_label.setText(os.path.basename(file_path))
            self.status_panel.add_log(f"选择文件: {os.path.basename(file_path)}")
    
    def load_environment(self):
        """加载环境"""
        if not self.map_file_path:
            QMessageBox.warning(self, "警告", "请先选择地图文件")
            return
        
        try:
            self.status_panel.add_log("正在加载环境...")
            
            # 创建环境
            self.env = OptimizedOpenPitMineEnv()
            if not self.env.load_from_file(self.map_file_path):
                raise Exception("环境加载失败")
            
            # 设置到场景
            self.scene.set_environment(self.env)
            
            # 创建系统组件
            self.create_system_components()
            
            self.status_panel.add_log("环境加载成功", 'success')
            self.fit_view()
            
        except Exception as e:
            self.status_panel.add_log(f"加载失败: {str(e)}", 'error')
            QMessageBox.critical(self, "错误", f"加载环境失败:\n{str(e)}")
    
    def create_system_components(self):
        """创建系统组件"""
        try:
            # 创建路径规划器
            self.path_planner = EnhancedPathPlannerWithConfig(self.env)
            
            # 创建骨干网络
            self.backbone_network = OptimizedBackboneNetwork(self.env)
            self.backbone_network.set_path_planner(self.path_planner)
            
            # 创建交通管理器
            self.traffic_manager = OptimizedTrafficManagerWithECBS(
                self.env, self.backbone_network, self.path_planner
            )
            
            # 创建车辆调度器
            self.vehicle_scheduler = EnhancedVehicleScheduler(
                self.env, self.path_planner, self.backbone_network, self.traffic_manager
            )
            
            # 设置组件引用
            self.scene.set_backbone_network(self.backbone_network)
            self.scene.set_traffic_manager(self.traffic_manager)
            
            # 初始化车辆
            self.vehicle_scheduler.initialize_vehicles()
            
            # 创建默认任务模板
            if self.env.loading_points and self.env.unloading_points:
                from vehicle_scheduler import TaskPriority
                self.vehicle_scheduler.create_enhanced_mission_template(
                    "default", priority=TaskPriority.NORMAL
                )
            
            self.status_panel.add_log("系统组件创建成功", 'success')
            
        except Exception as e:
            raise Exception(f"系统组件初始化失败: {str(e)}")
    
    def generate_backbone_network(self):
        """生成骨干网络"""
        if not self.env or not self.backbone_network:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            self.status_panel.add_log("正在生成骨干网络...")
            
            quality_threshold = self.control_panel.quality_spin.value()
            
            success = self.backbone_network.generate_backbone_network(
                quality_threshold=quality_threshold
            )
            
            if success:
                # 更新可视化
                self.scene.update_backbone_visualization()
                
                # 获取网络状态
                network_status = self.backbone_network.get_network_status()
                path_count = network_status['bidirectional_paths']
                interface_count = network_status['total_interfaces']
                
                self.status_panel.add_log(
                    f"骨干网络生成成功: {path_count} 条路径, {interface_count} 个接口", 
                    'success'
                )
                
                QMessageBox.information(self, "成功", 
                    f"骨干网络生成成功\n双向路径: {path_count} 条\n接口节点: {interface_count} 个")
            else:
                self.status_panel.add_log("骨干网络生成失败", 'error')
                QMessageBox.critical(self, "错误", "骨干网络生成失败")
                
        except Exception as e:
            self.status_panel.add_log(f"生成异常: {str(e)}", 'error')
            QMessageBox.critical(self, "错误", f"生成骨干网络失败:\n{str(e)}")
    
    def create_conflict_scenario(self):
        """创建冲突场景"""
        if not self.env or not self.vehicle_scheduler:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        scenario_index = self.control_panel.scenario_combo.currentIndex()
        scenario_names = ["对向冲突", "交叉路口冲突", "骨干路径拥塞", "多车聚集"]
        scenario_name = scenario_names[scenario_index]
        
        self.status_panel.add_log(f"创建冲突场景: {scenario_name}")
        
        try:
            if scenario_index == 0:  # 对向冲突
                self._create_head_on_conflict()
            elif scenario_index == 1:  # 交叉路口冲突
                self._create_intersection_conflict()
            elif scenario_index == 2:  # 骨干路径拥塞
                self._create_backbone_congestion()
            elif scenario_index == 3:  # 多车聚集
                self._create_multi_vehicle_gathering()
            
            self.status_panel.add_log(f"冲突场景创建成功", 'success')
            
        except Exception as e:
            self.status_panel.add_log(f"创建场景失败: {str(e)}", 'error')
    
    def _create_head_on_conflict(self):
        """创建对向冲突场景"""
        # 找两个车辆，让它们相向而行
        vehicles = list(self.env.vehicles.keys())
        if len(vehicles) < 2:
            raise Exception("至少需要2个车辆")
        
        v1_id, v2_id = vehicles[0], vehicles[1]
        
        # 设置相对的位置和目标
        center_x, center_y = self.env.width / 2, self.env.height / 2
        
        # 更新车辆位置
        self.env.update_vehicle_position(v1_id, (center_x - 50, center_y, 0))
        self.env.update_vehicle_position(v2_id, (center_x + 50, center_y, math.pi))
        
        # 分配相向的任务
        from vehicle_scheduler import TaskPriority
        self.vehicle_scheduler.assign_mission_intelligently(
            v1_id, "default", TaskPriority.HIGH
        )
        self.vehicle_scheduler.assign_mission_intelligently(
            v2_id, "default", TaskPriority.HIGH
        )
    
    def _create_intersection_conflict(self):
        """创建交叉路口冲突"""
        # 实现类似的逻辑
        pass
    
    def _create_backbone_congestion(self):
        """创建骨干路径拥塞"""
        # 让多个车辆使用同一条骨干路径
        pass
    
    def _create_multi_vehicle_gathering(self):
        """创建多车聚集场景"""
        # 让多个车辆聚集到同一个点
        pass
    
    def resolve_conflicts_demo(self):
        """演示冲突消解"""
        if not self.traffic_manager:
            QMessageBox.warning(self, "警告", "交通管理器未初始化")
            return
        
        self.status_panel.add_log("开始冲突消解演示...")
        
        try:
            # 检测当前冲突
            conflicts = self.traffic_manager.detect_all_conflicts()
            
            if not conflicts:
                self.status_panel.add_log("当前没有检测到冲突", 'info')
                return
            
            self.status_panel.add_log(f"检测到 {len(conflicts)} 个冲突", 'warning')
            
            # 解决冲突
            if hasattr(self.traffic_manager, 'resolve_conflicts'):
                resolved_paths = self.traffic_manager.resolve_conflicts(conflicts)
                
                # 应用解决方案
                for vehicle_id, new_path in resolved_paths.items():
                    if vehicle_id in self.env.vehicles:
                        vehicle = self.env.vehicles[vehicle_id]
                        if hasattr(vehicle, 'path'):
                            vehicle.path = new_path
                        else:
                            vehicle['path'] = new_path
                
                self.status_panel.add_log("冲突消解完成", 'success')
                
                # 重新检测冲突
                remaining_conflicts = self.traffic_manager.detect_all_conflicts()
                self.status_panel.add_log(
                    f"剩余冲突: {len(remaining_conflicts)}", 
                    'success' if len(remaining_conflicts) == 0 else 'warning'
                )
            
        except Exception as e:
            self.status_panel.add_log(f"冲突消解失败: {str(e)}", 'error')
    
    def coordinate_selected_vehicles(self):
        """协调选中车辆"""
        # 简化实现：协调前3个空闲车辆
        vehicle_ids = []
        
        if self.vehicle_scheduler:
            for vid, state in self.vehicle_scheduler.vehicle_states.items():
                if str(state.status) == 'VehicleStatus.IDLE':
                    vehicle_ids.append(vid)
                    if len(vehicle_ids) >= 3:
                        break
        
        if len(vehicle_ids) < 2:
            QMessageBox.information(self, "提示", "需要至少2个空闲车辆才能协调")
            return
        
        self._coordinate_vehicles(vehicle_ids)
    
    def coordinate_all_vehicles(self):
        """协调所有车辆"""
        if not self.vehicle_scheduler:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        vehicle_ids = list(self.env.vehicles.keys()) if self.env else []
        
        if len(vehicle_ids) < 2:
            QMessageBox.information(self, "提示", "需要至少2个车辆才能协调")
            return
        
        self._coordinate_vehicles(vehicle_ids)
    
    def _coordinate_vehicles(self, vehicle_ids):
        """执行车辆协调"""
        try:
            self.status_panel.add_log(f"开始ECBS协调 {len(vehicle_ids)} 个车辆...")
            
            # 获取协调模式
            mode_map = {
                0: "BATCH_COORDINATION",
                1: "REAL_TIME_COORDINATION",
                2: "PERIODIC_COORDINATION"
            }
            mode_index = self.control_panel.coord_mode_combo.currentIndex()
            
            from vehicle_scheduler import CoordinationMode, TaskPriority
            coordination_mode = getattr(CoordinationMode, mode_map[mode_index])
            
            # 获取优先级
            priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT"}
            priority_index = self.control_panel.priority_combo.currentIndex()
            priority = getattr(TaskPriority, priority_map[priority_index])
            
            success = self.vehicle_scheduler.coordinate_multiple_vehicles(
                vehicle_ids, 
                coordination_mode=coordination_mode,
                priority=priority
            )
            
            if success:
                self.status_panel.add_log(f"ECBS协调成功", 'success')
                QMessageBox.information(self, "成功", f"成功协调 {len(vehicle_ids)} 个车辆")
            else:
                self.status_panel.add_log("ECBS协调失败", 'error')
                QMessageBox.warning(self, "失败", "ECBS协调失败")
                
        except Exception as e:
            self.status_panel.add_log(f"协调异常: {str(e)}", 'error')
            QMessageBox.critical(self, "错误", f"ECBS协调失败:\n{str(e)}")
    
    def assign_single_task(self):
        """分配单个任务"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 获取优先级
            priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT"}
            priority_index = self.control_panel.priority_combo.currentIndex()
            
            from vehicle_scheduler import TaskPriority
            priority = getattr(TaskPriority, priority_map[priority_index])
            
            success = self.vehicle_scheduler.assign_mission_intelligently(
                vehicle_id=None, priority=priority
            )
            
            if success:
                self.status_panel.add_log("任务分配成功", 'success')
            else:
                self.status_panel.add_log("没有可用车辆", 'warning')
                
        except Exception as e:
            self.status_panel.add_log(f"任务分配失败: {str(e)}", 'error')
    
    def assign_batch_tasks(self):
        """批量分配任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        try:
            # 获取优先级
            priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT"}
            priority_index = self.control_panel.priority_combo.currentIndex()
            
            from vehicle_scheduler import TaskPriority
            priority = getattr(TaskPriority, priority_map[priority_index])
            
            assigned_count = 0
            
            for vehicle_id in self.env.vehicles.keys():
                if self.vehicle_scheduler.assign_mission_intelligently(
                    vehicle_id=vehicle_id, priority=priority
                ):
                    assigned_count += 1
            
            self.status_panel.add_log(f"批量分配成功: {assigned_count} 个任务", 'success')
            
        except Exception as e:
            self.status_panel.add_log(f"批量分配失败: {str(e)}", 'error')
    
    def start_simulation(self):
        """开始仿真"""
        if not self.env:
            return
        
        self.is_simulating = True
        self.control_panel.start_btn.setEnabled(False)
        self.control_panel.pause_btn.setEnabled(True)
        
        # 启动定时器
        interval = max(50, int(100 / self.simulation_speed))
        self.sim_timer.start(interval)
        
        self.status_panel.add_log("仿真已开始", 'success')
    
    def pause_simulation(self):
        """暂停仿真"""
        self.is_simulating = False
        self.control_panel.start_btn.setEnabled(True)
        self.control_panel.pause_btn.setEnabled(False)
        
        self.sim_timer.stop()
        
        self.status_panel.add_log("仿真已暂停", 'warning')
    
    def reset_simulation(self):
        """重置仿真"""
        if self.is_simulating:
            self.pause_simulation()
        
        if self.env:
            self.env.reset()
        
        if self.vehicle_scheduler:
            self.vehicle_scheduler.initialize_vehicles()
        
        # 清理交通管理器
        if self.traffic_manager:
            self.traffic_manager.clear_all()
        
        self.scene.set_environment(self.env)
        
        self.simulation_time = 0
        self.control_panel.progress_bar.setValue(0)
        
        self.status_panel.add_log("仿真已重置", 'success')
    
    def update_speed(self, value):
        """更新速度"""
        self.simulation_speed = value / 50.0
        self.control_panel.speed_label.setText(f"{self.simulation_speed:.1f}x")
        
        if self.is_simulating:
            interval = max(50, int(100 / self.simulation_speed))
            self.sim_timer.start(interval)
    
    def simulation_step(self):
        """仿真步骤 - 修复版"""
        if not self.is_simulating or not self.env:
            return
        
        time_delta = 0.5 * self.simulation_speed
        self.simulation_time += time_delta
        
        # 更新环境时间
        self.env.current_time = self.simulation_time
        
        # 更新调度器
        if self.vehicle_scheduler:
            try:
                # 调用调度器更新
                self.vehicle_scheduler.update(time_delta)
                
                # ★ 关键修复：同步车辆状态到环境
                if hasattr(self.vehicle_scheduler, 'vehicle_states'):
                    for vehicle_id, vehicle_state in self.vehicle_scheduler.vehicle_states.items():
                        if vehicle_id in self.env.vehicles:
                            # 获取环境中的车辆数据
                            vehicle_data = self.env.vehicles[vehicle_id]
                            
                            # 1. 更新位置
                            new_position = vehicle_state.current_position
                            
                            # 检查环境是否有update_vehicle_position方法
                            if hasattr(self.env, 'update_vehicle_position'):
                                self.env.update_vehicle_position(vehicle_id, new_position)
                            else:
                                # 直接更新
                                if isinstance(vehicle_data, dict):
                                    vehicle_data['position'] = new_position
                                else:
                                    vehicle_data.position = new_position
                            
                            # 2. 更新状态
                            status_str = str(vehicle_state.status).split('.')[-1].lower()
                            if isinstance(vehicle_data, dict):
                                vehicle_data['status'] = status_str
                            else:
                                vehicle_data.status = status_str
                            
                            # 3. 更新路径（如果有）
                            if hasattr(vehicle_state, 'current_path') and vehicle_state.current_path:
                                if isinstance(vehicle_data, dict):
                                    vehicle_data['path'] = vehicle_state.current_path
                                else:
                                    vehicle_data.path = vehicle_state.current_path
                            
                            # 4. 更新其他属性
                            # 当前载重
                            if hasattr(vehicle_state, 'current_load'):
                                if isinstance(vehicle_data, dict):
                                    vehicle_data['current_load'] = vehicle_state.current_load
                                else:
                                    vehicle_data.current_load = vehicle_state.current_load
                            
                            # 效率分数
                            if hasattr(vehicle_state, 'calculate_efficiency_score'):
                                efficiency = vehicle_state.calculate_efficiency_score()
                                if isinstance(vehicle_data, dict):
                                    vehicle_data['efficiency_score'] = efficiency
                                else:
                                    vehicle_data.efficiency_score = efficiency
                            
                            # 骨干使用率
                            if hasattr(vehicle_state, 'backbone_usage_ratio'):
                                if isinstance(vehicle_data, dict):
                                    vehicle_data['backbone_usage_ratio'] = vehicle_state.backbone_usage_ratio
                                else:
                                    vehicle_data.backbone_usage_ratio = vehicle_state.backbone_usage_ratio
                            
                            # ECBS信息
                            if hasattr(vehicle_state, 'is_coordinated') and vehicle_state.is_coordinated:
                                ecbs_info = f"协调组: {getattr(vehicle_state, 'coordination_group', 'N/A')}"
                                if isinstance(vehicle_data, dict):
                                    vehicle_data['ecbs_info'] = ecbs_info
                                    vehicle_data['is_coordinated'] = True
                                else:
                                    vehicle_data.ecbs_info = ecbs_info
                                    vehicle_data.is_coordinated = True
                                    
            except Exception as e:
                print(f"调度器更新错误: {e}")
                self.status_panel.add_log(f"调度器更新错误: {e}", 'error')
        
        # 更新交通管理器
        if self.traffic_manager:
            try:
                self.traffic_manager.update(time_delta)
            except Exception as e:
                print(f"交通管理器更新错误: {e}")
        
        # 更新进度条
        max_time = 1800  # 30分钟
        progress = min(100, int(self.simulation_time * 100 / max_time))
        self.control_panel.progress_bar.setValue(progress)
        
        # 检查是否完成
        if progress >= 100:
            self.pause_simulation()
            self.status_panel.add_log("仿真完成！", 'success')
            QMessageBox.information(self, "完成", "仿真已完成！")
    
    def toggle_backbone_display(self, show):
        """切换骨干网络显示"""
        self.scene.show_backbone = show
        self.scene.update_backbone_visualization()
        
        # 同时更新接口节点显示
        if show and self.control_panel.show_interfaces_cb.isChecked():
            self.scene.show_interfaces = True
    
    def update_display(self):
        """更新显示（防止递归）"""
        if not self.env:
            return
        
        # 更新车辆
        self.scene.update_vehicles()
        
        # 更新冲突
        if self.scene.show_conflicts:
            self.scene.update_conflicts()
    
    def update_statistics(self):
        """更新统计"""
        if not self.vehicle_scheduler:
            return
        
        try:
            stats = self.vehicle_scheduler.get_comprehensive_stats()
            
            # 更新车辆统计
            real_time = stats.get('real_time', {})
            self.status_panel.active_vehicles_label.setText(
                f"活跃车辆: {real_time.get('active_vehicles', 0)}"
            )
            self.status_panel.idle_vehicles_label.setText(
                f"空闲车辆: {real_time.get('idle_vehicles', 0)}"
            )
            self.status_panel.total_vehicles_label.setText(
                f"总车辆: {real_time.get('total_vehicles', 0)}"
            )
            
            # 更新任务统计
            self.status_panel.completed_tasks_label.setText(
                f"完成任务: {stats.get('completed_tasks', 0)}"
            )
            
            # 更新效率
            efficiency_metrics = stats.get('efficiency_metrics', {})
            system_efficiency = efficiency_metrics.get('current_system_efficiency', 0) * 100
            self.status_panel.system_efficiency_bar.setValue(int(system_efficiency))
            
            # 更新骨干利用率
            if self.backbone_network:
                network_status = self.backbone_network.get_network_status()
                backbone_usage = network_status.get('load_balancing', {}).get('average_path_utilization', 0) * 100
                self.status_panel.backbone_usage_bar.setValue(int(backbone_usage))
            
            # 更新冲突统计
            if self.traffic_manager:
                conflicts = self.traffic_manager.detect_all_conflicts()
                self.status_panel.current_conflicts_label.setText(
                    f"当前冲突: {len(conflicts)}"
                )
                
                traffic_stats = self.traffic_manager.get_statistics()
                self.status_panel.resolved_conflicts_label.setText(
                    f"已解决: {traffic_stats.get('resolved_conflicts', 0)}"
                )
                
                # 更新ECBS统计
                self.status_panel.ecbs_coordinations_label.setText(
                    f"协调次数: {traffic_stats.get('ecbs_coordinations', 0)}"
                )
                
                # ECBS成功率
                ecbs_success_rate = traffic_stats.get('ecbs_success_rate', 0) * 100
                self.status_panel.ecbs_success_rate_label.setText(
                    f"成功率: {ecbs_success_rate:.1f}%"
                )
                
                # 平均求解时间
                avg_solve_time = traffic_stats.get('average_solve_time', 0)
                self.status_panel.ecbs_avg_time_label.setText(
                    f"平均耗时: {avg_solve_time:.2f}s"
                )
                
        except Exception as e:
            print(f"更新统计失败: {e}")
    
    def fit_view(self):
        """适应视图"""
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.is_simulating:
            reply = QMessageBox.question(
                self, '确认退出',
                '仿真正在运行，确定要退出吗？',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.No:
                event.ignore()
                return
        
        # 停止定时器
        self.update_timer.stop()
        self.sim_timer.stop()
        self.stats_timer.stop()
        
        # 清理冲突标记
        for marker in self.scene.conflict_markers.values():
            marker.cleanup()
        
        # 关闭组件
        try:
            if self.vehicle_scheduler:
                self.vehicle_scheduler.shutdown()
            if self.traffic_manager:
                self.traffic_manager.shutdown()
            if self.path_planner:
                self.path_planner.shutdown()
        except:
            pass
        
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("露天矿多车协同调度系统")
    
    # 设置全局字体
    app.setStyleSheet("""
        QApplication {
            font-family: "Microsoft YaHei", "SimHei", Arial, sans-serif;
            font-size: 9pt;
        }
    """)
    
    try:
        window = ImprovedMineGUI()
        window.show()
        
        print("🚀 改进版露天矿调度系统启动成功")
        print("✨ 主要改进:")
        print("   - 修复递归深度问题")
        print("   - 完善骨干路径可视化（显示接口节点）")  
        print("   - 增强冲突显示效果")
        print("   - 添加冲突消解演示功能")
        
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"❌ 启动失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)