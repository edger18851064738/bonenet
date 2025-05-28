#!/usr/bin/env python3
"""
integrated_gui.py - 整合优化组件的专业GUI系统
整合了最新的backbone、traffic_manager和vehicle_scheduler组件
"""

import sys
import os
import math
import time
import json
from typing import Dict, List, Tuple, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QPropertyAnimation, QEasingCurve, QPointF, pyqtProperty
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QGroupBox, QGridLayout, QTableWidget, QTableWidgetItem,
    QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QDockWidget,
    QGraphicsRectItem, QGraphicsPathItem, QTabWidget, QFrame,
    QSlider, QCheckBox, QLCDNumber, QScrollArea, QTreeWidget, QTreeWidgetItem,
    QListWidget, QGraphicsItemGroup, QGraphicsPolygonItem, QGraphicsLineItem,
    QGraphicsTextItem, QAction, QToolBar, QMenuBar, QMenu, QStatusBar, QDial
)
from PyQt5.QtGui import (
    QPen, QBrush, QColor, QPainter, QPainterPath, QFont, QPixmap, QIcon,
    QLinearGradient, QRadialGradient, QPolygonF, QTransform
)

# 导入优化组件
try:
    from optimized_backbone_network import OptimizedBackboneNetwork
    from optimized_planner_config import EnhancedPathPlannerWithConfig
    from traffic_manager import OptimizedTrafficManager
    from vehicle_scheduler import EnhancedVehicleScheduler
    OPTIMIZED_COMPONENTS_AVAILABLE = True
    print("✅ 优化组件加载成功")
except ImportError as e:
    print(f"⚠️ 优化组件不可用: {e}")
    # 回退到基础组件
    try:
        from path_planner import SimplifiedPathPlanner as EnhancedPathPlannerWithConfig
        from traffic_manager import OptimizedTrafficManager
        from vehicle_scheduler import SimplifiedVehicleScheduler as EnhancedVehicleScheduler
        OPTIMIZED_COMPONENTS_AVAILABLE = False
    except ImportError:
        print("❌ 基础组件也不可用")
        sys.exit(1)

from environment import OptimizedOpenPitMineEnv


# 专业配色方案
PROFESSIONAL_COLORS = {
    'background': QColor(45, 47, 57),
    'surface': QColor(55, 58, 71),
    'primary': QColor(66, 135, 245),
    'secondary': QColor(156, 163, 175),
    'success': QColor(16, 185, 129),
    'warning': QColor(245, 158, 11),
    'error': QColor(239, 68, 68),
    'text': QColor(229, 231, 235),
    'text_muted': QColor(156, 163, 175),
    'border': QColor(75, 85, 99)
}

# 车辆状态专业配色
VEHICLE_STATUS_COLORS = {
    'idle': QColor(156, 163, 175),      # 灰色
    'loading': QColor(16, 185, 129),    # 绿色
    'unloading': QColor(245, 158, 11),  # 橙色
    'moving': QColor(66, 135, 245),     # 蓝色
    'waiting': QColor(168, 85, 247),    # 紫色
    'planning': QColor(244, 63, 94),    # 粉色
    'maintenance': QColor(239, 68, 68)  # 红色
}


class VehicleGraphicsItem(QGraphicsItemGroup):
    """专业的车辆图形项 - 支持新状态"""
    
    def __init__(self, vehicle_id, vehicle_data, parent=None):
        super().__init__(parent)
        self.vehicle_id = vehicle_id
        self.vehicle_data = vehicle_data
        self.position = vehicle_data.get('position', (0, 0, 0))
        
        # 创建车辆组件
        self.vehicle_body = QGraphicsPolygonItem(self)
        self.status_indicator = QGraphicsEllipseItem(self)
        self.load_indicator = QGraphicsRectItem(self)
        self.direction_line = QGraphicsLineItem(self)
        self.efficiency_ring = QGraphicsEllipseItem(self)  # 新增：效率环
        
        # 标签
        self.vehicle_label = QGraphicsTextItem(str(vehicle_id), self)
        self.vehicle_label.setDefaultTextColor(PROFESSIONAL_COLORS['text'])
        self.vehicle_label.setFont(QFont("Arial", 2, QFont.Bold))
        
        # 状态文本
        self.status_text = QGraphicsTextItem("", self)
        self.status_text.setDefaultTextColor(PROFESSIONAL_COLORS['text'])
        self.status_text.setFont(QFont("Arial", 1))
        
        self.setZValue(15)
        self.update_appearance()
        self.update_position()
    
    def update_appearance(self):
        """更新车辆外观 - 支持新状态"""
        status = self.vehicle_data.get('status', 'idle')
        color = VEHICLE_STATUS_COLORS.get(status, VEHICLE_STATUS_COLORS['idle'])
        
        # 车辆主体 - 专业风格
        self.vehicle_body.setBrush(QBrush(color))
        self.vehicle_body.setPen(QPen(color.darker(150), 1))
        
        # 状态指示器
        self.status_indicator.setBrush(QBrush(color.lighter(130)))
        self.status_indicator.setPen(QPen(color.darker(150), 1))
        
        # 负载指示器
        current_load = self.vehicle_data.get('current_load', 0)
        max_load = self.vehicle_data.get('max_load', 100)
        load_ratio = current_load / max_load if max_load > 0 else 0
        load_color = QColor.fromHsv(int(120 * (1 - load_ratio)), 200, 200)
        self.load_indicator.setBrush(QBrush(load_color))
        self.load_indicator.setPen(QPen(Qt.NoPen))
        
        # 效率环 - 新功能
        efficiency = self.vehicle_data.get('efficiency_score', 0.5)
        if efficiency > 0.8:
            efficiency_color = PROFESSIONAL_COLORS['success']
        elif efficiency > 0.6:
            efficiency_color = PROFESSIONAL_COLORS['primary']
        elif efficiency > 0.4:
            efficiency_color = PROFESSIONAL_COLORS['warning']
        else:
            efficiency_color = PROFESSIONAL_COLORS['error']
        
        self.efficiency_ring.setBrush(QBrush(Qt.NoBrush))
        self.efficiency_ring.setPen(QPen(efficiency_color, 1.5))
        
        # 方向线
        self.direction_line.setPen(QPen(PROFESSIONAL_COLORS['text'], 2))
        
        # 状态文本
        self.status_text.setPlainText(status.upper())
    
    def update_position(self):
        """更新车辆位置"""
        if not self.position or len(self.position) < 3:
            return
        
        x, y, theta = self.position
        
        # 车辆形状 - 简洁的卡车轮廓
        length, width = 6.0, 3.0
        half_length, half_width = length/2, width/2
        
        # 创建卡车形状
        truck_points = [
            QPointF(half_length, half_width * 0.8),      # 右前
            QPointF(half_length * 0.8, half_width),      # 右前角
            QPointF(-half_length * 0.8, half_width),     # 右后
            QPointF(-half_length, half_width * 0.6),     # 右后角
            QPointF(-half_length, -half_width * 0.6),    # 左后角
            QPointF(-half_length * 0.8, -half_width),    # 左后
            QPointF(half_length * 0.8, -half_width),     # 左前角
            QPointF(half_length, -half_width * 0.8)      # 左前
        ]
        
        # 应用旋转和平移
        transform = QTransform()
        transform.translate(x, y)
        transform.rotate(math.degrees(theta))
        
        polygon = QPolygonF()
        for point in truck_points:
            polygon.append(transform.map(point))
        
        self.vehicle_body.setPolygon(polygon)
        
        # 更新其他组件位置
        self.vehicle_label.setPos(x - 8, y - 12)
        self.status_text.setPos(x - 6, y + 8)
        self.status_indicator.setRect(x - 1, y - 1, 2, 2)
        
        # 负载指示器
        current_load = self.vehicle_data.get('current_load', 0)
        max_load = self.vehicle_data.get('max_load', 100)
        load_ratio = current_load / max_load if max_load > 0 else 0
        self.load_indicator.setRect(x - half_length, y - width/2 - 3, length * load_ratio, 2)
        
        # 效率环
        self.efficiency_ring.setRect(x - 1, y - 1, 4, 4)
        
        # 方向指示线
        line_length = 4
        end_x = x + line_length * math.cos(theta)
        end_y = y + line_length * math.sin(theta)
        self.direction_line.setLine(x, y, end_x, end_y)
    
    def update_data(self, vehicle_data):
        """更新车辆数据"""
        self.vehicle_data = vehicle_data
        new_position = vehicle_data.get('position', self.position)
        
        self.position = new_position
        self.update_position()
        self.update_appearance()


class BackboneNetworkView(QGraphicsItemGroup):
    """专业的骨干网络可视化 - 支持负载显示"""
    
    def __init__(self, backbone_network, parent=None):
        super().__init__(parent)
        self.backbone_network = backbone_network
        self.setZValue(3)
        self.path_items = {}
        self.interface_items = {}
        self.load_indicators = {}  # 新增：负载指示器
        self.update_visualization()
    
    def update_visualization(self):
        """更新骨干网络可视化"""
        # 清除现有项
        for item in self.childItems():
            self.removeFromGroup(item)
        
        self.path_items.clear()
        self.interface_items.clear()
        self.load_indicators.clear()
        
        if not self.backbone_network:
            return
        
        # 绘制双向路径
        if hasattr(self.backbone_network, 'bidirectional_paths'):
            self._draw_bidirectional_paths()
        elif hasattr(self.backbone_network, 'backbone_paths'):
            self._draw_legacy_paths()
        
        # 绘制接口
        if hasattr(self.backbone_network, 'backbone_interfaces'):
            self._draw_interfaces()
    
    def _draw_bidirectional_paths(self):
        """绘制双向路径 - 包含负载显示"""
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            forward_path = path_data.forward_path
            if len(forward_path) < 2:
                continue
            
            # 创建路径
            painter_path = QPainterPath()
            painter_path.moveTo(forward_path[0][0], forward_path[0][1])
            
            for point in forward_path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            path_item = QGraphicsPathItem(painter_path)
            
            # 根据质量和负载设置颜色
            quality = path_data.quality
            load_factor = path_data.get_load_factor()
            
            # 基础颜色根据质量
            if quality > 0.8:
                base_color = PROFESSIONAL_COLORS['success']
            elif quality > 0.6:
                base_color = PROFESSIONAL_COLORS['primary']
            else:
                base_color = PROFESSIONAL_COLORS['warning']
            
            # 根据负载调整颜色强度
            if load_factor > 0.8:
                # 高负载时使用更强的红色调
                path_color = QColor(239, 68, 68)  # 红色
                line_width = 3.0
            elif load_factor > 0.5:
                # 中等负载时使用橙色调
                path_color = QColor(245, 158, 11)  # 橙色
                line_width = 2.5
            else:
                # 低负载时使用基础颜色
                path_color = base_color
                line_width = 2.0
            
            # 专业线条样式
            pen = QPen(path_color, line_width)
            pen.setCapStyle(Qt.RoundCap)
            path_item.setPen(pen)
            
            self.addToGroup(path_item)
            self.path_items[path_id] = path_item
            
            # 添加负载文本指示器
            if forward_path:
                mid_point = forward_path[len(forward_path)//2]
                load_text = QGraphicsTextItem(f"{load_factor:.1%}")
                load_text.setPos(mid_point[0], mid_point[1] - 5)
                load_text.setDefaultTextColor(path_color)
                load_text.setFont(QFont("Arial", 1, QFont.Bold))
                self.addToGroup(load_text)
                self.load_indicators[path_id] = load_text
    
    def _draw_legacy_paths(self):
        """绘制传统路径"""
        for path_id, path_data in self.backbone_network.backbone_paths.items():
            path = path_data.get('path', [])
            if len(path) < 2:
                continue
            
            painter_path = QPainterPath()
            painter_path.moveTo(path[0][0], path[0][1])
            
            for point in path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            path_item = QGraphicsPathItem(painter_path)
            pen = QPen(PROFESSIONAL_COLORS['primary'], 2)
            path_item.setPen(pen)
            
            self.addToGroup(path_item)
            self.path_items[path_id] = path_item
    
    def _draw_interfaces(self):
        """绘制接口点 - 包含预留状态"""
        for interface_id, interface in self.backbone_network.backbone_interfaces.items():
            position = interface.get('position', (0, 0))
            if len(position) < 2:
                continue
            
            x, y = position[0], position[1]
            
            # 接口圆圈
            interface_item = QGraphicsEllipseItem(x-1.5, y-1.5, 3, 3)
            
            # 根据预留状态设置颜色
            reservation_count = interface.get('reservation_count', 0)
            if reservation_count > 0:
                color = PROFESSIONAL_COLORS['error']  # 已预留
            elif interface.get('is_occupied', False):
                color = PROFESSIONAL_COLORS['warning']  # 被占用
            else:
                color = PROFESSIONAL_COLORS['secondary']  # 可用
            
            interface_item.setBrush(QBrush(color))
            interface_item.setPen(QPen(color.darker(150), 1))
            
            self.addToGroup(interface_item)
            self.interface_items[interface_id] = interface_item


class SystemEfficiencyWidget(QWidget):
    """系统效率监控组件"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scheduler = None
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 标题
        title = QLabel("系统效率监控")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: rgb(229, 231, 235);
                padding: 8px;
                background-color: rgb(16, 185, 129);
                border-radius: 4px;
            }
        """)
        layout.addWidget(title)
        
        # 系统效率指标
        efficiency_group = QGroupBox("整体效率")
        efficiency_layout = QVBoxLayout()
        
        self.system_efficiency_bar = self._create_professional_progress("系统效率")
        efficiency_layout.addWidget(self.system_efficiency_bar[0])
        efficiency_layout.addWidget(self.system_efficiency_bar[1])
        
        self.target_efficiency_label = QLabel("目标效率: 85%")
        efficiency_layout.addWidget(self.target_efficiency_label)
        
        efficiency_group.setLayout(efficiency_layout)
        layout.addWidget(efficiency_group)
        
        # 负载分布
        load_group = QGroupBox("负载分布")
        load_layout = QGridLayout()
        
        self.vehicle_utilization_bar = self._create_professional_progress("车辆利用率")
        load_layout.addWidget(self.vehicle_utilization_bar[0], 0, 0)
        load_layout.addWidget(self.vehicle_utilization_bar[1], 1, 0)
        
        self.backbone_utilization_bar = self._create_professional_progress("骨干利用率")
        load_layout.addWidget(self.backbone_utilization_bar[0], 0, 1)
        load_layout.addWidget(self.backbone_utilization_bar[1], 1, 1)
        
        load_group.setLayout(load_layout)
        layout.addWidget(load_group)
        
        # 优化信息
        optimization_group = QGroupBox("优化状态")
        optimization_layout = QVBoxLayout()
        
        self.optimization_cycles_label = QLabel("优化周期: 0")
        self.last_optimization_label = QLabel("上次优化: 未执行")
        self.efficiency_improvement_label = QLabel("效率提升: +0%")
        
        optimization_layout.addWidget(self.optimization_cycles_label)
        optimization_layout.addWidget(self.last_optimization_label)
        optimization_layout.addWidget(self.efficiency_improvement_label)
        
        optimization_group.setLayout(optimization_layout)
        layout.addWidget(optimization_group)
        
        # 手动优化按钮
        self.manual_optimize_btn = QPushButton("手动优化")
        self.manual_optimize_btn.clicked.connect(self.trigger_manual_optimization)
        layout.addWidget(self.manual_optimize_btn)
        
        layout.addStretch()
    
    def _create_professional_progress(self, label_text):
        """创建专业进度条"""
        label = QLabel(label_text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold; color: rgb(156, 163, 175);")
        
        progress = QProgressBar()
        progress.setMinimumHeight(20)
        progress.setTextVisible(True)
        progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid rgb(75, 85, 99);
                border-radius: 4px;
                text-align: center;
                background-color: rgb(45, 47, 57);
                color: rgb(229, 231, 235);
            }
            QProgressBar::chunk {
                background-color: rgb(16, 185, 129);
                border-radius: 3px;
            }
        """)
        
        return (label, progress)
    
    def set_scheduler(self, scheduler):
        """设置调度器"""
        self.scheduler = scheduler
    
    def update_efficiency_display(self):
        """更新效率显示"""
        if not self.scheduler:
            return
        
        # 获取效率报告
        efficiency_report = self.scheduler.get_efficiency_report()
        
        # 更新系统效率
        system_efficiency = efficiency_report.get('system_efficiency', 0) * 100
        self.system_efficiency_bar[1].setValue(int(system_efficiency))
        
        # 更新系统指标
        system_metrics = efficiency_report.get('system_metrics', {})
        vehicle_utilization = system_metrics.get('vehicle_utilization', 0) * 100
        backbone_utilization = system_metrics.get('backbone_utilization', 0) * 100
        
        self.vehicle_utilization_bar[1].setValue(int(vehicle_utilization))
        self.backbone_utilization_bar[1].setValue(int(backbone_utilization))
        
        # 更新优化信息
        optimization_history = efficiency_report.get('optimization_history', [])
        if optimization_history:
            last_optimization = optimization_history[-1]
            improvement = last_optimization.get('improvement', 0) * 100
            
            self.efficiency_improvement_label.setText(f"效率提升: {improvement:+.1f}%")
            
            # 格式化时间
            import datetime
            last_time = datetime.datetime.fromtimestamp(last_optimization['timestamp'])
            time_str = last_time.strftime("%H:%M:%S")
            self.last_optimization_label.setText(f"上次优化: {time_str}")
        
        # 更新优化周期数
        stats = self.scheduler.get_comprehensive_stats()
        cycles = stats.get('optimization_cycles', 0)
        self.optimization_cycles_label.setText(f"优化周期: {cycles}")
    
    def trigger_manual_optimization(self):
        """触发手动优化"""
        if self.scheduler and hasattr(self.scheduler, 'efficiency_optimizer'):
            try:
                result = self.scheduler.efficiency_optimizer.optimize_system()
                QMessageBox.information(self, "优化完成", 
                    f"系统优化完成\n"
                    f"车辆重平衡: {result.get('vehicle_rebalancing', 0)}\n"
                    f"任务重分配: {result.get('task_reassignments', 0)}\n"
                    f"效率提升: {result.get('efficiency_improvement', 0):.2%}")
            except Exception as e:
                QMessageBox.warning(self, "优化失败", f"系统优化失败: {str(e)}")


class RealTimeStatusWidget(QWidget):
    """实时状态监控组件 - 增强版"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scheduler = None
        self.traffic_manager = None
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 标题
        title = QLabel("实时监控")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: rgb(229, 231, 235);
                padding: 8px;
                background-color: rgb(66, 135, 245);
                border-radius: 4px;
            }
        """)
        layout.addWidget(title)
        
        # 车辆状态
        vehicle_group = QGroupBox("车辆状态")
        vehicle_layout = QGridLayout()
        
        self.active_vehicles_lcd = self._create_professional_lcd("活跃")
        vehicle_layout.addWidget(self.active_vehicles_lcd[0], 0, 0)
        vehicle_layout.addWidget(self.active_vehicles_lcd[1], 1, 0)
        
        self.idle_vehicles_lcd = self._create_professional_lcd("空闲")
        vehicle_layout.addWidget(self.idle_vehicles_lcd[0], 0, 1)
        vehicle_layout.addWidget(self.idle_vehicles_lcd[1], 1, 1)
        
        self.planning_vehicles_lcd = self._create_professional_lcd("规划中")  # 新状态
        vehicle_layout.addWidget(self.planning_vehicles_lcd[0], 2, 0)
        vehicle_layout.addWidget(self.planning_vehicles_lcd[1], 3, 0)
        
        self.waiting_vehicles_lcd = self._create_professional_lcd("等待中")  # 新状态
        vehicle_layout.addWidget(self.waiting_vehicles_lcd[0], 2, 1)
        vehicle_layout.addWidget(self.waiting_vehicles_lcd[1], 3, 1)
        
        vehicle_group.setLayout(vehicle_layout)
        layout.addWidget(vehicle_group)
        
        # 任务状态
        task_group = QGroupBox("任务状态")
        task_layout = QGridLayout()
        
        self.completed_tasks_lcd = self._create_professional_lcd("完成")
        task_layout.addWidget(self.completed_tasks_lcd[0], 0, 0)
        task_layout.addWidget(self.completed_tasks_lcd[1], 1, 0)
        
        self.failed_tasks_lcd = self._create_professional_lcd("失败")
        task_layout.addWidget(self.failed_tasks_lcd[0], 0, 1)
        task_layout.addWidget(self.failed_tasks_lcd[1], 1, 1)
        
        self.pending_tasks_lcd = self._create_professional_lcd("待处理")  # 新状态
        task_layout.addWidget(self.pending_tasks_lcd[0], 2, 0)
        task_layout.addWidget(self.pending_tasks_lcd[1], 3, 0)
        
        task_group.setLayout(task_layout)
        layout.addWidget(task_group)
        
        # 性能指标
        perf_group = QGroupBox("性能指标")
        perf_layout = QVBoxLayout()
        
        self.efficiency_bar = self._create_professional_progress("系统效率")
        perf_layout.addWidget(self.efficiency_bar[0])
        perf_layout.addWidget(self.efficiency_bar[1])
        
        self.backbone_usage_bar = self._create_professional_progress("骨干利用率")
        perf_layout.addWidget(self.backbone_usage_bar[0])
        perf_layout.addWidget(self.backbone_usage_bar[1])
        
        perf_group.setLayout(perf_layout)
        layout.addWidget(perf_group)
        
        # 交通管理
        traffic_group = QGroupBox("交通管理")
        traffic_layout = QVBoxLayout()
        
        self.conflicts_label = QLabel("当前冲突: 0")
        self.resolved_label = QLabel("已解决: 0")
        self.parking_maneuvers_label = QLabel("停车避让: 0")  # 新功能
        self.backbone_switches_label = QLabel("路径切换: 0")  # 新功能
        
        traffic_layout.addWidget(self.conflicts_label)
        traffic_layout.addWidget(self.resolved_label)
        traffic_layout.addWidget(self.parking_maneuvers_label)
        traffic_layout.addWidget(self.backbone_switches_label)
        
        traffic_group.setLayout(traffic_layout)
        layout.addWidget(traffic_group)
        
        layout.addStretch()
    
    def _create_professional_lcd(self, label_text):
        """创建专业LCD显示器"""
        label = QLabel(label_text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold; color: rgb(156, 163, 175);")
        
        lcd = QLCDNumber()
        lcd.setSegmentStyle(QLCDNumber.Flat)
        lcd.setDigitCount(3)
        lcd.setMinimumHeight(35)
        lcd.setStyleSheet("""
            QLCDNumber {
                background-color: rgb(45, 47, 57);
                color: rgb(66, 135, 245);
                border: 1px solid rgb(75, 85, 99);
                border-radius: 4px;
            }
        """)
        
        return (label, lcd)
    
    def _create_professional_progress(self, label_text):
        """创建专业进度条"""
        label = QLabel(label_text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold; color: rgb(156, 163, 175);")
        
        progress = QProgressBar()
        progress.setMinimumHeight(20)
        progress.setTextVisible(True)
        progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid rgb(75, 85, 99);
                border-radius: 4px;
                text-align: center;
                background-color: rgb(45, 47, 57);
                color: rgb(229, 231, 235);
            }
            QProgressBar::chunk {
                background-color: rgb(16, 185, 129);
                border-radius: 3px;
            }
        """)
        
        return (label, progress)
    
    def set_scheduler(self, scheduler):
        """设置调度器"""
        self.scheduler = scheduler
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.traffic_manager = traffic_manager
    
    def update_stats(self):
        """更新统计显示"""
        if not self.scheduler:
            return
        
        stats = self.scheduler.get_comprehensive_stats()
        
        # 更新车辆统计
        real_time = stats.get('real_time', {})
        self.active_vehicles_lcd[1].display(real_time.get('active_vehicles', 0))
        self.idle_vehicles_lcd[1].display(real_time.get('idle_vehicles', 0))
        
        # 新增状态统计
        if hasattr(self.scheduler, 'vehicle_states'):
            planning_count = len([v for v in self.scheduler.vehicle_states.values() 
                                if hasattr(v, 'status') and str(v.status) == 'VehicleStatus.PLANNING'])
            waiting_count = len([v for v in self.scheduler.vehicle_states.values() 
                               if hasattr(v, 'status') and str(v.status) == 'VehicleStatus.WAITING'])
            
            self.planning_vehicles_lcd[1].display(planning_count)
            self.waiting_vehicles_lcd[1].display(waiting_count)
        
        # 更新任务统计
        self.completed_tasks_lcd[1].display(stats.get('completed_tasks', 0))
        self.failed_tasks_lcd[1].display(stats.get('failed_tasks', 0))
        
        # 待处理任务统计
        if hasattr(self.scheduler, 'tasks'):
            pending_count = len([t for t in self.scheduler.tasks.values() 
                               if hasattr(t, 'status') and str(t.status) == 'TaskStatus.PENDING'])
            self.pending_tasks_lcd[1].display(pending_count)
        
        # 更新性能指标
        efficiency_metrics = stats.get('efficiency_metrics', {})
        current_efficiency = efficiency_metrics.get('current_system_efficiency', 0) * 100
        backbone_rate = efficiency_metrics.get('backbone_utilization_rate', 0) * 100
        
        self.efficiency_bar[1].setValue(int(current_efficiency))
        self.backbone_usage_bar[1].setValue(int(backbone_rate))
        
        # 更新交通统计
        if self.traffic_manager:
            traffic_stats = self.traffic_manager.get_statistics()
            
            # 检测当前冲突
            conflicts = self.traffic_manager.detect_all_conflicts()
            self.conflicts_label.setText(f"当前冲突: {len(conflicts)}")
            
            # 解决统计
            resolved_count = traffic_stats.get('resolved_conflicts', 0)
            self.resolved_label.setText(f"已解决: {resolved_count}")
            
            # 新功能统计
            parking_count = traffic_stats.get('parking_maneuvers', 0)
            self.parking_maneuvers_label.setText(f"停车避让: {parking_count}")
            
            backbone_switches = traffic_stats.get('backbone_switches', 0)
            self.backbone_switches_label.setText(f"路径切换: {backbone_switches}")


class ProfessionalMineScene(QGraphicsScene):
    """专业的矿场场景 - 增强版"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.env = None
        self.backbone_network = None
        self.traffic_manager = None
        
        # 显示选项
        self.show_options = {
            'trajectories': True,
            'backbone': True,
            'interfaces': True,
            'conflicts': True,
            'efficiency_indicators': True,  # 新增：效率指示器
            'load_indicators': True,        # 新增：负载指示器
            'grid': False,
            'labels': True
        }
        
        # 图形项容器
        self.vehicle_items = {}
        self.path_items = {}
        self.backbone_view = None
        self.conflict_items = {}
        self.efficiency_items = {}  # 新增：效率指示器项
        
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
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.backbone_network = backbone_network
        self.update_backbone_visualization()
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.traffic_manager = traffic_manager
    
    def _reset_items(self):
        """重置图形项"""
        self.vehicle_items = {}
        self.path_items = {}
        self.backbone_view = None
        self.conflict_items = {}
        self.efficiency_items = {}
    
    def draw_environment(self):
        """绘制环境"""
        if not self.env:
            return
        
        # 专业背景
        background = QGraphicsRectItem(0, 0, self.env.width, self.env.height)
        background.setBrush(QBrush(PROFESSIONAL_COLORS['background']))
        background.setPen(QPen(Qt.NoPen))
        background.setZValue(-100)
        self.addItem(background)
        
        # 网格线（如果启用）
        if self.show_options['grid']:
            self._draw_professional_grid()
        
        # 障碍物
        self._draw_obstacles()
        
        # 特殊点
        self._draw_special_points()
        
        # 车辆
        self.draw_vehicles()
    
    def _draw_professional_grid(self):
        """绘制专业网格"""
        grid_size = 20
        pen = QPen(PROFESSIONAL_COLORS['border'])
        pen.setStyle(Qt.DotLine)
        
        # 垂直线
        for x in range(0, self.env.width + 1, grid_size):
            line = QGraphicsLineItem(x, 0, x, self.env.height)
            line.setPen(pen)
            line.setZValue(-90)
            self.addItem(line)
        
        # 水平线
        for y in range(0, self.env.height + 1, grid_size):
            line = QGraphicsLineItem(0, y, self.env.width, y)
            line.setPen(pen)
            line.setZValue(-90)
            self.addItem(line)
    
    def _draw_obstacles(self):
        """绘制障碍物"""
        if not hasattr(self.env, 'obstacle_points'):
            return
            
        for x, y in self.env.obstacle_points:
            rect = QGraphicsRectItem(x, y, 1, 1)
            rect.setBrush(QBrush(PROFESSIONAL_COLORS['surface']))
            rect.setPen(QPen(PROFESSIONAL_COLORS['border'], 0.1))
            rect.setZValue(-50)
            self.addItem(rect)
    
    def _draw_special_points(self):
        """绘制特殊点"""
        # 装载点
        for i, point in enumerate(self.env.loading_points):
            self._draw_loading_point(i, point)
        
        # 卸载点
        for i, point in enumerate(self.env.unloading_points):
            self._draw_unloading_point(i, point)
        
        # 停车区
        if hasattr(self.env, 'parking_areas'):
            for i, point in enumerate(self.env.parking_areas):
                self._draw_parking_area(i, point)
    
    def _draw_loading_point(self, index, point):
        """绘制装载点"""
        x, y = point[0], point[1]
        
        # 装载区域
        area = QGraphicsEllipseItem(x-3, y-3, 6, 6)
        area.setBrush(QBrush(QColor(16, 185, 129, 100)))
        area.setPen(QPen(PROFESSIONAL_COLORS['success'], 2))
        area.setZValue(-20)
        self.addItem(area)
        
        # 中心标记
        center = QGraphicsEllipseItem(x-1, y-1, 2, 2)
        center.setBrush(QBrush(PROFESSIONAL_COLORS['success']))
        center.setPen(QPen(Qt.NoPen))
        center.setZValue(-10)
        self.addItem(center)
        
        # 标签
        if self.show_options['labels']:
            text = QGraphicsTextItem(f"装载点 {index+1}")
            text.setPos(x-15, y-25)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['success'])
            text.setFont(QFont("Arial", 2, QFont.Bold))
            text.setZValue(-10)
            self.addItem(text)
    
    def _draw_unloading_point(self, index, point):
        """绘制卸载点"""
        x, y = point[0], point[1]
        
        # 卸载区域
        area = QGraphicsRectItem(x-3, y-3, 6, 6)
        area.setBrush(QBrush(QColor(245, 158, 11, 100)))
        area.setPen(QPen(PROFESSIONAL_COLORS['warning'], 2))
        area.setZValue(-20)
        self.addItem(area)
        
        # 中心标记
        center = QGraphicsRectItem(x-1, y-1, 2, 2)
        center.setBrush(QBrush(PROFESSIONAL_COLORS['warning']))
        center.setPen(QPen(Qt.NoPen))
        center.setZValue(-10)
        self.addItem(center)
        
        # 标签
        if self.show_options['labels']:
            text = QGraphicsTextItem(f"卸载点 {index+1}")
            text.setPos(x-15, y-25)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['warning'])
            text.setFont(QFont("Arial", 2, QFont.Bold))
            text.setZValue(-10)
            self.addItem(text)
    
    def _draw_parking_area(self, index, point):
        """绘制停车区"""
        x, y = point[0], point[1]
        
        # 停车区域
        area = QGraphicsEllipseItem(x-2, y-2, 4, 4)
        area.setBrush(QBrush(QColor(66, 135, 245, 100)))
        area.setPen(QPen(PROFESSIONAL_COLORS['primary'], 2, Qt.DashLine))
        area.setZValue(-20)
        self.addItem(area)
        
        # 中心标记
        diamond = QPolygonF([
            QPointF(x, y-2),
            QPointF(x+2, y),
            QPointF(x, y+2),
            QPointF(x-2, y)
        ])
        center = QGraphicsPolygonItem(diamond)
        center.setBrush(QBrush(PROFESSIONAL_COLORS['primary']))
        center.setPen(QPen(Qt.NoPen))
        center.setZValue(-10)
        self.addItem(center)
        
        # 标签
        if self.show_options['labels']:
            text = QGraphicsTextItem(f"停车区 {index+1}")
            text.setPos(x-15, y-30)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['primary'])
            text.setFont(QFont("Arial", 2, QFont.Bold))
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
        
        for item in self.path_items.values():
            self.removeItem(item)
        self.path_items.clear()
        
        # 添加车辆
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            # 添加效率分数到车辆数据
            if hasattr(vehicle_data, 'efficiency_score'):
                pass  # 已有效率分数
            else:
                # 从调度器获取效率分数
                vehicle_data['efficiency_score'] = 0.5  # 默认值
            
            vehicle_item = VehicleGraphicsItem(vehicle_id, vehicle_data)
            self.addItem(vehicle_item)
            self.vehicle_items[vehicle_id] = vehicle_item
            
            # 绘制路径
            if (self.show_options['trajectories'] and 
                hasattr(vehicle_data, 'path') and vehicle_data.path):
                self._draw_vehicle_path(vehicle_id, vehicle_data)
    
    def _draw_vehicle_path(self, vehicle_id, vehicle_data):
        """绘制车辆路径"""
        path = vehicle_data.path
        
        if not path or len(path) < 2:
            return
        
        painter_path = QPainterPath()
        painter_path.moveTo(path[0][0], path[0][1])
        
        for point in path[1:]:
            painter_path.lineTo(point[0], point[1])
        
        path_item = QGraphicsPathItem(painter_path)
        
        # 根据路径结构设置样式
        path_structure = getattr(vehicle_data, 'path_structure', {})
        backbone_utilization = path_structure.get('backbone_utilization', 0)
        
        if backbone_utilization > 0.5:
            # 高骨干利用率 - 实线
            pen = QPen(PROFESSIONAL_COLORS['primary'], 1.5)
        elif backbone_utilization > 0:
            # 部分骨干利用率 - 虚线
            pen = QPen(PROFESSIONAL_COLORS['secondary'], 1.2)
            pen.setStyle(Qt.DashLine)
        else:
            # 无骨干利用率 - 点线
            pen = QPen(PROFESSIONAL_COLORS['text_muted'], 1.0)
            pen.setStyle(Qt.DotLine)
        
        pen.setCapStyle(Qt.RoundCap)
        path_item.setPen(pen)
        path_item.setZValue(5)
        
        self.addItem(path_item)
        self.path_items[vehicle_id] = path_item
    
    def update_vehicles(self):
        """更新车辆显示"""
        if not self.env:
            return
        
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            if vehicle_id in self.vehicle_items:
                self.vehicle_items[vehicle_id].update_data(vehicle_data)
            else:
                # 添加新车辆
                vehicle_item = VehicleGraphicsItem(vehicle_id, vehicle_data)
                self.addItem(vehicle_item)
                self.vehicle_items[vehicle_id] = vehicle_item
            
            # 更新路径
            if self.show_options['trajectories']:
                if vehicle_id in self.path_items:
                    self.removeItem(self.path_items[vehicle_id])
                
                if hasattr(vehicle_data, 'path') and vehicle_data.path:
                    self._draw_vehicle_path(vehicle_id, vehicle_data)
    
    def update_backbone_visualization(self):
        """更新骨干网络可视化"""
        if self.backbone_view:
            self.removeItem(self.backbone_view)
        
        if self.backbone_network and self.show_options['backbone']:
            self.backbone_view = BackboneNetworkView(self.backbone_network)
            self.addItem(self.backbone_view)
    
    def update_conflicts(self):
        """更新冲突显示"""
        # 清除旧冲突
        for item in self.conflict_items.values():
            self.removeItem(item)
        self.conflict_items.clear()
        
        if (not self.show_options['conflicts'] or not self.traffic_manager):
            return
        
        # 获取当前冲突
        conflicts = self.traffic_manager.detect_all_conflicts()
        
        for i, conflict in enumerate(conflicts[:5]):  # 最多显示5个冲突
            conflict_item = self._create_conflict_indicator(conflict)
            if conflict_item:
                self.addItem(conflict_item)
                self.conflict_items[i] = conflict_item
    
    def _create_conflict_indicator(self, conflict):
        """创建冲突指示器"""
        x, y = conflict.location
        
        # 冲突区域 - 根据严重程度调整大小和颜色
        severity = getattr(conflict, 'severity', 1.0)
        radius = 2 + severity * 2  # 根据严重程度调整大小
        
        # 颜色根据冲突类型
        conflict_type = getattr(conflict, 'conflict_type', None)
        if hasattr(conflict_type, 'value'):
            type_name = conflict_type.value
        else:
            type_name = str(conflict_type)
        
        if type_name == 'backbone_congestion':
            color = QColor(245, 158, 11, 150)  # 橙色
        elif type_name == 'edge':
            color = QColor(239, 68, 68, 150)   # 红色
        else:
            color = QColor(239, 68, 68, 100)   # 默认红色
        
        warning_circle = QGraphicsEllipseItem(x-radius, y-radius, radius*2, radius*2)
        warning_circle.setBrush(QBrush(color))
        warning_circle.setPen(QPen(PROFESSIONAL_COLORS['error'], 2))
        warning_circle.setZValue(20)
        
        return warning_circle
    
    def toggle_show_option(self, option, show):
        """切换显示选项"""
        if option in self.show_options:
            self.show_options[option] = show
            
            if option == 'backbone':
                self.update_backbone_visualization()
            elif option == 'conflicts':
                self.update_conflicts()
            elif option in ['grid', 'trajectories', 'labels']:
                self.draw_environment()
            elif option == 'efficiency_indicators':
                self._toggle_efficiency_indicators(show)
    
    def _toggle_efficiency_indicators(self, show):
        """切换效率指示器显示"""
        # 清除现有指示器
        for item in self.efficiency_items.values():
            self.removeItem(item)
        self.efficiency_items.clear()
        
        if not show or not self.env:
            return
        
        # 为每个车辆添加效率指示器
        for vehicle_id, vehicle_data in self.env.vehicles.items():
            if vehicle_id in self.vehicle_items:
                efficiency = vehicle_data.get('efficiency_score', 0.5)
                self._create_efficiency_indicator(vehicle_id, vehicle_data.get('position', (0, 0, 0)), efficiency)
    
    def _create_efficiency_indicator(self, vehicle_id, position, efficiency):
        """创建效率指示器"""
        x, y = position[0], position[1]
        
        # 效率条
        bar_width = 20
        bar_height = 4
        bar_x = x - bar_width / 2
        bar_y = y + 15
        
        # 背景条
        bg_rect = QGraphicsRectItem(bar_x, bar_y, bar_width, bar_height)
        bg_rect.setBrush(QBrush(QColor(75, 85, 99)))
        bg_rect.setPen(QPen(Qt.NoPen))
        bg_rect.setZValue(25)
        self.addItem(bg_rect)
        
        # 效率条
        efficiency_width = bar_width * efficiency
        if efficiency > 0.8:
            efficiency_color = PROFESSIONAL_COLORS['success']
        elif efficiency > 0.6:
            efficiency_color = PROFESSIONAL_COLORS['primary']
        elif efficiency > 0.4:
            efficiency_color = PROFESSIONAL_COLORS['warning']
        else:
            efficiency_color = PROFESSIONAL_COLORS['error']
        
        eff_rect = QGraphicsRectItem(bar_x, bar_y, efficiency_width, bar_height)
        eff_rect.setBrush(QBrush(efficiency_color))
        eff_rect.setPen(QPen(Qt.NoPen))
        eff_rect.setZValue(26)
        self.addItem(eff_rect)
        
        # 存储指示器
        self.efficiency_items[f"{vehicle_id}_bg"] = bg_rect
        self.efficiency_items[f"{vehicle_id}_eff"] = eff_rect


class ProfessionalMineView(QGraphicsView):
    """专业的矿场图形视图"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setRenderHint(QPainter.TextAntialiasing, True)
        self.setMouseTracking(True)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        
        # 创建场景
        self.mine_scene = ProfessionalMineScene(self)
        self.setScene(self.mine_scene)
        
        # 缩放限制
        self.min_scale = 0.2
        self.max_scale = 3.0
        self.current_scale = 1.0
        
        # 状态标签
        self.create_status_overlay()
    
    def create_status_overlay(self):
        """创建状态覆盖层"""
        # 坐标显示
        self.coord_label = QLabel(self)
        self.coord_label.setStyleSheet("""
            QLabel {
                background-color: rgba(55, 58, 71, 200);
                color: rgb(229, 231, 235);
                padding: 6px 12px;
                border-radius: 4px;
                font-family: 'Consolas', monospace;
                font-size: 11px;
            }
        """)
        self.coord_label.setText("坐标: (0, 0)")
        self.coord_label.move(10, 10)
        self.coord_label.show()
        
        # 缩放显示
        self.zoom_label = QLabel(self)
        self.zoom_label.setStyleSheet("""
            QLabel {
                background-color: rgba(55, 58, 71, 200);
                color: rgb(229, 231, 235);
                padding: 6px 12px;
                border-radius: 4px;
                font-family: 'Consolas', monospace;
                font-size: 11px;
            }
        """)
        self.zoom_label.setText("缩放: 100%")
        self.zoom_label.move(120, 10)
        self.zoom_label.show()
    
    def wheelEvent(self, event):
        """鼠标滚轮缩放"""
        delta = event.angleDelta().y()
        factor = 1.15 if delta > 0 else 1/1.15
        
        new_scale = self.current_scale * factor
        if self.min_scale <= new_scale <= self.max_scale:
            self.scale(factor, factor)
            self.current_scale = new_scale
            self.zoom_label.setText(f"缩放: {int(self.current_scale * 100)}%")
    
    def mouseMoveEvent(self, event):
        """鼠标移动显示坐标"""
        super().mouseMoveEvent(event)
        scene_pos = self.mapToScene(event.pos())
        x, y = scene_pos.x(), scene_pos.y()
        self.coord_label.setText(f"坐标: ({x:.1f}, {y:.1f})")
    
    def set_environment(self, env):
        """设置环境"""
        if env:
            self.mine_scene.set_environment(env)
            self.fitInView(self.mine_scene.sceneRect(), Qt.KeepAspectRatio)
            self.current_scale = self.transform().m11()
            self.zoom_label.setText(f"缩放: {int(self.current_scale * 100)}%")
    
    def set_backbone_network(self, backbone_network):
        """设置骨干网络"""
        self.mine_scene.backbone_network = backbone_network
        self.mine_scene.update_backbone_visualization()
    
    def set_traffic_manager(self, traffic_manager):
        """设置交通管理器"""
        self.mine_scene.traffic_manager = traffic_manager
    
    def update_vehicles(self):
        """更新车辆"""
        self.mine_scene.update_vehicles()
    
    def toggle_show_option(self, option, show):
        """切换显示选项"""
        self.mine_scene.toggle_show_option(option, show)


class VehicleManagementWidget(QWidget):
    """车辆管理组件 - 增强版"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.env = None
        self.scheduler = None
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        
        # 车辆选择
        select_layout = QHBoxLayout()
        select_layout.addWidget(QLabel("车辆:"))
        
        self.vehicle_combo = QComboBox()
        self.vehicle_combo.currentIndexChanged.connect(self.update_vehicle_info)
        select_layout.addWidget(self.vehicle_combo, 1)
        
        layout.addLayout(select_layout)
        
        # 车辆信息表格
        self.vehicle_table = QTableWidget()
        self.vehicle_table.setColumnCount(4)
        self.vehicle_table.setHorizontalHeaderLabels(["属性", "值", "状态", "进度"])
        self.vehicle_table.setAlternatingRowColors(True)
        self.vehicle_table.verticalHeader().setVisible(False)
        
        layout.addWidget(self.vehicle_table)
        
        # 任务控制
        control_layout = QHBoxLayout()
        
        self.assign_btn = QPushButton("分配任务")
        self.cancel_btn = QPushButton("取消任务")
        self.optimize_btn = QPushButton("优化路径")  # 新功能
        
        control_layout.addWidget(self.assign_btn)
        control_layout.addWidget(self.cancel_btn)
        control_layout.addWidget(self.optimize_btn)
        
        layout.addLayout(control_layout)
        
        # 效率信息
        efficiency_group = QGroupBox("效率分析")
        efficiency_layout = QVBoxLayout()
        
        self.efficiency_label = QLabel("当前效率: --")
        self.avg_efficiency_label = QLabel("平均效率: --")
        self.backbone_usage_label = QLabel("骨干使用率: --")
        self.completed_cycles_label = QLabel("完成循环: --")
        
        efficiency_layout.addWidget(self.efficiency_label)
        efficiency_layout.addWidget(self.avg_efficiency_label)
        efficiency_layout.addWidget(self.backbone_usage_label)
        efficiency_layout.addWidget(self.completed_cycles_label)
        
        efficiency_group.setLayout(efficiency_layout)
        layout.addWidget(efficiency_group)
    
    def set_environment(self, env, scheduler):
        """设置环境和调度器"""
        self.env = env
        self.scheduler = scheduler
        
        self.vehicle_combo.clear()
        if env and env.vehicles:
            for v_id in sorted(env.vehicles.keys()):
                self.vehicle_combo.addItem(f"车辆 {v_id}", v_id)
        
        if self.vehicle_combo.count() > 0:
            self.update_vehicle_info(0)
    
    def update_vehicle_info(self, index=None):
        """更新车辆信息"""
        if not self.env or index is None or index < 0:
            return
        
        v_id = self.vehicle_combo.itemData(index)
        if v_id not in self.env.vehicles:
            return
        
        vehicle = self.env.vehicles[v_id]
        
        # 从调度器获取增强信息
        vehicle_state = None
        if self.scheduler and hasattr(self.scheduler, 'vehicle_states'):
            vehicle_state = self.scheduler.vehicle_states.get(v_id)
        
        # 构建信息行
        info_rows = [
            ("车辆ID", str(v_id), "", ""),
            ("位置", f"({vehicle.position[0]:.1f}, {vehicle.position[1]:.1f})", "", ""),
            ("状态", vehicle.status, "", ""),
            ("载重", f"{vehicle.current_load}/{vehicle.max_load}", "", f"{vehicle.current_load/vehicle.max_load*100:.0f}%"),
            ("完成循环", str(vehicle.completed_cycles), "", "")
        ]
        
        # 添加增强信息
        if vehicle_state:
            info_rows.extend([
                ("优先级", f"{vehicle_state.priority_level:.2f}", "", ""),
                ("总距离", f"{vehicle_state.total_distance:.1f}m", "", ""),
                ("平均速度", f"{vehicle_state.average_speed:.2f}", "", ""),
                ("空闲时间", f"{vehicle_state.idle_time:.1f}s", "", ""),
                ("生产时间", f"{vehicle_state.productive_time:.1f}s", "", "")
            ])
        
        # 更新表格
        self.vehicle_table.setRowCount(len(info_rows))
        for row, (attr, value, status, progress) in enumerate(info_rows):
            self.vehicle_table.setItem(row, 0, QTableWidgetItem(attr))
            self.vehicle_table.setItem(row, 1, QTableWidgetItem(value))
            self.vehicle_table.setItem(row, 2, QTableWidgetItem(status))
            self.vehicle_table.setItem(row, 3, QTableWidgetItem(progress))
        
        self.vehicle_table.resizeColumnsToContents()
        
        # 更新效率信息
        if vehicle_state:
            current_eff = vehicle_state.calculate_efficiency_score()
            self.efficiency_label.setText(f"当前效率: {current_eff:.2%}")
            
            if vehicle_state.efficiency_history:
                avg_eff = sum(vehicle_state.efficiency_history) / len(vehicle_state.efficiency_history)
                self.avg_efficiency_label.setText(f"平均效率: {avg_eff:.2%}")
            
            self.backbone_usage_label.setText(f"骨干使用率: {vehicle_state.backbone_usage_ratio:.2%}")
            self.completed_cycles_label.setText(f"完成循环: {vehicle_state.completed_cycles}")


class ProfessionalControlPanel(QWidget):
    """专业控制面板 - 增强版"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = None
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        
        # 环境管理
        env_group = QGroupBox("环境管理")
        env_layout = QVBoxLayout()
        
        file_layout = QHBoxLayout()
        self.file_label = QLabel("未选择文件")
        self.file_label.setStyleSheet("""
            QLabel {
                background-color: rgb(45, 47, 57);
                padding: 8px;
                border: 1px solid rgb(75, 85, 99);
                border-radius: 4px;
                color: rgb(156, 163, 175);
            }
        """)
        
        self.browse_btn = QPushButton("浏览文件")
        file_layout.addWidget(self.file_label, 1)
        file_layout.addWidget(self.browse_btn)
        
        env_layout.addLayout(file_layout)
        
        control_layout = QHBoxLayout()
        self.load_btn = QPushButton("加载环境")
        self.save_btn = QPushButton("保存环境")
        control_layout.addWidget(self.load_btn)
        control_layout.addWidget(self.save_btn)
        
        env_layout.addLayout(control_layout)
        env_group.setLayout(env_layout)
        layout.addWidget(env_group)
        
        # 骨干网络
        backbone_group = QGroupBox("骨干网络")
        backbone_layout = QVBoxLayout()
        
        param_layout = QGridLayout()
        param_layout.addWidget(QLabel("质量阈值:"), 0, 0)
        self.quality_spin = QDoubleSpinBox()
        self.quality_spin.setRange(0.1, 1.0)
        self.quality_spin.setSingleStep(0.1)
        self.quality_spin.setValue(0.6)
        param_layout.addWidget(self.quality_spin, 0, 1)
        
        param_layout.addWidget(QLabel("负载均衡:"), 1, 0)
        self.load_balancing_cb = QCheckBox("启用负载均衡")
        self.load_balancing_cb.setChecked(True)
        param_layout.addWidget(self.load_balancing_cb, 1, 1)
        
        backbone_layout.addLayout(param_layout)
        
        self.generate_btn = QPushButton("生成骨干网络")
        backbone_layout.addWidget(self.generate_btn)
        
        self.backbone_stats_label = QLabel("路径: 0 条")
        self.backbone_stats_label.setStyleSheet("color: rgb(16, 185, 129); font-weight: bold;")
        backbone_layout.addWidget(self.backbone_stats_label)
        
        backbone_group.setLayout(backbone_layout)
        layout.addWidget(backbone_group)
        
        # 任务管理
        task_group = QGroupBox("任务管理")
        task_layout = QVBoxLayout()
        
        priority_layout = QHBoxLayout()
        priority_layout.addWidget(QLabel("优先级:"))
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["低", "普通", "高", "紧急", "关键"])
        self.priority_combo.setCurrentIndex(1)  # 默认普通
        priority_layout.addWidget(self.priority_combo)
        task_layout.addLayout(priority_layout)
        
        assign_layout = QHBoxLayout()
        self.assign_single_btn = QPushButton("分配单个")
        self.assign_all_btn = QPushButton("批量分配")
        assign_layout.addWidget(self.assign_single_btn)
        assign_layout.addWidget(self.assign_all_btn)
        
        task_layout.addLayout(assign_layout)
        
        # 优化控制
        optimize_layout = QHBoxLayout()
        self.optimize_system_btn = QPushButton("系统优化")
        self.rebalance_btn = QPushButton("负载重平衡")
        optimize_layout.addWidget(self.optimize_system_btn)
        optimize_layout.addWidget(self.rebalance_btn)
        
        task_layout.addLayout(optimize_layout)
        
        task_group.setLayout(task_layout)
        layout.addWidget(task_group)
        
        # 仿真控制
        sim_group = QGroupBox("仿真控制")
        sim_layout = QVBoxLayout()
        
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
        self.progress_bar.setTextVisible(True)
        sim_layout.addWidget(self.progress_bar)
        
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # 显示选项
        display_group = QGroupBox("显示选项")
        display_layout = QVBoxLayout()
        
        self.show_backbone_cb = QCheckBox("显示骨干网络")
        self.show_backbone_cb.setChecked(True)
        
        self.show_trajectories_cb = QCheckBox("显示车辆轨迹")
        self.show_trajectories_cb.setChecked(True)
        
        self.show_conflicts_cb = QCheckBox("显示冲突")
        self.show_conflicts_cb.setChecked(True)
        
        self.show_efficiency_cb = QCheckBox("显示效率指示器")  # 新功能
        self.show_efficiency_cb.setChecked(False)
        
        self.show_load_indicators_cb = QCheckBox("显示负载指示器")  # 新功能
        self.show_load_indicators_cb.setChecked(True)
        
        display_layout.addWidget(self.show_backbone_cb)
        display_layout.addWidget(self.show_trajectories_cb)
        display_layout.addWidget(self.show_conflicts_cb)
        display_layout.addWidget(self.show_efficiency_cb)
        display_layout.addWidget(self.show_load_indicators_cb)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        layout.addStretch()
    
    def set_main_window(self, main_window):
        """设置主窗口引用"""
        self.main_window = main_window
        
        # 连接信号
        self.browse_btn.clicked.connect(main_window.browse_file)
        self.load_btn.clicked.connect(main_window.load_environment)
        self.save_btn.clicked.connect(main_window.save_environment)
        self.generate_btn.clicked.connect(main_window.generate_backbone_network)
        
        self.assign_single_btn.clicked.connect(main_window.assign_single_vehicle)
        self.assign_all_btn.clicked.connect(main_window.assign_all_vehicles)
        self.optimize_system_btn.clicked.connect(main_window.optimize_system)
        self.rebalance_btn.clicked.connect(main_window.rebalance_loads)
        
        self.start_btn.clicked.connect(main_window.start_simulation)
        self.pause_btn.clicked.connect(main_window.pause_simulation)
        self.reset_btn.clicked.connect(main_window.reset_simulation)
        
        self.speed_slider.valueChanged.connect(main_window.update_simulation_speed)
        
        # 显示选项
        self.show_backbone_cb.toggled.connect(
            lambda s: main_window.toggle_display_option('backbone', s)
        )
        self.show_trajectories_cb.toggled.connect(
            lambda s: main_window.toggle_display_option('trajectories', s)
        )
        self.show_conflicts_cb.toggled.connect(
            lambda s: main_window.toggle_display_option('conflicts', s)
        )
        self.show_efficiency_cb.toggled.connect(
            lambda s: main_window.toggle_display_option('efficiency_indicators', s)
        )
        self.show_load_indicators_cb.toggled.connect(
            lambda s: main_window.toggle_display_option('load_indicators', s)
        )
    
    def update_backbone_stats(self, path_count, load_info=None):
        """更新骨干网络统计"""
        stats_text = f"路径: {path_count} 条"
        if load_info:
            avg_load = load_info.get('average_load', 0)
            stats_text += f" (平均负载: {avg_load:.1%})"
        
        self.backbone_stats_label.setText(stats_text)


class ProfessionalMineGUI(QMainWindow):
    """专业版露天矿调度系统GUI - 整合增强组件"""
    
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
        self.update_timer.start(100)
        
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.simulation_step)
        
        self.stats_timer = QTimer(self)
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(2000)
        
        # 效率监控定时器
        self.efficiency_timer = QTimer(self)
        self.efficiency_timer.timeout.connect(self.update_efficiency_display)
        self.efficiency_timer.start(5000)  # 每5秒更新一次效率显示
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("露天矿多车协同调度系统 - 增强版")
        self.setGeometry(100, 100, 1600, 1000)
        
        # 专业配色
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {PROFESSIONAL_COLORS['background'].name()};
                color: {PROFESSIONAL_COLORS['text'].name()};
            }}
            QGroupBox {{
                font-weight: bold;
                border: 1px solid {PROFESSIONAL_COLORS['border'].name()};
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 6px;
                color: {PROFESSIONAL_COLORS['text'].name()};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
                color: {PROFESSIONAL_COLORS['text'].name()};
            }}
            QPushButton {{
                background-color: {PROFESSIONAL_COLORS['primary'].name()};
                color: white;
                border: none;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {PROFESSIONAL_COLORS['primary'].darker(110).name()};
            }}
            QPushButton:pressed {{
                background-color: {PROFESSIONAL_COLORS['primary'].darker(120).name()};
            }}
            QComboBox, QSpinBox, QDoubleSpinBox {{
                background-color: {PROFESSIONAL_COLORS['surface'].name()};
                border: 1px solid {PROFESSIONAL_COLORS['border'].name()};
                border-radius: 4px;
                padding: 4px;
                color: {PROFESSIONAL_COLORS['text'].name()};
            }}
        """)
        
        # 中央组件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)
        
        # 左侧控制面板
        self.control_panel = ProfessionalControlPanel()
        self.control_panel.set_main_window(self)
        self.control_panel.setMaximumWidth(300)
        main_layout.addWidget(self.control_panel)
        
        # 中央视图
        self.graphics_view = ProfessionalMineView()
        main_layout.addWidget(self.graphics_view, 1)
        
        # 右侧面板（使用标签页）
        right_widget = QTabWidget()
        right_widget.setMaximumWidth(350)
        
        # 实时状态标签页
        self.status_widget = RealTimeStatusWidget()
        right_widget.addTab(self.status_widget, "实时状态")
        
        # 车辆管理标签页
        self.vehicle_widget = VehicleManagementWidget()
        right_widget.addTab(self.vehicle_widget, "车辆管理")
        
        # 系统效率标签页
        self.efficiency_widget = SystemEfficiencyWidget()
        right_widget.addTab(self.efficiency_widget, "系统效率")
        
        main_layout.addWidget(right_widget)
        
        # 创建菜单和工具栏
        self.create_menu_bar()
        self.create_status_bar()
    
    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu('文件')
        
        open_action = file_menu.addAction('打开地图')
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.browse_file)
        
        save_action = file_menu.addAction('保存环境')
        save_action.setShortcut('Ctrl+S')
        save_action.triggered.connect(self.save_environment)
        
        file_menu.addSeparator()
        
        exit_action = file_menu.addAction('退出')
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        
        # 视图菜单
        view_menu = menubar.addMenu('视图')
        
        zoom_in_action = view_menu.addAction('放大')
        zoom_in_action.setShortcut('Ctrl++')
        zoom_in_action.triggered.connect(self.zoom_in)
        
        zoom_out_action = view_menu.addAction('缩小')
        zoom_out_action.setShortcut('Ctrl+-')
        zoom_out_action.triggered.connect(self.zoom_out)
        
        fit_view_action = view_menu.addAction('适应视图')
        fit_view_action.setShortcut('Ctrl+0')
        fit_view_action.triggered.connect(self.fit_view)
        
        # 优化菜单
        optimize_menu = menubar.addMenu('优化')
        
        system_optimize_action = optimize_menu.addAction('系统优化')
        system_optimize_action.setShortcut('F8')
        system_optimize_action.triggered.connect(self.optimize_system)
        
        rebalance_action = optimize_menu.addAction('负载重平衡')
        rebalance_action.setShortcut('F9')
        rebalance_action.triggered.connect(self.rebalance_loads)
        
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
    
    def create_status_bar(self):
        """创建状态栏"""
        self.status_bar = self.statusBar()
        
        self.status_label = QLabel("系统就绪")
        self.status_bar.addWidget(self.status_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.vehicle_count_label = QLabel("车辆: 0")
        self.status_bar.addPermanentWidget(self.vehicle_count_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.sim_time_label = QLabel("时间: 00:00")
        self.status_bar.addPermanentWidget(self.sim_time_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.efficiency_label = QLabel("效率: --")
        self.status_bar.addPermanentWidget(self.efficiency_label)
    
    # 主要功能方法
    def browse_file(self):
        """浏览文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开地图文件", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            self.map_file_path = file_path
            filename = os.path.basename(file_path)
            self.control_panel.file_label.setText(filename)
            self.control_panel.file_label.setStyleSheet("""
                QLabel {
                    background-color: rgb(16, 185, 129);
                    color: white;
                    padding: 8px;
                    border: 1px solid rgb(75, 85, 99);
                    border-radius: 4px;
                }
            """)
    
    def load_environment(self):
        """加载环境"""
        if not self.map_file_path:
            QMessageBox.warning(self, "警告", "请先选择地图文件")
            return
        
        try:
            self.status_label.setText("正在加载环境...")
            
            # 创建环境
            self.env = OptimizedOpenPitMineEnv()
            if not self.env.load_from_file(self.map_file_path):
                raise Exception("环境加载失败")
            
            # 设置到视图
            self.graphics_view.set_environment(self.env)
            
            # 创建系统组件
            self.create_system_components()
            
            # 更新UI组件
            self.vehicle_widget.set_environment(self.env, self.vehicle_scheduler)
            self.status_widget.set_scheduler(self.vehicle_scheduler)
            self.status_widget.set_traffic_manager(self.traffic_manager)
            self.efficiency_widget.set_scheduler(self.vehicle_scheduler)
            
            # 设置组件引用
            self.graphics_view.mine_scene.backbone_network = self.backbone_network
            self.graphics_view.mine_scene.traffic_manager = self.traffic_manager
            
            self.status_label.setText("环境加载成功")
            self.enable_controls(True)
            
            # 更新车辆计数
            self.vehicle_count_label.setText(f"车辆: {len(self.env.vehicles)}")
            
        except Exception as e:
            self.status_label.setText("加载失败")
            QMessageBox.critical(self, "错误", f"加载环境失败:\n{str(e)}")
    
    def save_environment(self):
        """保存环境"""
        if not self.env:
            QMessageBox.warning(self, "警告", "没有可保存的环境")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存环境", 
            f"mine_env_{time.strftime('%Y%m%d_%H%M%S')}.json",
            "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            try:
                if self.env.save_to_file(file_path):
                    self.status_label.setText("环境保存成功")
                    QMessageBox.information(self, "成功", "环境保存成功")
                else:
                    raise Exception("保存失败")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"保存失败:\n{str(e)}")
    
    def create_system_components(self):
        """创建系统组件"""
        try:
            print("开始创建增强系统组件...")
            
            # 创建路径规划器
            if OPTIMIZED_COMPONENTS_AVAILABLE:
                self.path_planner = EnhancedPathPlannerWithConfig(self.env)
                print("✅ 增强路径规划器创建成功")
            else:
                self.path_planner = EnhancedPathPlannerWithConfig(self.env)
                print("✅ 基础路径规划器创建成功")
            
            # 创建骨干网络
            self.backbone_network = OptimizedBackboneNetwork(self.env)
            self.backbone_network.set_path_planner(self.path_planner)
            print("✅ 优化骨干网络创建成功")
            
            # 创建交通管理器
            self.traffic_manager = OptimizedTrafficManager(
                self.env, self.backbone_network, self.path_planner
            )
            print("✅ 优化交通管理器创建成功")
            
            # 创建车辆调度器
            self.vehicle_scheduler = EnhancedVehicleScheduler(
                self.env, self.path_planner, self.backbone_network, self.traffic_manager
            )
            print("✅ 增强车辆调度器创建成功")
            
            # 设置组件间引用
            if hasattr(self.path_planner, 'set_backbone_network'):
                self.path_planner.set_backbone_network(self.backbone_network)
            
            self.traffic_manager.set_backbone_network(self.backbone_network)
            self.traffic_manager.set_path_planner(self.path_planner)
            self.vehicle_scheduler.set_backbone_network(self.backbone_network)
            
            # 初始化车辆状态
            self.vehicle_scheduler.initialize_vehicles()
            
            # 创建默认任务模板
            if self.env.loading_points and self.env.unloading_points:
                from vehicle_scheduler import TaskPriority
                self.vehicle_scheduler.create_enhanced_mission_template(
                    "default", priority=TaskPriority.NORMAL
                )
                print("✅ 默认任务模板创建成功")
            
            print("🎉 所有增强组件创建完成")
            
        except Exception as e:
            raise Exception(f"系统组件初始化失败: {str(e)}")
    
    def generate_backbone_network(self):
        """生成骨干网络"""
        if not self.env or not self.backbone_network:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            self.status_label.setText("正在生成增强骨干网络...")
            
            quality_threshold = self.control_panel.quality_spin.value()
            enable_load_balancing = self.control_panel.load_balancing_cb.isChecked()
            
            # 设置负载均衡权重
            if enable_load_balancing:
                self.backbone_network.config['load_balancing_weight'] = 0.4
            else:
                self.backbone_network.config['load_balancing_weight'] = 0.0
            
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
                
                # 更新可视化
                self.graphics_view.mine_scene.set_backbone_network(self.backbone_network)
                
                # 获取网络状态
                network_status = self.backbone_network.get_network_status()
                path_count = network_status['bidirectional_paths']
                load_info = network_status.get('load_balancing', {})
                
                self.control_panel.update_backbone_stats(path_count, load_info)
                
                self.status_label.setText("增强骨干网络生成成功")
                QMessageBox.information(self, "成功", 
                    f"增强骨干网络生成成功\n"
                    f"双向路径: {path_count} 条\n"
                    f"负载均衡: {'启用' if enable_load_balancing else '禁用'}\n"
                    f"平均路径利用率: {load_info.get('average_path_utilization', 0):.2%}")
            else:
                self.status_label.setText("生成失败")
                QMessageBox.critical(self, "错误", "骨干网络生成失败")
                
        except Exception as e:
            self.status_label.setText("生成异常")
            QMessageBox.critical(self, "错误", f"生成骨干网络失败:\n{str(e)}")
    
    def assign_single_vehicle(self):
        """分配单个车辆任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        # 获取优先级
        priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT", 4: "CRITICAL"}
        priority_index = self.control_panel.priority_combo.currentIndex()
        priority_name = priority_map[priority_index]
        
        try:
            from vehicle_scheduler import TaskPriority
            priority = getattr(TaskPriority, priority_name)
            
            # 智能分配（让调度器选择最优车辆）
            success = self.vehicle_scheduler.assign_mission_intelligently(
                vehicle_id=None, priority=priority
            )
            
            if success:
                self.status_label.setText(f"已智能分配任务 (优先级: {priority_name})")
            else:
                QMessageBox.information(self, "提示", "没有找到合适的车辆")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"任务分配失败: {str(e)}")
    
    def assign_all_vehicles(self):
        """批量分配任务 - 修改为完整工作循环"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        # 获取优先级
        priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT", 4: "CRITICAL"}
        priority_index = self.control_panel.priority_combo.currentIndex()
        priority_name = priority_map[priority_index]
        
        try:
            from vehicle_scheduler import TaskPriority
            priority = getattr(TaskPriority, priority_name)
            
            # 获取可用的点位
            available_points = self._get_available_points()
            
            if not self._validate_available_points(available_points):
                QMessageBox.warning(self, "警告", 
                    "没有足够的空闲点位进行批量分配\n"
                    f"需要: 装载点、卸载点、停车点\n"
                    f"可用: 装载点{len(available_points['loading'])}个, "
                    f"卸载点{len(available_points['unloading'])}个, "
                    f"停车点{len(available_points['parking'])}个")
                return
            
            assigned_count = 0
            assignment_details = []
            
            for vehicle_id in self.env.vehicles.keys():
                # 为每个车辆智能分配点位组合
                point_assignment = self._assign_optimal_points(vehicle_id, available_points)
                
                if point_assignment:
                    success = self.vehicle_scheduler.assign_complete_work_cycle(
                        vehicle_id=vehicle_id,
                        loading_point_id=point_assignment['loading'],
                        unloading_point_id=point_assignment['unloading'],
                        parking_point_id=point_assignment['parking'],
                        priority=priority
                    )
                    
                    if success:
                        assigned_count += 1
                        assignment_details.append(
                            f"车辆{vehicle_id}: L{point_assignment['loading']} → "
                            f"U{point_assignment['unloading']} → P{point_assignment['parking']}"
                        )
                        
                        # 标记点位为已占用（避免重复分配）
                        self._mark_points_as_assigned(point_assignment, available_points)
            
            self.status_label.setText(f"已为 {assigned_count} 个车辆分配完整工作循环")
            
            # 显示详细分配结果
            details_text = "\n".join(assignment_details[:10])  # 最多显示10个
            if len(assignment_details) > 10:
                details_text += f"\n... 还有{len(assignment_details) - 10}个分配"
            
            QMessageBox.information(self, "批量分配成功", 
                f"已为 {assigned_count} 个车辆分配完整工作循环\n"
                f"优先级: {priority_name}\n\n"
                f"分配详情:\n{details_text}")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"批量任务分配失败: {str(e)}")
    
    def optimize_system(self):
        """系统优化"""
        if not self.vehicle_scheduler:
            QMessageBox.warning(self, "警告", "请先加载环境并初始化调度器")
            return
        
        try:
            if hasattr(self.vehicle_scheduler, 'efficiency_optimizer'):
                self.status_label.setText("正在执行系统优化...")
                
                result = self.vehicle_scheduler.efficiency_optimizer.optimize_system()
                
                improvement = result.get('efficiency_improvement', 0)
                optimization_time = result.get('optimization_time', 0)
                
                self.status_label.setText("系统优化完成")
                
                QMessageBox.information(self, "系统优化完成",
                    f"优化结果:\n"
                    f"车辆重平衡: {result.get('vehicle_rebalancing', 0)} 次\n"
                    f"任务重分配: {result.get('task_reassignments', 0)} 次\n"
                    f"骨干路径优化: {result.get('backbone_optimizations', 0)} 次\n"
                    f"效率提升: {improvement:.2%}\n"
                    f"优化耗时: {optimization_time:.2f}秒")
            else:
                QMessageBox.warning(self, "不支持", "当前调度器不支持系统优化功能")
                
        except Exception as e:
            self.status_label.setText("优化失败")
            QMessageBox.critical(self, "错误", f"系统优化失败:\n{str(e)}")
    
    def rebalance_loads(self):
        """负载重平衡"""
        if not self.vehicle_scheduler:
            QMessageBox.warning(self, "警告", "请先加载环境并初始化调度器")
            return
        
        try:
            if hasattr(self.vehicle_scheduler, 'efficiency_optimizer'):
                self.status_label.setText("正在执行负载重平衡...")
                
                optimizer = self.vehicle_scheduler.efficiency_optimizer
                rebalanced_count = optimizer._rebalance_vehicle_loads()
                
                self.status_label.setText("负载重平衡完成")
                
                if rebalanced_count > 0:
                    QMessageBox.information(self, "重平衡完成",
                        f"成功重平衡 {rebalanced_count} 个任务分配")
                else:
                    QMessageBox.information(self, "重平衡完成",
                        "当前负载已经均衡，无需调整")
            else:
                QMessageBox.warning(self, "不支持", "当前调度器不支持负载重平衡功能")
                
        except Exception as e:
            self.status_label.setText("重平衡失败")
            QMessageBox.critical(self, "错误", f"负载重平衡失败:\n{str(e)}")
    
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
        
        self.status_label.setText("仿真运行中...")
    
    def pause_simulation(self):
        """暂停仿真"""
        self.is_simulating = False
        self.control_panel.start_btn.setEnabled(True)
        self.control_panel.pause_btn.setEnabled(False)
        
        self.sim_timer.stop()
        
        self.status_label.setText("仿真已暂停")
    
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
        
        self.graphics_view.set_environment(self.env)
        self.vehicle_widget.set_environment(self.env, self.vehicle_scheduler)
        
        self.simulation_time = 0
        self.control_panel.progress_bar.setValue(0)
        
        self.status_label.setText("仿真已重置")
    
    def simulation_step(self):
        """仿真步骤"""
        if not self.is_simulating or not self.env:
            return
        
        time_step = 0.5 * self.simulation_speed
        self.simulation_time += time_step
        
        # 更新环境
        self.env.current_time = self.simulation_time
        
        # 更新调度器
        if self.vehicle_scheduler:
            try:
                self.vehicle_scheduler.update(time_step)
            except Exception as e:
                print(f"调度器更新错误: {e}")
        
        # 更新交通管理器
        if self.traffic_manager:
            try:
                self.traffic_manager.update(time_step)
            except Exception as e:
                print(f"交通管理器更新错误: {e}")
        
        # 更新骨干网络负载均衡
        if self.backbone_network:
            try:
                self.backbone_network.update_load_balancing(time_step)
            except Exception as e:
                print(f"骨干网络更新错误: {e}")
        
        # 更新进度条
        max_time = 1800  # 30分钟
        progress = min(100, int(self.simulation_time * 100 / max_time))
        self.control_panel.progress_bar.setValue(progress)
        
        # 更新时间显示
        minutes = int(self.simulation_time // 60)
        seconds = int(self.simulation_time % 60)
        self.sim_time_label.setText(f"时间: {minutes:02d}:{seconds:02d}")
        
        if progress >= 100:
            self.pause_simulation()
            QMessageBox.information(self, "完成", "仿真已完成！")
    
    def update_simulation_speed(self, value):
        """更新仿真速度"""
        self.simulation_speed = value / 50.0
        self.control_panel.speed_label.setText(f"{self.simulation_speed:.1f}x")
        
        # 更新定时器间隔
        if self.is_simulating:
            interval = max(50, int(100 / self.simulation_speed))
            self.sim_timer.start(interval)
    
    def toggle_display_option(self, option, show):
        """切换显示选项"""
        self.graphics_view.toggle_show_option(option, show)
    
    def update_display(self):
        """更新显示"""
        if not self.env:
            return
        
        # 更新车辆显示
        self.graphics_view.update_vehicles()
        
        # 更新冲突显示
        if self.graphics_view.mine_scene.show_options.get('conflicts', False):
            self.graphics_view.mine_scene.update_conflicts()
        
        # 更新骨干网络可视化（如果有负载变化）
        if self.backbone_network and self.graphics_view.mine_scene.show_options.get('load_indicators', False):
            self.graphics_view.mine_scene.update_backbone_visualization()
    
    def update_statistics(self):
        """更新统计信息"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 更新实时状态
            self.status_widget.update_stats()
            
            # 更新车辆管理组件
            current_index = self.vehicle_widget.vehicle_combo.currentIndex()
            if current_index >= 0:
                self.vehicle_widget.update_vehicle_info(current_index)
                
        except Exception as e:
            print(f"统计更新错误: {e}")
    
    def update_efficiency_display(self):
        """更新效率显示"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 更新系统效率组件
            self.efficiency_widget.update_efficiency_display()
            
            # 更新状态栏效率显示
            if hasattr(self.vehicle_scheduler, 'efficiency_optimizer'):
                current_efficiency = self.vehicle_scheduler.efficiency_optimizer.calculate_system_efficiency()
                self.efficiency_label.setText(f"效率: {current_efficiency:.1%}")
            
        except Exception as e:
            print(f"效率显示更新错误: {e}")
    
    def enable_controls(self, enabled):
        """启用/禁用控件"""
        self.control_panel.start_btn.setEnabled(enabled)
        self.control_panel.reset_btn.setEnabled(enabled)
        self.control_panel.generate_btn.setEnabled(enabled)
        self.control_panel.assign_single_btn.setEnabled(enabled)
        self.control_panel.assign_all_btn.setEnabled(enabled)
        self.control_panel.optimize_system_btn.setEnabled(enabled)
        self.control_panel.rebalance_btn.setEnabled(enabled)
        self.control_panel.save_btn.setEnabled(enabled)
    
    def zoom_in(self):
        """放大视图"""
        self.graphics_view.scale(1.2, 1.2)
        self.graphics_view.current_scale *= 1.2
        self.graphics_view.zoom_label.setText(f"缩放: {int(self.graphics_view.current_scale * 100)}%")
    
    def zoom_out(self):
        """缩小视图"""
        self.graphics_view.scale(1/1.2, 1/1.2)
        self.graphics_view.current_scale /= 1.2
        self.graphics_view.zoom_label.setText(f"缩放: {int(self.graphics_view.current_scale * 100)}%")
    
    def fit_view(self):
        """适应视图"""
        self.graphics_view.fitInView(
            self.graphics_view.mine_scene.sceneRect(), Qt.KeepAspectRatio
        )
        self.graphics_view.current_scale = self.graphics_view.transform().m11()
        self.graphics_view.zoom_label.setText(f"缩放: {int(self.graphics_view.current_scale * 100)}%")
    
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
        
        # 停止所有定时器
        self.update_timer.stop()
        self.sim_timer.stop()
        self.stats_timer.stop()
        self.efficiency_timer.stop()
        
        # 关闭系统组件
        try:
            if self.vehicle_scheduler:
                self.vehicle_scheduler.shutdown()
            if self.traffic_manager:
                self.traffic_manager.shutdown()
            if self.path_planner:
                self.path_planner.shutdown()
        except Exception as e:
            print(f"组件关闭错误: {e}")
        
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # 设置应用信息
    app.setApplicationName("露天矿调度系统")
    app.setApplicationVersion("增强版 v2.0")
    
    # 检查组件可用性
    print("=" * 60)
    print("露天矿多车协同调度系统 - 增强版")
    print("=" * 60)
    print(f"优化组件可用性: {'✅ 是' if OPTIMIZED_COMPONENTS_AVAILABLE else '⚠️ 否'}")
    print("支持功能:")
    print("  ✅ 智能骨干路径网络生成")
    print("  ✅ 动态负载均衡")
    print("  ✅ 停车避让机制")
    print("  ✅ 骨干路径动态切换")
    print("  ✅ 系统效率优化")
    print("  ✅ 实时性能监控")
    print("  ✅ 增强冲突解决")
    print("  ✅ 智能任务分配")
    print("=" * 60)
    
    window = ProfessionalMineGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()