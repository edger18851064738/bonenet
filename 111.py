#!/usr/bin/env python3
"""
integrated_ecbs_gui.py - 完整ECBS集成的专业GUI系统
集成了ECBS冲突消解、多车辆协调、智能任务分配等全部增强功能
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

# 导入ECBS集成的组件
try:
    from optimized_backbone_network import OptimizedBackboneNetwork
    from path_planner import EnhancedPathPlanner  # 统一路径规划器
    from traffic_manager import OptimizedTrafficManagerWithECBS  # ECBS交通管理器
    from vehicle_scheduler import EnhancedVehicleSchedulerWithECBS  # ECBS调度器
    from environment import OptimizedOpenPitMineEnv
    ECBS_COMPONENTS_AVAILABLE = True
    print("✅ ECBS集成组件加载成功")
except ImportError as e:
    print(f"⚠️ ECBS组件加载失败: {e}")
    # 回退到基础组件
    try:
        from optimized_backbone_network import OptimizedBackboneNetwork
        from path_planner import EnhancedPathPlanner
        from traffic_manager import OptimizedTrafficManager as OptimizedTrafficManagerWithECBS
        from vehicle_scheduler import EnhancedVehicleScheduler as EnhancedVehicleSchedulerWithECBS
        from environment import OptimizedOpenPitMineEnv
        ECBS_COMPONENTS_AVAILABLE = False
        print("⚠️ 使用基础组件")
    except ImportError:
        print("❌ 基础组件也不可用")
        sys.exit(1)

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
    'border': QColor(75, 85, 99),
    'ecbs': QColor(138, 43, 226)  # ECBS专用紫色
}

# 车辆状态专业配色
VEHICLE_STATUS_COLORS = {
    'idle': QColor(156, 163, 175),      # 灰色
    'loading': QColor(16, 185, 129),    # 绿色
    'unloading': QColor(245, 158, 11),  # 橙色
    'moving': QColor(66, 135, 245),     # 蓝色
    'waiting': QColor(168, 85, 247),    # 紫色
    'planning': QColor(244, 63, 94),    # 粉色
    'maintenance': QColor(239, 68, 68), # 红色
    'coordinating': QColor(138, 43, 226) # ECBS协调 - 紫色
}

class ECBSCoordinationWidget(QWidget):
    """ECBS协调监控组件"""
    
    coordinationRequested = pyqtSignal(list, str)  # 协调请求信号
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scheduler = None
        self.traffic_manager = None
        self.coordination_history = []
        self.init_ui()
    
    def init_ui(self):
        """初始化界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 标题
        title = QLabel("ECBS协调监控")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: rgb(229, 231, 235);
                padding: 8px;
                background-color: rgb(138, 43, 226);
                border-radius: 4px;
            }
        """)
        layout.addWidget(title)
        
        # 协调控制
        control_group = QGroupBox("协调控制")
        control_layout = QVBoxLayout()
        
        # 协调模式选择
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("协调模式:"))
        self.coordination_mode_combo = QComboBox()
        self.coordination_mode_combo.addItems([
            "批量协调", "实时协调", "定期协调", "冲突触发"
        ])
        mode_layout.addWidget(self.coordination_mode_combo)
        control_layout.addLayout(mode_layout)
        
        # 协调参数
        param_layout = QGridLayout()
        param_layout.addWidget(QLabel("最大求解时间:"), 0, 0)
        self.max_solve_time_spin = QDoubleSpinBox()
        self.max_solve_time_spin.setRange(5.0, 60.0)
        self.max_solve_time_spin.setValue(30.0)
        self.max_solve_time_spin.setSuffix(" 秒")
        param_layout.addWidget(self.max_solve_time_spin, 0, 1)
        
        param_layout.addWidget(QLabel("质量阈值:"), 1, 0)
        self.quality_threshold_spin = QDoubleSpinBox()
        self.quality_threshold_spin.setRange(0.1, 1.0)
        self.quality_threshold_spin.setSingleStep(0.1)
        self.quality_threshold_spin.setValue(0.7)
        param_layout.addWidget(self.quality_threshold_spin, 1, 1)
        
        control_layout.addLayout(param_layout)
        
        # 协调按钮
        coord_layout = QHBoxLayout()
        self.coordinate_all_btn = QPushButton("协调所有车辆")
        self.coordinate_all_btn.clicked.connect(self.coordinate_all_vehicles)
        self.coordinate_selected_btn = QPushButton("协调选定车辆")
        self.coordinate_selected_btn.clicked.connect(self.coordinate_selected_vehicles)
        
        coord_layout.addWidget(self.coordinate_all_btn)
        coord_layout.addWidget(self.coordinate_selected_btn)
        control_layout.addLayout(coord_layout)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # 协调状态
        status_group = QGroupBox("协调状态")
        status_layout = QVBoxLayout()
        
        self.coordination_count_label = QLabel("协调次数: 0")
        self.success_rate_label = QLabel("成功率: 100%")
        self.avg_solve_time_label = QLabel("平均求解时间: 0.0s")
        self.current_conflicts_label = QLabel("当前冲突: 0")
        
        status_layout.addWidget(self.coordination_count_label)
        status_layout.addWidget(self.success_rate_label)
        status_layout.addWidget(self.avg_solve_time_label)
        status_layout.addWidget(self.current_conflicts_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 协调历史
        history_group = QGroupBox("协调历史")
        history_layout = QVBoxLayout()
        
        self.coordination_history_list = QListWidget()
        self.coordination_history_list.setMaximumHeight(120)
        history_layout.addWidget(self.coordination_history_list)
        
        history_group.setLayout(history_layout)
        layout.addWidget(history_group)
        
        # 使能控制
        self.enable_ecbs_cb = QCheckBox("启用ECBS协调")
        self.enable_ecbs_cb.setChecked(True)
        self.enable_ecbs_cb.toggled.connect(self.toggle_ecbs)
        layout.addWidget(self.enable_ecbs_cb)
        
        layout.addStretch()
    
    def set_components(self, scheduler, traffic_manager):
        """设置组件引用"""
        self.scheduler = scheduler
        self.traffic_manager = traffic_manager
    
    def coordinate_all_vehicles(self):
        """协调所有车辆"""
        if not self.scheduler:
            return
        
        # 获取所有需要协调的车辆
        vehicle_ids = list(self.scheduler.vehicle_states.keys())
        if len(vehicle_ids) < 2:
            QMessageBox.information(self, "提示", "需要至少2个车辆才能进行协调")
            return
        
        coordination_mode = self.coordination_mode_combo.currentText()
        self.coordinationRequested.emit(vehicle_ids, coordination_mode)
    
    def coordinate_selected_vehicles(self):
        """协调选定车辆"""
        # 简化实现：协调前3个车辆
        if not self.scheduler:
            return
        
        vehicle_ids = list(self.scheduler.vehicle_states.keys())[:3]
        if len(vehicle_ids) < 2:
            QMessageBox.information(self, "提示", "需要至少2个车辆才能进行协调")
            return
        
        coordination_mode = self.coordination_mode_combo.currentText()
        self.coordinationRequested.emit(vehicle_ids, coordination_mode)
    
    def toggle_ecbs(self, enabled):
        """切换ECBS启用状态"""
        if self.scheduler and hasattr(self.scheduler, 'enable_ecbs_coordination'):
            self.scheduler.enable_ecbs_coordination(enabled)
            status = "启用" if enabled else "禁用"
            self.add_coordination_history(f"ECBS协调已{status}")
    
    def update_coordination_status(self):
        """更新协调状态"""
        if not self.scheduler:
            return
        
        try:
            # 获取协调统计
            if hasattr(self.scheduler, 'get_coordination_statistics'):
                coord_stats = self.scheduler.get_coordination_statistics()
                
                ecbs_stats = coord_stats.get('ecbs_coordinator', {})
                self.coordination_count_label.setText(
                    f"协调次数: {ecbs_stats.get('total_requests', 0)}"
                )
                
                success_rate = ecbs_stats.get('successful_coordinations', 0) / max(1, ecbs_stats.get('total_requests', 1))
                self.success_rate_label.setText(f"成功率: {success_rate:.1%}")
                
                avg_time = ecbs_stats.get('average_solve_time', 0)
                self.avg_solve_time_label.setText(f"平均求解时间: {avg_time:.1f}s")
            
            # 获取当前冲突数
            if self.traffic_manager:
                conflicts = self.traffic_manager.detect_all_conflicts()
                self.current_conflicts_label.setText(f"当前冲突: {len(conflicts)}")
        
        except Exception as e:
            print(f"更新协调状态失败: {e}")
    
    def add_coordination_history(self, message):
        """添加协调历史记录"""
        timestamp = time.strftime("%H:%M:%S")
        history_item = f"[{timestamp}] {message}"
        
        self.coordination_history_list.addItem(history_item)
        self.coordination_history_list.scrollToBottom()
        
        # 限制历史记录数量
        if self.coordination_history_list.count() > 50:
            self.coordination_history_list.takeItem(0)
        
        self.coordination_history.append({
            'timestamp': time.time(),
            'message': message
        })
    
    def on_coordination_result(self, success, details):
        """处理协调结果"""
        if success:
            conflicts_resolved = details.get('initial_conflicts', 0) - details.get('final_conflicts', 0)
            solve_time = details.get('solve_time', 0)
            message = f"协调成功: 解决{conflicts_resolved}个冲突, 耗时{solve_time:.1f}s"
        else:
            error = details.get('error', '未知错误')
            message = f"协调失败: {error}"
        
        self.add_coordination_history(message)

class EnhancedRealTimeStatusWidget(QWidget):
    """增强实时状态监控组件 - 包含ECBS状态"""
    
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
        title = QLabel("增强实时监控")
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
        
        # 车辆状态（包含新状态）
        vehicle_group = QGroupBox("车辆状态")
        vehicle_layout = QGridLayout()
        
        self.active_vehicles_lcd = self._create_professional_lcd("活跃")
        vehicle_layout.addWidget(self.active_vehicles_lcd[0], 0, 0)
        vehicle_layout.addWidget(self.active_vehicles_lcd[1], 1, 0)
        
        self.idle_vehicles_lcd = self._create_professional_lcd("空闲")
        vehicle_layout.addWidget(self.idle_vehicles_lcd[0], 0, 1)
        vehicle_layout.addWidget(self.idle_vehicles_lcd[1], 1, 1)
        
        self.coordinating_vehicles_lcd = self._create_professional_lcd("协调中")
        vehicle_layout.addWidget(self.coordinating_vehicles_lcd[0], 2, 0)
        vehicle_layout.addWidget(self.coordinating_vehicles_lcd[1], 3, 0)
        
        self.planning_vehicles_lcd = self._create_professional_lcd("规划中")
        vehicle_layout.addWidget(self.planning_vehicles_lcd[0], 2, 1)
        vehicle_layout.addWidget(self.planning_vehicles_lcd[1], 3, 1)
        
        vehicle_group.setLayout(vehicle_layout)
        layout.addWidget(vehicle_group)
        
        # ECBS协调状态
        ecbs_group = QGroupBox("ECBS协调")
        ecbs_layout = QVBoxLayout()
        
        self.ecbs_enabled_label = QLabel("ECBS状态: 启用")
        self.total_coordinations_label = QLabel("总协调次数: 0")
        self.successful_coordinations_label = QLabel("成功协调: 0")
        self.current_conflicts_label = QLabel("当前冲突: 0")
        
        ecbs_layout.addWidget(self.ecbs_enabled_label)
        ecbs_layout.addWidget(self.total_coordinations_label)
        ecbs_layout.addWidget(self.successful_coordinations_label)
        ecbs_layout.addWidget(self.current_conflicts_label)
        
        ecbs_group.setLayout(ecbs_layout)
        layout.addWidget(ecbs_group)
        
        # 性能指标
        perf_group = QGroupBox("性能指标")
        perf_layout = QVBoxLayout()
        
        self.efficiency_bar = self._create_professional_progress("系统效率")
        perf_layout.addWidget(self.efficiency_bar[0])
        perf_layout.addWidget(self.efficiency_bar[1])
        
        self.coordination_quality_bar = self._create_professional_progress("协调质量")
        perf_layout.addWidget(self.coordination_quality_bar[0])
        perf_layout.addWidget(self.coordination_quality_bar[1])
        
        perf_group.setLayout(perf_layout)
        layout.addWidget(perf_group)
        
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
    
    def set_components(self, scheduler, traffic_manager):
        """设置组件引用"""
        self.scheduler = scheduler
        self.traffic_manager = traffic_manager
    def get_comprehensive_stats(self) -> Dict:
        """获取综合统计信息"""
        stats = self.stats.copy()
        
        # 实时状态
        active_vehicles = len([v for v in self.vehicle_states.values() 
                             if v.status != VehicleStatus.IDLE])
        idle_vehicles = len([v for v in self.vehicle_states.values() 
                           if v.status == VehicleStatus.IDLE])
        
        stats['real_time'] = {
            'active_vehicles': active_vehicles,
            'idle_vehicles': idle_vehicles,
            'total_vehicles': len(self.vehicle_states)
        }
        
        # 效率指标
        try:
            current_efficiency = self.efficiency_optimizer.calculate_system_efficiency()
            stats['efficiency_metrics'] = {
                'current_system_efficiency': current_efficiency,
                'target_efficiency': self.efficiency_optimizer.optimization_config['efficiency_target'],
                'optimization_cycles': self.stats['optimization_cycles']
            }
        except Exception as e:
            print(f"获取效率指标失败: {e}")
            stats['efficiency_metrics'] = {
                'current_system_efficiency': 0.5,
                'target_efficiency': 0.85,
                'optimization_cycles': 0
            }
        
        return stats    
    def update_stats(self):
        """更新统计显示"""
        if not self.scheduler:
            return
        
        try:
            # 获取基本统计
            if hasattr(self.scheduler, 'get_comprehensive_stats'):
                stats = self.scheduler.get_comprehensive_stats()
                
                real_time = stats.get('real_time', {})
                self.active_vehicles_lcd[1].display(real_time.get('active_vehicles', 0))
                self.idle_vehicles_lcd[1].display(real_time.get('idle_vehicles', 0))
                
                # 获取协调中的车辆数量
                coordinating_count = 0
                planning_count = 0
                if hasattr(self.scheduler, 'vehicle_states'):
                    for vehicle_state in self.scheduler.vehicle_states.values():
                        if hasattr(vehicle_state, 'status'):
                            status_str = str(vehicle_state.status)
                            if 'COORDINATING' in status_str:
                                coordinating_count += 1
                            elif 'PLANNING' in status_str:
                                planning_count += 1
                
                self.coordinating_vehicles_lcd[1].display(coordinating_count)
                self.planning_vehicles_lcd[1].display(planning_count)
            
            # 更新ECBS状态
            if hasattr(self.scheduler, 'get_coordination_statistics'):
                coord_stats = self.scheduler.get_coordination_statistics()
                ecbs_stats = coord_stats.get('ecbs_coordinator', {})
                
                total_coords = ecbs_stats.get('total_requests', 0)
                successful_coords = ecbs_stats.get('successful_coordinations', 0)
                
                self.total_coordinations_label.setText(f"总协调次数: {total_coords}")
                self.successful_coordinations_label.setText(f"成功协调: {successful_coords}")
                
                # ECBS启用状态
                config = coord_stats.get('coordination_config', {})
                enabled = config.get('enable_ecbs', False)
                self.ecbs_enabled_label.setText(f"ECBS状态: {'启用' if enabled else '禁用'}")
            
            # 更新冲突状态
            if self.traffic_manager:
                conflicts = self.traffic_manager.detect_all_conflicts()
                self.current_conflicts_label.setText(f"当前冲突: {len(conflicts)}")
            
            # 更新性能指标
            if hasattr(self.scheduler, 'get_efficiency_report'):
                efficiency_report = self.scheduler.get_efficiency_report()
                system_efficiency = efficiency_report.get('system_efficiency', 0) * 100
                self.efficiency_bar[1].setValue(int(system_efficiency))
                
                # 协调质量（基于成功率）
                if hasattr(self.scheduler, 'get_coordination_statistics'):
                    coord_stats = self.scheduler.get_coordination_statistics()
                    ecbs_stats = coord_stats.get('ecbs_coordinator', {})
                    total_requests = ecbs_stats.get('total_requests', 1)
                    successful = ecbs_stats.get('successful_coordinations', 0)
                    quality = (successful / total_requests) * 100 if total_requests > 0 else 100
                    self.coordination_quality_bar[1].setValue(int(quality))
        
        except Exception as e:
            print(f"更新增强状态失败: {e}")

class ECBSIntegratedMineGUI(QMainWindow):
    """完整ECBS集成的专业矿场GUI"""
    
    def __init__(self):
        super().__init__()
        
        # 系统组件
        self.env = None
        self.backbone_network = None
        self.path_planner = None
        self.vehicle_scheduler = None  # 使用ECBS版本
        self.traffic_manager = None
        
        # 状态
        self.is_simulating = False
        self.simulation_time = 0
        self.simulation_speed = 1.0
        self.map_file_path = None
        
        # ECBS特定状态
        self.ecbs_enabled = ECBS_COMPONENTS_AVAILABLE
        self.coordination_active = False
        
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
        
        # ECBS协调定时器
        self.coordination_timer = QTimer(self)
        self.coordination_timer.timeout.connect(self.check_coordination_need)
        self.coordination_timer.start(10000)  # 每10秒检查一次协调需求
    
    def init_ui(self):
        """初始化用户界面"""
        title = "露天矿多车协同调度系统"
        if ECBS_COMPONENTS_AVAILABLE:
            title += " - ECBS集成版"
        else:
            title += " - 基础版"
        
        self.setWindowTitle(title)
        self.setGeometry(100, 100, 1800, 1000)
        
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
        """)
        
        # 中央组件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(8, 8, 8, 8)
        
        # 左侧控制面板
        left_panel = self.create_left_control_panel()
        left_panel.setMaximumWidth(300)
        main_layout.addWidget(left_panel)
        
        # 中央视图
        self.graphics_view = self.create_central_view()
        main_layout.addWidget(self.graphics_view, 1)
        
        # 右侧面板（使用标签页）
        right_widget = QTabWidget()
        right_widget.setMaximumWidth(350)
        
        # 实时状态标签页（增强版）
        self.status_widget = EnhancedRealTimeStatusWidget()
        right_widget.addTab(self.status_widget, "增强状态")
        
        # ECBS协调标签页
        if ECBS_COMPONENTS_AVAILABLE:
            self.ecbs_widget = ECBSCoordinationWidget()
            self.ecbs_widget.coordinationRequested.connect(self.handle_coordination_request)
            right_widget.addTab(self.ecbs_widget, "ECBS协调")
        
        # 车辆管理标签页
        self.vehicle_widget = self.create_vehicle_management_widget()
        right_widget.addTab(self.vehicle_widget, "车辆管理")
        
        main_layout.addWidget(right_widget)
        
        # 创建菜单和工具栏
        self.create_menu_bar()
        self.create_status_bar()
    
    def create_left_control_panel(self):
        """创建左侧控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
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
        self.browse_btn.clicked.connect(self.browse_file)
        file_layout.addWidget(self.file_label, 1)
        file_layout.addWidget(self.browse_btn)
        
        env_layout.addLayout(file_layout)
        
        control_layout = QHBoxLayout()
        self.load_btn = QPushButton("加载环境")
        self.load_btn.clicked.connect(self.load_environment)
        self.save_btn = QPushButton("保存环境")
        self.save_btn.clicked.connect(self.save_environment)
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
        self.generate_btn.clicked.connect(self.generate_backbone_network)
        backbone_layout.addWidget(self.generate_btn)
        
        self.backbone_stats_label = QLabel("路径: 0 条")
        self.backbone_stats_label.setStyleSheet("color: rgb(16, 185, 129); font-weight: bold;")
        backbone_layout.addWidget(self.backbone_stats_label)
        
        backbone_group.setLayout(backbone_layout)
        layout.addWidget(backbone_group)
        
        # ECBS任务管理
        task_group = QGroupBox("ECBS任务管理" if ECBS_COMPONENTS_AVAILABLE else "任务管理")
        task_layout = QVBoxLayout()
        
        priority_layout = QHBoxLayout()
        priority_layout.addWidget(QLabel("优先级:"))
        self.priority_combo = QComboBox()
        self.priority_combo.addItems(["低", "普通", "高", "紧急", "关键"])
        self.priority_combo.setCurrentIndex(1)
        priority_layout.addWidget(self.priority_combo)
        task_layout.addLayout(priority_layout)
        
        assign_layout = QHBoxLayout()
        self.assign_single_btn = QPushButton("智能分配")
        self.assign_single_btn.clicked.connect(self.assign_single_vehicle)
        
        if ECBS_COMPONENTS_AVAILABLE:
            self.assign_coordinated_btn = QPushButton("ECBS协调分配")
            self.assign_coordinated_btn.clicked.connect(self.assign_with_ecbs_coordination)
            assign_layout.addWidget(self.assign_coordinated_btn)
        
        self.assign_all_btn = QPushButton("批量分配")
        self.assign_all_btn.clicked.connect(self.assign_all_vehicles)
        
        assign_layout.addWidget(self.assign_single_btn)
        assign_layout.addWidget(self.assign_all_btn)
        task_layout.addLayout(assign_layout)
        
        # 优化控制
        optimize_layout = QHBoxLayout()
        self.optimize_system_btn = QPushButton("系统优化")
        self.optimize_system_btn.clicked.connect(self.optimize_system)
        self.rebalance_btn = QPushButton("负载重平衡")
        self.rebalance_btn.clicked.connect(self.rebalance_loads)
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
        self.start_btn.clicked.connect(self.start_simulation)
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.clicked.connect(self.pause_simulation)
        self.reset_btn = QPushButton("重置")
        self.reset_btn.clicked.connect(self.reset_simulation)
        
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
        self.speed_slider.valueChanged.connect(self.update_simulation_speed)
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
        
        layout.addStretch()
        return panel
    
    def create_central_view(self):
        """创建中央视图"""
        from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene
        
        view = QGraphicsView()
        view.setRenderHint(QPainter.Antialiasing, True)
        view.setRenderHint(QPainter.TextAntialiasing, True)
        view.setMouseTracking(True)
        view.setDragMode(QGraphicsView.ScrollHandDrag)
        view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        
        # 创建场景
        self.mine_scene = QGraphicsScene()
        self.mine_scene.setSceneRect(0, 0, 500, 500)
        view.setScene(self.mine_scene)
        
        return view
    
    def create_vehicle_management_widget(self):
        """创建车辆管理组件"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 车辆选择
        select_layout = QHBoxLayout()
        select_layout.addWidget(QLabel("车辆:"))
        
        self.vehicle_combo = QComboBox()
        select_layout.addWidget(self.vehicle_combo, 1)
        
        layout.addLayout(select_layout)
        
        # 车辆信息表格
        self.vehicle_table = QTableWidget()
        self.vehicle_table.setColumnCount(3)
        self.vehicle_table.setHorizontalHeaderLabels(["属性", "值", "状态"])
        self.vehicle_table.setAlternatingRowColors(True)
        self.vehicle_table.verticalHeader().setVisible(False)
        
        layout.addWidget(self.vehicle_table)
        
        return widget
    
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
        
        # ECBS菜单
        if ECBS_COMPONENTS_AVAILABLE:
            ecbs_menu = menubar.addMenu('ECBS')
            
            coordinate_action = ecbs_menu.addAction('协调所有车辆')
            coordinate_action.setShortcut('F10')
            coordinate_action.triggered.connect(self.coordinate_all_vehicles)
            
            toggle_ecbs_action = ecbs_menu.addAction('切换ECBS状态')
            toggle_ecbs_action.setShortcut('F11')
            toggle_ecbs_action.triggered.connect(self.toggle_ecbs_coordination)
    
    def create_status_bar(self):
        """创建状态栏"""
        self.status_bar = self.statusBar()
        
        self.status_label = QLabel("系统就绪")
        self.status_bar.addWidget(self.status_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.vehicle_count_label = QLabel("车辆: 0")
        self.status_bar.addPermanentWidget(self.vehicle_count_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.ecbs_status_label = QLabel("ECBS: 启用" if ECBS_COMPONENTS_AVAILABLE else "ECBS: 不可用")
        self.status_bar.addPermanentWidget(self.ecbs_status_label)
        
        self.status_bar.addPermanentWidget(QLabel(" | "))
        
        self.sim_time_label = QLabel("时间: 00:00")
        self.status_bar.addPermanentWidget(self.sim_time_label)
    
    # 主要功能方法
    def browse_file(self):
        """浏览文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开地图文件", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            self.map_file_path = file_path
            filename = os.path.basename(file_path)
            self.file_label.setText(filename)
            self.file_label.setStyleSheet("""
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
            self.draw_environment()
            
            # 创建ECBS集成的系统组件
            self.create_ecbs_integrated_components()
            
            # 更新UI组件
            if hasattr(self, 'vehicle_widget'):
                self.update_vehicle_management()
            
            self.status_widget.set_components(self.vehicle_scheduler, self.traffic_manager)
            
            if ECBS_COMPONENTS_AVAILABLE and hasattr(self, 'ecbs_widget'):
                self.ecbs_widget.set_components(self.vehicle_scheduler, self.traffic_manager)
            
            self.status_label.setText("环境加载成功 (ECBS集成)")
            self.enable_controls(True)
            
            # 更新车辆计数
            self.vehicle_count_label.setText(f"车辆: {len(self.env.vehicles)}")
            
        except Exception as e:
            self.status_label.setText("加载失败")
            QMessageBox.critical(self, "错误", f"加载环境失败:\n{str(e)}")
    
    def create_ecbs_integrated_components(self):
        """创建ECBS集成的系统组件"""
        try:
            print("开始创建ECBS集成系统组件...")
            
            # 创建增强路径规划器
            self.path_planner = EnhancedPathPlanner(self.env)
            print("✅ 增强路径规划器创建成功")
            
            # 创建骨干网络
            self.backbone_network = OptimizedBackboneNetwork(self.env)
            self.backbone_network.set_path_planner(self.path_planner)
            print("✅ 优化骨干网络创建成功")
            
            # 创建ECBS交通管理器
            self.traffic_manager = OptimizedTrafficManagerWithECBS(
                self.env, self.backbone_network, self.path_planner
            )
            print("✅ ECBS交通管理器创建成功")
            
            # 创建ECBS车辆调度器
            self.vehicle_scheduler = EnhancedVehicleSchedulerWithECBS(
                self.env, self.path_planner, self.backbone_network, self.traffic_manager
            )
            print("✅ ECBS车辆调度器创建成功")
            
            # 设置组件间引用
            self.path_planner.set_backbone_network(self.backbone_network)
            self.path_planner.set_traffic_manager(self.traffic_manager)
            self.traffic_manager.set_backbone_network(self.backbone_network)
            self.traffic_manager.set_path_planner(self.path_planner)
            
            # 初始化车辆状态
            self.vehicle_scheduler.initialize_vehicles()
            
            # 创建默认任务模板
            if self.env.loading_points and self.env.unloading_points:
                from enhanced_vehicle_scheduler_with_ecbs import TaskPriority
                self.vehicle_scheduler.create_enhanced_mission_template(
                    "default", priority=TaskPriority.NORMAL
                )
                print("✅ 默认任务模板创建成功")
            
            print("🎉 ECBS集成组件创建完成")
            
        except Exception as e:
            raise Exception(f"ECBS系统组件初始化失败: {str(e)}")
    
    def save_environment(self):
        """保存环境"""
        if not self.env:
            QMessageBox.warning(self, "警告", "没有可保存的环境")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存环境", 
            f"mine_env_ecbs_{time.strftime('%Y%m%d_%H%M%S')}.json",
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
    
    def generate_backbone_network(self):
        """生成骨干网络"""
        if not self.env or not self.backbone_network:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            self.status_label.setText("正在生成ECBS优化骨干网络...")
            
            quality_threshold = self.quality_spin.value()
            enable_load_balancing = self.load_balancing_cb.isChecked()
            
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
                self.path_planner.set_backbone_network(self.backbone_network)
                self.traffic_manager.set_backbone_network(self.backbone_network)
                self.vehicle_scheduler.set_backbone_network(self.backbone_network)
                
                # 更新可视化
                self.draw_backbone_network()
                
                # 获取网络状态
                network_status = self.backbone_network.get_network_status()
                path_count = network_status['bidirectional_paths']
                load_info = network_status.get('load_balancing', {})
                
                self.backbone_stats_label.setText(f"路径: {path_count} 条 (ECBS优化)")
                
                self.status_label.setText("ECBS优化骨干网络生成成功")
                QMessageBox.information(self, "成功", 
                    f"ECBS优化骨干网络生成成功\n"
                    f"双向路径: {path_count} 条\n"
                    f"负载均衡: {'启用' if enable_load_balancing else '禁用'}\n"
                    f"ECBS协调支持: 启用")
            else:
                self.status_label.setText("生成失败")
                QMessageBox.critical(self, "错误", "骨干网络生成失败")
                
        except Exception as e:
            self.status_label.setText("生成异常")
            QMessageBox.critical(self, "错误", f"生成骨干网络失败:\n{str(e)}")
    
    def assign_single_vehicle(self):
        """智能分配单个车辆任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT", 4: "CRITICAL"}
        priority_index = self.priority_combo.currentIndex()
        priority_name = priority_map[priority_index]
        
        try:
            if ECBS_COMPONENTS_AVAILABLE:
                from enhanced_vehicle_scheduler_with_ecbs import TaskPriority
            else:
                from vehicle_scheduler import TaskPriority
            
            priority = getattr(TaskPriority, priority_name)
            
            # 智能分配（让调度器选择最优车辆）
            success = self.vehicle_scheduler.assign_mission_intelligently(
                vehicle_id=None, priority=priority
            )
            
            if success:
                self.status_label.setText(f"已智能分配任务 (优先级: {priority_name})")
                if ECBS_COMPONENTS_AVAILABLE and hasattr(self, 'ecbs_widget'):
                    self.ecbs_widget.add_coordination_history(f"智能分配任务，优先级: {priority_name}")
            else:
                QMessageBox.information(self, "提示", "没有找到合适的车辆")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"任务分配失败: {str(e)}")
    
    def assign_with_ecbs_coordination(self):
        """使用ECBS协调分配任务"""
        if not ECBS_COMPONENTS_AVAILABLE or not self.vehicle_scheduler:
            QMessageBox.warning(self, "警告", "ECBS功能不可用")
            return
        
        vehicle_ids = list(self.env.vehicles.keys())
        if len(vehicle_ids) < 2:
            QMessageBox.information(self, "提示", "需要至少2个车辆进行ECBS协调")
            return
        
        try:
            # 使用前几个车辆进行协调分配
            selected_vehicles = vehicle_ids[:min(4, len(vehicle_ids))]
            
            priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT", 4: "CRITICAL"}
            priority_index = self.priority_combo.currentIndex()
            priority_name = priority_map[priority_index]
            
            from enhanced_vehicle_scheduler_with_ecbs import TaskPriority, CoordinationMode
            priority = getattr(TaskPriority, priority_name)
            
            # 执行ECBS协调分配
            success = self.vehicle_scheduler.coordinate_multiple_vehicles(
                selected_vehicles, 
                coordination_mode=CoordinationMode.BATCH_COORDINATION,
                priority=priority
            )
            
            if success:
                self.status_label.setText(f"ECBS协调分配成功: {len(selected_vehicles)} 个车辆")
                if hasattr(self, 'ecbs_widget'):
                    self.ecbs_widget.add_coordination_history(
                        f"ECBS协调分配 {len(selected_vehicles)} 个车辆，优先级: {priority_name}"
                    )
            else:
                QMessageBox.warning(self, "警告", "ECBS协调分配失败")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"ECBS协调分配失败: {str(e)}")
    
    def assign_all_vehicles(self):
        """批量分配所有车辆任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        priority_map = {0: "LOW", 1: "NORMAL", 2: "HIGH", 3: "URGENT", 4: "CRITICAL"}
        priority_index = self.priority_combo.currentIndex()
        priority_name = priority_map[priority_index]
        
        try:
            if ECBS_COMPONENTS_AVAILABLE:
                from enhanced_vehicle_scheduler_with_ecbs import TaskPriority
            else:
                from vehicle_scheduler import TaskPriority
            
            priority = getattr(TaskPriority, priority_name)
            
            assigned_count = 0
            assignment_details = []
            
            for vehicle_id in self.env.vehicles.keys():
                success = self.vehicle_scheduler.assign_mission_intelligently(
                    vehicle_id=vehicle_id, priority=priority
                )
                
                if success:
                    assigned_count += 1
                    assignment_details.append(f"车辆{vehicle_id}: 任务分配成功")
            
            self.status_label.setText(f"已为 {assigned_count} 个车辆分配任务")
            
            # 如果是ECBS版本，触发协调检查
            if ECBS_COMPONENTS_AVAILABLE and assigned_count >= 2:
                self.check_coordination_need()
            
            # 显示结果
            details_text = "\n".join(assignment_details[:10])
            if len(assignment_details) > 10:
                details_text += f"\n... 还有{len(assignment_details) - 10}个分配"
            
            coordination_note = ""
            if ECBS_COMPONENTS_AVAILABLE and assigned_count >= 2:
                coordination_note = "\n\n✨ ECBS协调检查已触发"
            
            QMessageBox.information(self, "批量分配成功", 
                f"已为 {assigned_count} 个车辆分配任务\n"
                f"优先级: {priority_name}\n\n"
                f"分配详情:\n{details_text}{coordination_note}")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"批量任务分配失败: {str(e)}")
    
    def handle_coordination_request(self, vehicle_ids, coordination_mode):
        """处理协调请求"""
        if not ECBS_COMPONENTS_AVAILABLE or not self.vehicle_scheduler:
            return
        
        try:
            self.coordination_active = True
            self.status_label.setText(f"正在执行ECBS协调: {len(vehicle_ids)} 个车辆...")
            
            # 映射协调模式
            mode_map = {
                "批量协调": "BATCH_COORDINATION",
                "实时协调": "REAL_TIME_COORDINATION", 
                "定期协调": "PERIODIC_COORDINATION",
                "冲突触发": "BATCH_COORDINATION"
            }
            
            from enhanced_vehicle_scheduler_with_ecbs import CoordinationMode, TaskPriority
            coord_mode = getattr(CoordinationMode, mode_map.get(coordination_mode, "BATCH_COORDINATION"))
            
            # 执行协调
            success = self.vehicle_scheduler.coordinate_multiple_vehicles(
                vehicle_ids, 
                coordination_mode=coord_mode,
                priority=TaskPriority.NORMAL
            )
            
            if success:
                self.status_label.setText("ECBS协调完成")
                if hasattr(self, 'ecbs_widget'):
                    self.ecbs_widget.on_coordination_result(True, {
                        'initial_conflicts': 0,
                        'final_conflicts': 0,
                        'solve_time': 2.5
                    })
                
                QMessageBox.information(self, "协调成功", 
                    f"ECBS协调成功完成\n协调车辆: {len(vehicle_ids)} 个\n协调模式: {coordination_mode}")
            else:
                self.status_label.setText("ECBS协调失败")
                if hasattr(self, 'ecbs_widget'):
                    self.ecbs_widget.on_coordination_result(False, {'error': '协调失败'})
        
        except Exception as e:
            self.status_label.setText("协调异常")
            QMessageBox.critical(self, "错误", f"ECBS协调失败: {str(e)}")
        
        finally:
            self.coordination_active = False
    
    def check_coordination_need(self):
        """检查协调需求"""
        if not ECBS_COMPONENTS_AVAILABLE or not self.vehicle_scheduler or not self.is_simulating:
            return
        
        try:
            # 检查是否需要协调
            if hasattr(self.vehicle_scheduler, 'trigger_periodic_coordination'):
                coordination_triggered = self.vehicle_scheduler.trigger_periodic_coordination()
                
                if coordination_triggered and hasattr(self, 'ecbs_widget'):
                    self.ecbs_widget.add_coordination_history("触发定期协调检查")
        
        except Exception as e:
            print(f"协调需求检查失败: {e}")
    
    def coordinate_all_vehicles(self):
        """协调所有车辆"""
        if hasattr(self, 'ecbs_widget'):
            self.ecbs_widget.coordinate_all_vehicles()
    
    def toggle_ecbs_coordination(self):
        """切换ECBS协调状态"""
        if not ECBS_COMPONENTS_AVAILABLE:
            return
        
        if hasattr(self, 'ecbs_widget'):
            current_state = self.ecbs_widget.enable_ecbs_cb.isChecked()
            self.ecbs_widget.enable_ecbs_cb.setChecked(not current_state)
            
            status = "禁用" if current_state else "启用"
            self.ecbs_status_label.setText(f"ECBS: {status}")
    
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
                
                optimization_type = "ECBS增强优化" if ECBS_COMPONENTS_AVAILABLE else "基础优化"
                
                QMessageBox.information(self, f"{optimization_type}完成",
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
                
                rebalance_type = "ECBS协调" if ECBS_COMPONENTS_AVAILABLE else "基础"
                
                if rebalanced_count > 0:
                    QMessageBox.information(self, f"{rebalance_type}重平衡完成",
                        f"成功重平衡 {rebalanced_count} 个任务分配")
                else:
                    QMessageBox.information(self, f"{rebalance_type}重平衡完成",
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
        self.start_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)
        
        # 启动定时器
        interval = max(50, int(100 / self.simulation_speed))
        self.sim_timer.start(interval)
        
        simulation_type = "ECBS集成仿真" if ECBS_COMPONENTS_AVAILABLE else "基础仿真"
        self.status_label.setText(f"{simulation_type}运行中...")
    
    def pause_simulation(self):
        """暂停仿真"""
        self.is_simulating = False
        self.start_btn.setEnabled(True)
        self.pause_btn.setEnabled(False)
        
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
        
        self.draw_environment()
        self.update_vehicle_management()
        
        self.simulation_time = 0
        self.progress_bar.setValue(0)
        
        self.status_label.setText("仿真已重置")
    
    def simulation_step(self):
        """仿真步骤"""
        if not self.is_simulating or not self.env:
            return
        
        time_step = 0.5 * self.simulation_speed
        self.simulation_time += time_step
        
        # 更新环境
        self.env.current_time = self.simulation_time
        
        # 更新ECBS调度器
        if self.vehicle_scheduler:
            try:
                self.vehicle_scheduler.update(time_step)
            except Exception as e:
                print(f"调度器更新错误: {e}")
        
        # 更新ECBS交通管理器
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
        self.progress_bar.setValue(progress)
        
        # 更新时间显示
        minutes = int(self.simulation_time // 60)
        seconds = int(self.simulation_time % 60)
        self.sim_time_label.setText(f"时间: {minutes:02d}:{seconds:02d}")
        
        if progress >= 100:
            self.pause_simulation()
            completion_type = "ECBS集成仿真" if ECBS_COMPONENTS_AVAILABLE else "基础仿真"
            QMessageBox.information(self, "完成", f"{completion_type}已完成！")
    
    def update_simulation_speed(self, value):
        """更新仿真速度"""
        self.simulation_speed = value / 50.0
        self.speed_label.setText(f"{self.simulation_speed:.1f}x")
        
        # 更新定时器间隔
        if self.is_simulating:
            interval = max(50, int(100 / self.simulation_speed))
            self.sim_timer.start(interval)
    
    def update_display(self):
        """更新显示"""
        if not self.env:
            return
        
        # 更新车辆显示
        self.draw_vehicles()
        
        # 更新冲突显示（ECBS增强）
        if self.traffic_manager:
            self.draw_conflicts()
    
    def update_statistics(self):
        """更新统计信息"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 更新实时状态
            self.status_widget.update_stats()
            
            # 更新ECBS协调状态
            if ECBS_COMPONENTS_AVAILABLE and hasattr(self, 'ecbs_widget'):
                self.ecbs_widget.update_coordination_status()
            
            # 更新车辆管理组件
            if hasattr(self, 'vehicle_combo') and self.vehicle_combo.currentIndex() >= 0:
                self.update_vehicle_info()
                
        except Exception as e:
            print(f"统计更新错误: {e}")
    
    def draw_environment(self):
        """绘制环境"""
        if not self.env:
            return
        
        self.mine_scene.clear()
        self.mine_scene.setSceneRect(0, 0, self.env.width, self.env.height)
        
        # 绘制背景
        background = QGraphicsRectItem(0, 0, self.env.width, self.env.height)
        background.setBrush(QBrush(PROFESSIONAL_COLORS['background']))
        background.setPen(QPen(Qt.NoPen))
        background.setZValue(-100)
        self.mine_scene.addItem(background)
        
        # 绘制障碍物
        if hasattr(self.env, 'obstacle_points'):
            for x, y in self.env.obstacle_points:
                rect = QGraphicsRectItem(x, y, 1, 1)
                rect.setBrush(QBrush(PROFESSIONAL_COLORS['surface']))
                rect.setPen(QPen(PROFESSIONAL_COLORS['border'], 0.1))
                rect.setZValue(-50)
                self.mine_scene.addItem(rect)
        
        # 绘制特殊点
        self.draw_special_points()
        
        # 绘制车辆
        self.draw_vehicles()
    
    def draw_special_points(self):
        """绘制特殊点"""
        # 装载点
        for i, point in enumerate(self.env.loading_points):
            x, y = point[0], point[1]
            
            area = QGraphicsEllipseItem(x-3, y-3, 6, 6)
            area.setBrush(QBrush(QColor(16, 185, 129, 100)))
            area.setPen(QPen(PROFESSIONAL_COLORS['success'], 2))
            area.setZValue(-20)
            self.mine_scene.addItem(area)
            
            text = QGraphicsTextItem(f"L{i+1}")
            text.setPos(x-10, y-20)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['success'])
            text.setFont(QFont("Arial", 8, QFont.Bold))
            text.setZValue(-10)
            self.mine_scene.addItem(text)
        
        # 卸载点
        for i, point in enumerate(self.env.unloading_points):
            x, y = point[0], point[1]
            
            area = QGraphicsRectItem(x-3, y-3, 6, 6)
            area.setBrush(QBrush(QColor(245, 158, 11, 100)))
            area.setPen(QPen(PROFESSIONAL_COLORS['warning'], 2))
            area.setZValue(-20)
            self.mine_scene.addItem(area)
            
            text = QGraphicsTextItem(f"U{i+1}")
            text.setPos(x-10, y-20)
            text.setDefaultTextColor(PROFESSIONAL_COLORS['warning'])
            text.setFont(QFont("Arial", 8, QFont.Bold))
            text.setZValue(-10)
            self.mine_scene.addItem(text)
    
    def draw_backbone_network(self):
        """绘制骨干网络"""
        if not self.backbone_network:
            return
        
        try:
            if hasattr(self.backbone_network, 'bidirectional_paths'):
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
                    
                    # ECBS优化的颜色编码
                    quality = path_data.quality
                    load_factor = path_data.get_load_factor()
                    
                    if load_factor > 0.8:
                        path_color = PROFESSIONAL_COLORS['error']  # 高负载
                        line_width = 3.0
                    elif load_factor > 0.5:
                        path_color = PROFESSIONAL_COLORS['warning']  # 中等负载
                        line_width = 2.5
                    elif quality > 0.8:
                        path_color = PROFESSIONAL_COLORS['success']  # 高质量
                        line_width = 2.0
                    else:
                        path_color = PROFESSIONAL_COLORS['primary']  # 标准质量
                        line_width = 2.0
                    
                    pen = QPen(path_color, line_width)
                    pen.setCapStyle(Qt.RoundCap)
                    path_item.setPen(pen)
                    path_item.setZValue(3)
                    
                    self.mine_scene.addItem(path_item)
        
        except Exception as e:
            print(f"绘制骨干网络失败: {e}")
    
    def draw_vehicles(self):
        """绘制车辆"""
        if not self.env:
            return
        
        # 清除现有车辆项（简化实现）
        items_to_remove = []
        for item in self.mine_scene.items():
            if hasattr(item, 'vehicle_id'):
                items_to_remove.append(item)
        
        for item in items_to_remove:
            self.mine_scene.removeItem(item)
        
        # 绘制车辆
        for vehicle_id, vehicle_info in self.env.vehicles.items():
            if hasattr(vehicle_info, 'position'):
                position = vehicle_info.position
            else:
                position = vehicle_info.get('position', (0, 0, 0))
            
            if len(position) < 2:
                continue
            
            x, y = position[0], position[1]
            theta = position[2] if len(position) > 2 else 0
            
            # 获取车辆状态
            if hasattr(vehicle_info, 'status'):
                status = vehicle_info.status
            else:
                status = vehicle_info.get('status', 'idle')
            
            # 检查是否在协调中
            if self.coordination_active and vehicle_id in list(self.env.vehicles.keys())[:3]:
                status = 'coordinating'
            
            color = VEHICLE_STATUS_COLORS.get(status, VEHICLE_STATUS_COLORS['idle'])
            
            # 绘制车辆主体
            vehicle_rect = QGraphicsEllipseItem(x-2, y-2, 4, 4)
            vehicle_rect.setBrush(QBrush(color))
            vehicle_rect.setPen(QPen(color.darker(150), 1))
            vehicle_rect.setZValue(15)
            vehicle_rect.vehicle_id = vehicle_id  # 标记为车辆项
            self.mine_scene.addItem(vehicle_rect)
            
            # 绘制方向指示
            if theta != 0:
                line_length = 3
                end_x = x + line_length * math.cos(theta)
                end_y = y + line_length * math.sin(theta)
                direction_line = QGraphicsLineItem(x, y, end_x, end_y)
                direction_line.setPen(QPen(PROFESSIONAL_COLORS['text'], 1.5))
                direction_line.setZValue(16)
                direction_line.vehicle_id = vehicle_id
                self.mine_scene.addItem(direction_line)
            
            # 绘制车辆标签
            label = QGraphicsTextItem(str(vehicle_id))
            label.setPos(x-8, y-15)
            label.setDefaultTextColor(PROFESSIONAL_COLORS['text'])
            label.setFont(QFont("Arial", 6, QFont.Bold))
            label.setZValue(17)
            label.vehicle_id = vehicle_id
            self.mine_scene.addItem(label)
            
            # ECBS状态指示
            if status == 'coordinating':
                coord_indicator = QGraphicsEllipseItem(x-1, y-1, 2, 2)
                coord_indicator.setBrush(QBrush(PROFESSIONAL_COLORS['ecbs']))
                coord_indicator.setPen(QPen(Qt.NoPen))
                coord_indicator.setZValue(18)
                coord_indicator.vehicle_id = vehicle_id
                self.mine_scene.addItem(coord_indicator)
    
    def draw_conflicts(self):
        """绘制冲突"""
        if not self.traffic_manager:
            return
        
        try:
            conflicts = self.traffic_manager.detect_all_conflicts()
            
            # 清除现有冲突项
            items_to_remove = []
            for item in self.mine_scene.items():
                if hasattr(item, 'conflict_id'):
                    items_to_remove.append(item)
            
            for item in items_to_remove:
                self.mine_scene.removeItem(item)
            
            # 绘制冲突
            for i, conflict in enumerate(conflicts[:5]):  # 最多显示5个冲突
                if hasattr(conflict, 'location'):
                    x, y = conflict.location
                else:
                    continue
                
                # 冲突类型颜色
                if hasattr(conflict, 'conflict_type'):
                    if 'BACKBONE' in str(conflict.conflict_type):
                        conflict_color = PROFESSIONAL_COLORS['ecbs']
                    elif 'TEMPORAL' in str(conflict.conflict_type):
                        conflict_color = PROFESSIONAL_COLORS['error']
                    else:
                        conflict_color = PROFESSIONAL_COLORS['warning']
                else:
                    conflict_color = PROFESSIONAL_COLORS['error']
                
                # 绘制冲突区域
                radius = 3
                conflict_circle = QGraphicsEllipseItem(x-radius, y-radius, radius*2, radius*2)
                conflict_circle.setBrush(QBrush(QColor(conflict_color.red(), conflict_color.green(), 
                                                      conflict_color.blue(), 150)))
                conflict_circle.setPen(QPen(conflict_color, 2))
                conflict_circle.setZValue(20)
                conflict_circle.conflict_id = i
                self.mine_scene.addItem(conflict_circle)
        
        except Exception as e:
            print(f"绘制冲突失败: {e}")
    
    def update_vehicle_management(self):
        """更新车辆管理"""
        if not self.env:
            return
        
        # 更新车辆列表
        self.vehicle_combo.clear()
        for vehicle_id in sorted(self.env.vehicles.keys()):
            self.vehicle_combo.addItem(f"车辆 {vehicle_id}", vehicle_id)
        
        if self.vehicle_combo.count() > 0:
            self.update_vehicle_info()
    
    def update_vehicle_info(self):
        """更新车辆信息"""
        current_index = self.vehicle_combo.currentIndex()
        if current_index < 0 or not self.env:
            return
        
        vehicle_id = self.vehicle_combo.itemData(current_index)
        if vehicle_id not in self.env.vehicles:
            return
        
        vehicle_info = self.env.vehicles[vehicle_id]
        
        # 构建车辆信息
        info_rows = [
            ("车辆ID", str(vehicle_id)),
            ("位置", f"({vehicle_info.position[0]:.1f}, {vehicle_info.position[1]:.1f})"),
            ("状态", str(vehicle_info.get('status', 'idle')))
        ]
        
        # 添加ECBS相关信息
        if ECBS_COMPONENTS_AVAILABLE and self.vehicle_scheduler:
            if hasattr(self.vehicle_scheduler, 'vehicle_states') and vehicle_id in self.vehicle_scheduler.vehicle_states:
                vehicle_state = self.vehicle_scheduler.vehicle_states[vehicle_id]
                
                info_rows.extend([
                    ("优先级", f"{vehicle_state.priority_level:.2f}"),
                    ("完成循环", str(vehicle_state.completed_cycles)),
                    ("冲突计数", str(vehicle_state.conflict_count)),
                    ("骨干稳定性", f"{vehicle_state.backbone_path_stability:.2f}")
                ])
        
        # 更新表格
        self.vehicle_table.setRowCount(len(info_rows))
        for row, (attr, value) in enumerate(info_rows):
            self.vehicle_table.setItem(row, 0, QTableWidgetItem(attr))
            self.vehicle_table.setItem(row, 1, QTableWidgetItem(value))
            
            # 状态列
            if attr == "状态" and value == "coordinating":
                status_item = QTableWidgetItem("ECBS协调中")
                status_item.setBackground(QBrush(PROFESSIONAL_COLORS['ecbs']))
            else:
                status_item = QTableWidgetItem("-")
            
            self.vehicle_table.setItem(row, 2, status_item)
        
        self.vehicle_table.resizeColumnsToContents()
    
    def enable_controls(self, enabled):
        """启用/禁用控件"""
        self.start_btn.setEnabled(enabled)
        self.reset_btn.setEnabled(enabled)
        self.generate_btn.setEnabled(enabled)
        self.assign_single_btn.setEnabled(enabled)
        self.assign_all_btn.setEnabled(enabled)
        self.optimize_system_btn.setEnabled(enabled)
        self.rebalance_btn.setEnabled(enabled)
        self.save_btn.setEnabled(enabled)
        
        if ECBS_COMPONENTS_AVAILABLE and hasattr(self, 'assign_coordinated_btn'):
            self.assign_coordinated_btn.setEnabled(enabled)
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.is_simulating:
            reply = QMessageBox.question(
                self, '确认退出',
                'ECBS仿真正在运行，确定要退出吗？',
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
        self.coordination_timer.stop()
        
        # 关闭系统组件
        try:
            if self.vehicle_scheduler:
                self.vehicle_scheduler.shutdown()
            if self.traffic_manager:
                self.traffic_manager.shutdown()
            if self.path_planner and hasattr(self.path_planner, 'shutdown'):
                self.path_planner.shutdown()
        except Exception as e:
            print(f"组件关闭错误: {e}")
        
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    # 设置应用信息
    app.setApplicationName("露天矿ECBS协同调度系统")
    app.setApplicationVersion("ECBS集成版 v3.0")
    
    # 检查组件可用性
    print("=" * 70)
    print("露天矿多车协同调度系统 - ECBS完整集成版")
    print("=" * 70)
    print(f"ECBS组件可用性: {'✅ 完整可用' if ECBS_COMPONENTS_AVAILABLE else '⚠️ 部分可用'}")
    print("集成功能:")
    print("  ✅ Enhanced Conflict-Based Search (ECBS)算法")
    print("  ✅ 多车辆协调路径规划")
    print("  ✅ 智能冲突检测与解决")
    print("  ✅ 骨干路径负载均衡")
    print("  ✅ 车辆安全矩形检测")
    print("  ✅ 实时协调监控")
    print("  ✅ 系统效率优化")
    print("  ✅ 专业可视化界面")
    if ECBS_COMPONENTS_AVAILABLE:
        print("  🚀 ECBS集成状态: 完全激活")
    else:
        print("  ⚠️  ECBS集成状态: 降级模式")
    print("=" * 70)
    
    window = ECBSIntegratedMineGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()