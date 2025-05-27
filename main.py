#!/usr/bin/env python3
"""
main.py - 精简版露天矿多车协同调度系统
保留全部核心功能，简化GUI复杂度
"""

import sys
import os
import math
import time
import json
from typing import Dict, List, Tuple, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QComboBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QTextEdit, QFileDialog, QMessageBox, QSplitter,
    QGroupBox, QGridLayout, QTabWidget, QTableWidget, QTableWidgetItem,
    QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsEllipseItem,
    QGraphicsRectItem, QGraphicsLineItem, QGraphicsPathItem
)
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter, QPainterPath

# 导入系统组件
from backbone_network import SimplifiedBackbonePathNetwork
from path_planner import SimplifiedPathPlanner
from traffic_manager import OptimizedTrafficManager
from vehicle_scheduler import SimplifiedVehicleScheduler, SimplifiedECBSVehicleScheduler
from environment import OpenPitMineEnv


class SimpleMapView(QGraphicsView):
    """简化的地图视图"""
    
    def __init__(self):
        super().__init__()
        
        # 基本设置
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        
        # 创建场景
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        
        # 数据引用
        self.env = None
        self.backbone_network = None
        
        # 显示选项
        self.show_backbone = True
        self.show_interfaces = False
        self.show_paths = True
        
        # 图形项存储
        self.vehicle_items = {}
        self.path_items = {}
    
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
    
    def redraw_environment(self):
        """重绘环境"""
        self.scene.clear()
        self.vehicle_items.clear()
        self.path_items.clear()
        
        if not self.env:
            return
        
        # 1. 绘制背景
        bg = QGraphicsRectItem(0, 0, self.env.width, self.env.height)
        bg.setBrush(QBrush(QColor(40, 40, 50)))
        bg.setPen(QPen(Qt.NoPen))
        self.scene.addItem(bg)
        
        # 2. 绘制障碍物
        for x, y in self.env.obstacle_points:
            obstacle = QGraphicsRectItem(x, y, 1, 1)
            obstacle.setBrush(QBrush(QColor(100, 100, 100)))
            obstacle.setPen(QPen(Qt.NoPen))
            self.scene.addItem(obstacle)
        
        # 3. 绘制特殊点
        self._draw_special_points()
        
        # 4. 绘制骨干网络
        if self.show_backbone and self.backbone_network:
            self._draw_backbone_network()
        
        # 5. 绘制车辆
        self._draw_vehicles()
        
        # 适应视图
        self.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
    
    def _draw_special_points(self):
        """绘制特殊点"""
        # 装载点
        for i, point in enumerate(self.env.loading_points):
            item = QGraphicsEllipseItem(point[0]-8, point[1]-8, 16, 16)
            item.setBrush(QBrush(QColor(0, 255, 0, 150)))
            item.setPen(QPen(QColor(0, 200, 0), 2))
            self.scene.addItem(item)
            
            # 标签
            text = self.scene.addText(f"L{i+1}", font=self.font())
            text.setPos(point[0]-10, point[1]-20)
            text.setDefaultTextColor(QColor(0, 255, 0))
        
        # 卸载点
        for i, point in enumerate(self.env.unloading_points):
            item = QGraphicsRectItem(point[0]-8, point[1]-8, 16, 16)
            item.setBrush(QBrush(QColor(255, 0, 0, 150)))
            item.setPen(QPen(QColor(200, 0, 0), 2))
            self.scene.addItem(item)
            
            # 标签
            text = self.scene.addText(f"U{i+1}", font=self.font())
            text.setPos(point[0]-10, point[1]-20)
            text.setDefaultTextColor(QColor(255, 0, 0))
    
    def _draw_backbone_network(self):
        """绘制骨干网络"""
        if not hasattr(self.backbone_network, 'backbone_paths'):
            return
        
        for path_id, path_data in self.backbone_network.backbone_paths.items():
            path = path_data.get('path', [])
            if len(path) < 2:
                continue
            
            # 创建路径
            painter_path = QPainterPath()
            painter_path.moveTo(path[0][0], path[0][1])
            
            for point in path[1:]:
                painter_path.lineTo(point[0], point[1])
            
            # 绘制路径
            path_item = QGraphicsPathItem(painter_path)
            pen = QPen(QColor(0, 150, 255, 200), 3)
            path_item.setPen(pen)
            self.scene.addItem(path_item)
        
        # 绘制接口
        if self.show_interfaces and hasattr(self.backbone_network, 'backbone_interfaces'):
            for interface_id, interface in self.backbone_network.backbone_interfaces.items():
                x, y = interface.position[0], interface.position[1]
                item = QGraphicsEllipseItem(x-3, y-3, 6, 6)
                
                if interface.is_available():
                    item.setBrush(QBrush(QColor(255, 255, 0)))
                else:
                    item.setBrush(QBrush(QColor(255, 100, 100)))
                
                item.setPen(QPen(Qt.black, 1))
                self.scene.addItem(item)
    
    def _draw_vehicles(self):
        """绘制车辆"""
        if not self.env or not self.env.vehicles:
            return
        
        colors = [QColor(255, 200, 0), QColor(0, 255, 200), QColor(255, 100, 255),
                 QColor(100, 255, 100), QColor(255, 150, 100)]
        
        for i, (vehicle_id, vehicle_data) in enumerate(self.env.vehicles.items()):
            pos = vehicle_data.get('position', (0, 0, 0))
            status = vehicle_data.get('status', 'idle')
            
            # 车辆主体
            color = colors[i % len(colors)]
            vehicle_item = QGraphicsEllipseItem(pos[0]-4, pos[1]-4, 8, 8)
            vehicle_item.setBrush(QBrush(color))
            vehicle_item.setPen(QPen(Qt.black, 2))
            self.scene.addItem(vehicle_item)
            self.vehicle_items[vehicle_id] = vehicle_item
            
            # 车辆标签
            text = self.scene.addText(str(vehicle_id), font=self.font())
            text.setPos(pos[0]+6, pos[1]-8)
            text.setDefaultTextColor(color)
            
            # 绘制路径
            if self.show_paths and 'path' in vehicle_data and vehicle_data['path']:
                self._draw_vehicle_path(vehicle_id, vehicle_data['path'], color)
    
    def _draw_vehicle_path(self, vehicle_id, path, color):
        """绘制车辆路径"""
        if len(path) < 2:
            return
        
        painter_path = QPainterPath()
        painter_path.moveTo(path[0][0], path[0][1])
        
        for point in path[1:]:
            painter_path.lineTo(point[0], point[1])
        
        path_item = QGraphicsPathItem(painter_path)
        pen = QPen(color, 2)
        pen.setStyle(Qt.DashLine)
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
                item.setRect(pos[0]-4, pos[1]-4, 8, 8)
    
    def wheelEvent(self, event):
        """鼠标滚轮缩放"""
        factor = 1.2 if event.angleDelta().y() > 0 else 1/1.2
        self.scale(factor, factor)


class ControlPanel(QWidget):
    """控制面板"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 文件操作组
        file_group = QGroupBox("文件操作")
        file_layout = QVBoxLayout()
        
        self.file_label = QLabel("未选择文件")
        self.browse_btn = QPushButton("选择地图文件")
        self.load_btn = QPushButton("加载环境")
        self.save_btn = QPushButton("保存环境")
        
        self.browse_btn.clicked.connect(self.main_window.browse_file)
        self.load_btn.clicked.connect(self.main_window.load_environment)
        self.save_btn.clicked.connect(self.main_window.save_environment)
        
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(self.browse_btn)
        file_layout.addWidget(self.load_btn)
        file_layout.addWidget(self.save_btn)
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # 骨干网络组
        backbone_group = QGroupBox("骨干网络")
        backbone_layout = QGridLayout()
        
        backbone_layout.addWidget(QLabel("质量阈值:"), 0, 0)
        self.quality_spin = QDoubleSpinBox()
        self.quality_spin.setRange(0.1, 1.0)
        self.quality_spin.setSingleStep(0.1)
        self.quality_spin.setValue(0.6)
        backbone_layout.addWidget(self.quality_spin, 0, 1)
        
        backbone_layout.addWidget(QLabel("接口间距:"), 1, 0)
        self.interface_spacing_spin = QSpinBox()
        self.interface_spacing_spin.setRange(5, 20)
        self.interface_spacing_spin.setValue(8)
        backbone_layout.addWidget(self.interface_spacing_spin, 1, 1)
        
        self.generate_btn = QPushButton("生成骨干网络")
        self.generate_btn.clicked.connect(self.main_window.generate_backbone_network)
        backbone_layout.addWidget(self.generate_btn, 2, 0, 1, 2)
        
        self.backbone_stats_label = QLabel("路径: 0, 接口: 0")
        backbone_layout.addWidget(self.backbone_stats_label, 3, 0, 1, 2)
        
        backbone_group.setLayout(backbone_layout)
        layout.addWidget(backbone_group)
        
        # 调度控制组
        schedule_group = QGroupBox("调度控制")
        schedule_layout = QVBoxLayout()
        
        self.scheduler_combo = QComboBox()
        self.scheduler_combo.addItems(["标准调度", "ECBS增强"])
        self.scheduler_combo.currentIndexChanged.connect(self.main_window.change_scheduler)
        
        self.assign_all_btn = QPushButton("分配所有车辆任务")
        self.assign_all_btn.clicked.connect(self.main_window.assign_all_vehicles)
        
        schedule_layout.addWidget(QLabel("调度器类型:"))
        schedule_layout.addWidget(self.scheduler_combo)
        schedule_layout.addWidget(self.assign_all_btn)
        schedule_group.setLayout(schedule_layout)
        layout.addWidget(schedule_group)
        
        # 仿真控制组
        sim_group = QGroupBox("仿真控制")
        sim_layout = QVBoxLayout()
        
        # 控制按钮
        button_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始")
        self.pause_btn = QPushButton("暂停")
        self.reset_btn = QPushButton("重置")
        
        self.start_btn.clicked.connect(self.main_window.start_simulation)
        self.pause_btn.clicked.connect(self.main_window.pause_simulation)
        self.reset_btn.clicked.connect(self.main_window.reset_simulation)
        
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.pause_btn)
        button_layout.addWidget(self.reset_btn)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("仿真速度:"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 5.0)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setValue(1.0)
        self.speed_spin.valueChanged.connect(self.main_window.update_simulation_speed)
        speed_layout.addWidget(self.speed_spin)
        
        # 进度条
        self.progress_bar = QProgressBar()
        
        sim_layout.addLayout(button_layout)
        sim_layout.addLayout(speed_layout)
        sim_layout.addWidget(self.progress_bar)
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # 显示选项组
        display_group = QGroupBox("显示选项")
        display_layout = QVBoxLayout()
        
        self.show_backbone_btn = QPushButton("显示/隐藏骨干路径")
        self.show_interfaces_btn = QPushButton("显示/隐藏接口")
        self.show_paths_btn = QPushButton("显示/隐藏车辆路径")
        
        self.show_backbone_btn.clicked.connect(self.main_window.toggle_backbone)
        self.show_interfaces_btn.clicked.connect(self.main_window.toggle_interfaces)
        self.show_paths_btn.clicked.connect(self.main_window.toggle_paths)
        
        display_layout.addWidget(self.show_backbone_btn)
        display_layout.addWidget(self.show_interfaces_btn)
        display_layout.addWidget(self.show_paths_btn)
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        layout.addStretch()


class StatusPanel(QWidget):
    """状态面板"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 系统状态组
        system_group = QGroupBox("系统状态")
        system_layout = QGridLayout()
        
        self.vehicles_label = QLabel("车辆: 0/0")
        self.tasks_label = QLabel("任务: 0/0")
        self.conflicts_label = QLabel("冲突: 0")
        self.backbone_label = QLabel("骨干利用率: 0%")
        
        system_layout.addWidget(QLabel("活跃车辆:"), 0, 0)
        system_layout.addWidget(self.vehicles_label, 0, 1)
        system_layout.addWidget(QLabel("任务状态:"), 1, 0)
        system_layout.addWidget(self.tasks_label, 1, 1)
        system_layout.addWidget(QLabel("当前冲突:"), 2, 0)
        system_layout.addWidget(self.conflicts_label, 2, 1)
        system_layout.addWidget(QLabel("骨干利用率:"), 3, 0)
        system_layout.addWidget(self.backbone_label, 3, 1)
        
        system_group.setLayout(system_layout)
        layout.addWidget(system_group)
        
        # 车辆详情表
        vehicle_group = QGroupBox("车辆详情")
        vehicle_layout = QVBoxLayout()
        
        self.vehicle_table = QTableWidget()
        self.vehicle_table.setColumnCount(4)
        self.vehicle_table.setHorizontalHeaderLabels(["车辆ID", "状态", "位置", "任务"])
        self.vehicle_table.horizontalHeader().setStretchLastSection(True)
        
        vehicle_layout.addWidget(self.vehicle_table)
        vehicle_group.setLayout(vehicle_layout)
        layout.addWidget(vehicle_group)
        
        # 日志
        log_group = QGroupBox("系统日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
    
    def update_status(self, stats):
        """更新状态显示"""
        if not stats:
            return
        
        # 更新基本统计
        real_time = stats.get('real_time', {})
        self.vehicles_label.setText(
            f"{real_time.get('active_vehicles', 0)}/{real_time.get('active_vehicles', 0) + real_time.get('idle_vehicles', 0)}"
        )
        
        completed = stats.get('completed_tasks', 0)
        total = stats.get('total_tasks', 0)
        self.tasks_label.setText(f"{completed}/{total}")
        
        backbone_rate = stats.get('backbone_utilization_rate', 0)
        self.backbone_label.setText(f"{backbone_rate*100:.0f}%")
    
    def update_vehicle_table(self, vehicles, vehicle_states):
        """更新车辆表格"""
        if not vehicles:
            return
        
        self.vehicle_table.setRowCount(len(vehicles))
        
        for row, (vehicle_id, vehicle_data) in enumerate(vehicles.items()):
            # 车辆ID
            self.vehicle_table.setItem(row, 0, QTableWidgetItem(str(vehicle_id)))
            
            # 状态
            status = vehicle_data.get('status', 'idle')
            status_map = {'idle': '空闲', 'moving': '移动', 'loading': '装载', 'unloading': '卸载'}
            self.vehicle_table.setItem(row, 1, QTableWidgetItem(status_map.get(status, status)))
            
            # 位置
            pos = vehicle_data.get('position', (0, 0, 0))
            pos_str = f"({pos[0]:.1f}, {pos[1]:.1f})"
            self.vehicle_table.setItem(row, 2, QTableWidgetItem(pos_str))
            
            # 任务
            task = "无"
            if vehicle_states and vehicle_id in vehicle_states:
                vehicle_state = vehicle_states[vehicle_id]
                if vehicle_state.current_task:
                    task = vehicle_state.current_task.split('_')[-1]  # 简化显示
            
            self.vehicle_table.setItem(row, 3, QTableWidgetItem(task))
    
    def add_log(self, message, level="info"):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        color_map = {
            "error": "red",
            "warning": "orange", 
            "success": "green",
            "info": "black"
        }
        
        color = color_map.get(level, "black")
        formatted_message = f'<span style="color: {color};">[{timestamp}] {message}</span>'
        
        self.log_text.append(formatted_message)
        
        # 滚动到底部
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


class SimplifiedMineGUI(QMainWindow):
    """精简版露天矿调度系统主窗口"""
    
    def __init__(self):
        super().__init__()
        
        # 系统组件
        self.env = None
        self.backbone_network = None
        self.path_planner = None
        self.vehicle_scheduler = None
        self.traffic_manager = None
        
        # GUI状态
        self.is_simulating = False
        self.simulation_speed = 1.0
        self.simulation_time = 0
        self.map_file_path = None
        
        # 初始化UI
        self.init_ui()
        
        # 定时器
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 10 FPS
        
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self.simulation_step)
        
        self.stats_timer = QTimer(self)
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(1000)  # 1秒更新
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("露天矿多车协同调度系统 - 精简版")
        self.setGeometry(100, 100, 1400, 800)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        self.control_panel = ControlPanel(self)
        self.control_panel.setMaximumWidth(300)
        main_layout.addWidget(self.control_panel)
        
        # 中央分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 地图视图
        self.map_view = SimpleMapView()
        splitter.addWidget(self.map_view)
        
        # 右侧状态面板
        self.status_panel = StatusPanel()
        self.status_panel.setMaximumWidth(350)
        splitter.addWidget(self.status_panel)
        
        splitter.setStretchFactor(0, 1)  # 地图视图可伸缩
        splitter.setStretchFactor(1, 0)  # 状态面板固定
        
        main_layout.addWidget(splitter)
        
        # 创建菜单栏
        self.create_menu_bar()
        
        # 状态栏
        self.statusBar().showMessage("系统就绪")
        
        # 初始状态
        self.enable_controls(False)
    
    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu('文件')
        
        open_action = file_menu.addAction('打开地图文件')
        open_action.setShortcut('Ctrl+O')
        open_action.triggered.connect(self.browse_file)
        
        save_action = file_menu.addAction('保存环境')
        save_action.setShortcut('Ctrl+S')
        save_action.triggered.connect(self.save_environment)
        
        file_menu.addSeparator()
        
        exit_action = file_menu.addAction('退出')
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        
        # 仿真菜单
        sim_menu = menubar.addMenu('仿真')
        
        start_action = sim_menu.addAction('开始仿真')
        start_action.setShortcut('F5')
        start_action.triggered.connect(self.start_simulation)
        
        pause_action = sim_menu.addAction('暂停仿真')
        pause_action.setShortcut('F6')
        pause_action.triggered.connect(self.pause_simulation)
        
        reset_action = sim_menu.addAction('重置仿真')
        reset_action.setShortcut('F7')
        reset_action.triggered.connect(self.reset_simulation)
        
        # 帮助菜单
        help_menu = menubar.addMenu('帮助')
        
        about_action = help_menu.addAction('关于')
        about_action.triggered.connect(self.show_about)
    
    # 文件操作方法
    def browse_file(self):
        """浏览文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "打开地图文件", "", "JSON文件 (*.json);;所有文件 (*)"
        )
        
        if file_path:
            self.map_file_path = file_path
            self.control_panel.file_label.setText(os.path.basename(file_path))
            self.status_panel.add_log(f"已选择地图文件: {os.path.basename(file_path)}")
    
    def load_environment(self):
        """加载环境"""
        if not self.map_file_path:
            QMessageBox.warning(self, "警告", "请先选择地图文件")
            return
        
        try:
            self.status_panel.add_log("正在加载环境...")
            
            # 创建环境
            self.env = OpenPitMineEnv()
            if not self.env.load_from_file(self.map_file_path):
                raise Exception("环境加载失败")
            
            # 设置到地图视图
            self.map_view.set_environment(self.env)
            
            # 创建系统组件
            self.create_system_components()
            
            self.status_panel.add_log("环境加载成功", "success")
            self.statusBar().showMessage("环境已加载")
            self.enable_controls(True)
            
        except Exception as e:
            self.status_panel.add_log(f"加载环境失败: {str(e)}", "error")
            QMessageBox.critical(self, "错误", f"加载环境失败:\n{str(e)}")
    
    def save_environment(self):
        """保存环境"""
        if not self.env:
            QMessageBox.warning(self, "警告", "没有加载的环境")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存环境", 
            f"mine_env_{time.strftime('%Y%m%d_%H%M%S')}.json",
            "JSON文件 (*.json)"
        )
        
        if file_path:
            try:
                if self.env.save_to_file(file_path):
                    self.status_panel.add_log(f"环境已保存: {os.path.basename(file_path)}", "success")
                    QMessageBox.information(self, "成功", "环境保存成功")
                else:
                    raise Exception("保存失败")
            except Exception as e:
                self.status_panel.add_log(f"保存环境失败: {str(e)}", "error")
                QMessageBox.critical(self, "错误", f"保存环境失败:\n{str(e)}")
    
    def create_system_components(self):
        """创建系统组件"""
        try:
            # 创建骨干路径网络
            self.backbone_network = SimplifiedBackbonePathNetwork(self.env)
            
            # 创建路径规划器
            self.path_planner = SimplifiedPathPlanner(self.env, self.backbone_network)
            
            # 创建交通管理器
            self.traffic_manager = OptimizedTrafficManager(self.env, self.backbone_network)
            
            # 创建车辆调度器
            self.create_scheduler()
            
            # 初始化车辆状态
            if self.vehicle_scheduler:
                self.vehicle_scheduler.initialize_vehicles()
                if self.env.loading_points and self.env.unloading_points:
                    self.vehicle_scheduler.create_enhanced_mission_template("default")
            
            self.status_panel.add_log("系统组件初始化完成", "success")
            
        except Exception as e:
            self.status_panel.add_log(f"系统组件初始化失败: {str(e)}", "error")
    
    def create_scheduler(self):
        """创建调度器"""
        scheduler_type = self.control_panel.scheduler_combo.currentIndex()
        
        if scheduler_type == 1:  # ECBS增强
            self.vehicle_scheduler = SimplifiedECBSVehicleScheduler(
                self.env, self.path_planner, self.traffic_manager, self.backbone_network
            )
            self.status_panel.add_log("使用ECBS增强调度器", "success")
        else:  # 标准调度
            self.vehicle_scheduler = SimplifiedVehicleScheduler(
                self.env, self.path_planner, self.backbone_network, self.traffic_manager
            )
            self.status_panel.add_log("使用标准调度器", "success")
    
    # 骨干网络操作
    def generate_backbone_network(self):
        """生成骨干路径网络"""
        if not self.env or not self.backbone_network:
            QMessageBox.warning(self, "警告", "请先加载环境")
            return
        
        try:
            self.status_panel.add_log("正在生成骨干路径网络...")
            
            quality_threshold = self.control_panel.quality_spin.value()
            interface_spacing = self.control_panel.interface_spacing_spin.value()
            
            success = self.backbone_network.generate_backbone_network(
                quality_threshold=quality_threshold,
                interface_spacing=interface_spacing
            )
            
            if success:
                # 更新组件引用
                self.path_planner.set_backbone_network(self.backbone_network)
                self.traffic_manager.set_backbone_network(self.backbone_network)
                self.vehicle_scheduler.set_backbone_network(self.backbone_network)
                
                # 更新地图显示
                self.map_view.set_backbone_network(self.backbone_network)
                
                # 更新统计
                path_count = len(self.backbone_network.backbone_paths)
                interface_count = len(self.backbone_network.backbone_interfaces)
                
                self.control_panel.backbone_stats_label.setText(
                    f"路径: {path_count}, 接口: {interface_count}"
                )
                
                self.status_panel.add_log(f"骨干网络生成成功: {path_count}条路径, {interface_count}个接口", "success")
                
                QMessageBox.information(
                    self, "成功", 
                    f"骨干路径网络生成成功！\n路径数量: {path_count}\n接口数量: {interface_count}"
                )
            else:
                self.status_panel.add_log("骨干路径网络生成失败", "error")
                QMessageBox.critical(self, "错误", "骨干路径网络生成失败")
        
        except Exception as e:
            self.status_panel.add_log(f"生成骨干网络失败: {str(e)}", "error")
            QMessageBox.critical(self, "错误", f"生成骨干网络失败:\n{str(e)}")
    
    # 调度操作
    def change_scheduler(self):
        """更改调度器类型"""
        if not self.env:
            return
        
        self.create_scheduler()
        if self.vehicle_scheduler:
            self.vehicle_scheduler.initialize_vehicles()
            if self.env.loading_points and self.env.unloading_points:
                self.vehicle_scheduler.create_enhanced_mission_template("default")
    
    def assign_all_vehicles(self):
        """批量分配任务"""
        if not self.vehicle_scheduler or not self.env:
            return
        
        assigned_count = 0
        for vehicle_id in self.env.vehicles.keys():
            if self.vehicle_scheduler.assign_mission_intelligently(vehicle_id):
                assigned_count += 1
        
        self.status_panel.add_log(f"已为 {assigned_count} 个车辆分配任务", "success")
        QMessageBox.information(self, "成功", f"已为 {assigned_count} 个车辆分配任务")
    
    # 仿真控制
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
        
        self.status_panel.add_log("仿真已开始")
        self.statusBar().showMessage("仿真运行中...")
    
    def pause_simulation(self):
        """暂停仿真"""
        self.is_simulating = False
        self.control_panel.start_btn.setEnabled(True)
        self.control_panel.pause_btn.setEnabled(False)
        
        self.sim_timer.stop()
        
        self.status_panel.add_log("仿真已暂停")
        self.statusBar().showMessage("仿真已暂停")
    
    def reset_simulation(self):
        """重置仿真"""
        if self.is_simulating:
            self.pause_simulation()
        
        if self.env:
            self.env.reset()
        
        if self.vehicle_scheduler:
            self.vehicle_scheduler.initialize_vehicles()
        
        self.map_view.redraw_environment()
        self.control_panel.progress_bar.setValue(0)
        
        self.status_panel.add_log("仿真已重置")
        self.statusBar().showMessage("仿真已重置")
    
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
                self.status_panel.add_log(f"调度器更新错误: {e}", "error")
        
        # 更新进度条
        max_time = 3600  # 1小时
        progress = min(100, int(self.simulation_time * 100 / max_time))
        self.control_panel.progress_bar.setValue(progress)
        
        if progress >= 100:
            self.pause_simulation()
            self.status_panel.add_log("仿真完成", "success")
            QMessageBox.information(self, "完成", "仿真已完成！")
    
    def update_simulation_speed(self):
        """更新仿真速度"""
        self.simulation_speed = self.control_panel.speed_spin.value()
        
        if self.is_simulating:
            interval = max(50, int(100 / self.simulation_speed))
            self.sim_timer.start(interval)
    
    # 显示控制
    def toggle_backbone(self):
        """切换骨干路径显示"""
        self.map_view.show_backbone = not self.map_view.show_backbone
        self.map_view.redraw_environment()
    
    def toggle_interfaces(self):
        """切换接口显示"""
        self.map_view.show_interfaces = not self.map_view.show_interfaces
        self.map_view.redraw_environment()
    
    def toggle_paths(self):
        """切换车辆路径显示"""
        self.map_view.show_paths = not self.map_view.show_paths
        self.map_view.redraw_environment()
    
    # 状态更新
    def update_display(self):
        """更新显示"""
        if not self.env:
            return
        
        # 更新车辆显示
        self.map_view.update_vehicles()
    
    def update_statistics(self):
        """更新统计信息"""
        if not self.vehicle_scheduler:
            return
        
        try:
            # 更新状态面板
            stats = self.vehicle_scheduler.get_comprehensive_stats()
            self.status_panel.update_status(stats)
            
            # 更新车辆表格
            vehicle_states = getattr(self.vehicle_scheduler, 'vehicle_states', {})
            self.status_panel.update_vehicle_table(self.env.vehicles, vehicle_states)
            
            # 更新冲突计数
            if self.traffic_manager:
                conflicts = self.traffic_manager.detect_all_conflicts()
                self.status_panel.conflicts_label.setText(str(len(conflicts)))
            
        except Exception as e:
            pass  # 静默处理统计更新错误
    
    def enable_controls(self, enabled):
        """启用/禁用控件"""
        self.control_panel.start_btn.setEnabled(enabled)
        self.control_panel.reset_btn.setEnabled(enabled)
        self.control_panel.generate_btn.setEnabled(enabled)
        self.control_panel.assign_all_btn.setEnabled(enabled)
        self.control_panel.save_btn.setEnabled(enabled)
    
    def show_about(self):
        """显示关于对话框"""
        QMessageBox.about(
            self, "关于", 
            "露天矿多车协同调度系统 - 精简版\n\n"
            "基于骨干路径和接口系统的智能调度平台\n\n"
            "主要功能:\n"
            "• 优化的RRT路径规划\n"
            "• 骨干路径网络与接口系统\n"
            "• ECBS冲突解决\n"
            "• 智能任务分配\n"
            "• 实时性能监控"
        )
    
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
        
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    
    # 设置应用属性
    app.setApplicationName("露天矿调度系统")
    app.setApplicationVersion("1.0")
    
    # 创建并显示主窗口
    window = SimplifiedMineGUI()
    window.show()
    
    # 运行应用
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()