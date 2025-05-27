"""
enhanced_system_integration.py - 增强系统集成
整合优化后的骨干网络、交通管理器、车辆调度器
"""

import time
import threading
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
import threading
# 导入优化组件
from optimized_backbone_network import OptimizedBackboneNetwork
from traffic_manager import OptimizedTrafficManager
from vehicle_scheduler import EnhancedVehicleScheduler, TaskPriority
from optimized_planner_config import EnhancedPathPlannerWithConfig

@dataclass
class SystemPerformanceMetrics:
    """系统性能指标"""
    timestamp: float
    overall_efficiency: float
    backbone_utilization: float
    conflict_resolution_rate: float
    average_task_completion_time: float
    vehicle_idle_ratio: float
    system_throughput: float  # 单位时间完成的任务数

class EnhancedMineSchedulingSystem:
    """增强版露天矿调度系统 - 完整集成方案"""
    
    def __init__(self, env):
        self.env = env
        
        # 核心组件初始化
        self.path_planner = None
        self.backbone_network = None
        self.traffic_manager = None
        self.vehicle_scheduler = None
        
        # 系统状态
        self.is_running = False
        self.system_start_time = 0
        
        # 性能监控
        self.performance_history = []
        self.monitoring_interval = 30.0  # 30秒
        self.last_monitoring_time = 0
        
        # 系统配置
        self.system_config = {
            'enable_load_balancing': True,
            'enable_conflict_avoidance': True,
            'enable_efficiency_optimization': True,
            'enable_dynamic_priority': True,
            'monitoring_enabled': True,
            'auto_optimization': True,
            'optimization_threshold': 0.7
        }
        
        # 线程管理
        self.monitoring_thread = None
        self.optimization_thread = None
        self.system_lock = threading.RLock()
        
        print("🚀 初始化增强版露天矿调度系统")
    
    def initialize_system(self, quality_threshold: float = 0.6) -> bool:
        """初始化完整系统"""
        try:
            print("📋 开始系统初始化...")
            
            # 1. 初始化路径规划器
            print("  🎯 初始化增强路径规划器...")
            self.path_planner = EnhancedPathPlannerWithConfig(self.env)
            
            # 2. 初始化骨干网络
            print("  🌐 初始化优化骨干网络...")
            self.backbone_network = OptimizedBackboneNetwork(self.env)
            self.backbone_network.set_path_planner(self.path_planner)
            
            # 3. 生成骨干网络
            print("  🔗 生成骨干路径网络...")
            if not self.backbone_network.generate_backbone_network(quality_threshold):
                print("❌ 骨干网络生成失败")
                return False
            
            # 4. 初始化交通管理器
            print("  🚦 初始化智能交通管理器...")
            self.traffic_manager = OptimizedTrafficManager(
                self.env, self.backbone_network, self.path_planner
            )
            
            # 5. 初始化车辆调度器
            print("  🚚 初始化增强车辆调度器...")
            self.vehicle_scheduler = EnhancedVehicleScheduler(
                self.env, self.path_planner, self.backbone_network, self.traffic_manager
            )
            
            # 6. 建立组件间连接
            print("  🔗 建立组件间连接...")
            self._establish_component_connections()
            
            # 7. 初始化车辆状态
            print("  🏁 初始化车辆状态...")
            self.vehicle_scheduler.initialize_vehicles()
            
            # 8. 设置车辆优先级
            self._initialize_vehicle_priorities()
            
            print("✅ 系统初始化完成!")
            return True
            
        except Exception as e:
            print(f"❌ 系统初始化失败: {e}")
            return False
    
    def _establish_component_connections(self):
        """建立组件间连接"""
        # 路径规划器设置骨干网络
        self.path_planner.set_backbone_network(self.backbone_network)
        
        # 交通管理器设置组件
        self.traffic_manager.set_backbone_network(self.backbone_network)
        self.traffic_manager.set_path_planner(self.path_planner)
        
        # 车辆调度器设置组件
        self.vehicle_scheduler.set_backbone_network(self.backbone_network)
    
    def _initialize_vehicle_priorities(self):
        """初始化车辆优先级"""
        for i, vehicle_id in enumerate(self.env.vehicles.keys()):
            # 根据车辆ID分配不同优先级（示例）
            if i % 3 == 0:
                priority = 0.8  # 高优先级
            elif i % 3 == 1:
                priority = 0.5  # 中等优先级
            else:
                priority = 0.3  # 低优先级
            
            self.traffic_manager.set_vehicle_priority(vehicle_id, priority)
            
            if vehicle_id in self.vehicle_scheduler.vehicle_states:
                self.vehicle_scheduler.vehicle_states[vehicle_id].priority_level = priority
    
    def start_system(self) -> bool:
        """启动系统"""
        if self.is_running:
            print("⚠️ 系统已在运行中")
            return False
        
        if not self._validate_system_readiness():
            print("❌ 系统未就绪，无法启动")
            return False
        
        print("🚀 启动增强调度系统...")
        
        with self.system_lock:
            self.is_running = True
            self.system_start_time = time.time()
        
        # 启动监控线程
        if self.system_config['monitoring_enabled']:
            self._start_monitoring_thread()
        
        # 自动分配初始任务
        self._assign_initial_missions()
        
        print("✅ 系统启动成功!")
        return True
    
    def _validate_system_readiness(self) -> bool:
        """验证系统就绪状态"""
        if not self.path_planner:
            print("❌ 路径规划器未初始化")
            return False
        
        if not self.backbone_network:
            print("❌ 骨干网络未初始化")
            return False
        
        if not self.traffic_manager:
            print("❌ 交通管理器未初始化")
            return False
        
        if not self.vehicle_scheduler:
            print("❌ 车辆调度器未初始化")
            return False
        
        if not self.env.vehicles:
            print("❌ 无可用车辆")
            return False
        
        return True
    
    def _start_monitoring_thread(self):
        """启动监控线程"""
        def monitoring_loop():
            while self.is_running:
                try:
                    current_time = time.time()
                    
                    if current_time - self.last_monitoring_time >= self.monitoring_interval:
                        self._collect_performance_metrics()
                        self.last_monitoring_time = current_time
                        
                        # 自动优化检查
                        if self.system_config['auto_optimization']:
                            self._check_and_trigger_optimization()
                    
                    time.sleep(5.0)  # 每5秒检查一次
                    
                except Exception as e:
                    print(f"监控线程错误: {e}")
        
        self.monitoring_thread = threading.Thread(target=monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        print("📊 性能监控已启动")
    
    def _assign_initial_missions(self):
        """分配初始任务"""
        print("📋 分配初始任务...")
        
        # 为每个车辆分配任务，根据优先级分配不同优先级的任务
        for vehicle_id, vehicle_state in self.vehicle_scheduler.vehicle_states.items():
            if vehicle_state.priority_level > 0.7:
                task_priority = TaskPriority.HIGH
            elif vehicle_state.priority_level > 0.4:
                task_priority = TaskPriority.NORMAL
            else:
                task_priority = TaskPriority.LOW
            
            success = self.vehicle_scheduler.assign_mission_intelligently(
                vehicle_id, "default", task_priority
            )
            
            if success:
                print(f"  ✅ 车辆 {vehicle_id} 已分配 {task_priority.name} 优先级任务")
            else:
                print(f"  ❌ 车辆 {vehicle_id} 任务分配失败")
    
    def update_system(self, time_delta: float):
        """系统主更新循环"""
        if not self.is_running:
            return
        
        try:
            with self.system_lock:
                # 1. 更新车辆调度器（包含效率优化）
                self.vehicle_scheduler.update(time_delta)
                
                # 2. 更新交通管理器（包含冲突解决）
                self.traffic_manager.update(time_delta)
                
                # 3. 更新骨干网络负载均衡
                self.backbone_network.update_load_balancing(time_delta)
        
        except Exception as e:
            print(f"系统更新错误: {e}")
    
    def _collect_performance_metrics(self):
        """收集性能指标"""
        try:
            current_time = time.time()
            
            # 获取各组件统计
            scheduler_stats = self.vehicle_scheduler.get_comprehensive_stats()
            traffic_stats = self.traffic_manager.get_statistics()
            backbone_status = self.backbone_network.get_network_status()
            efficiency_report = self.vehicle_scheduler.get_efficiency_report()
            
            # 计算系统级指标
            runtime = current_time - self.system_start_time
            
            # 系统吞吐量（任务/小时）
            completed_tasks = scheduler_stats.get('completed_tasks', 0)
            if runtime > 0:
                throughput = (completed_tasks / runtime) * 3600  # 转换为每小时
            else:
                throughput = 0.0
            
            # 创建性能指标
            metrics = SystemPerformanceMetrics(
                timestamp=current_time,
                overall_efficiency=efficiency_report.get('system_efficiency', 0.0),
                backbone_utilization=backbone_status.get('load_balancing', {}).get('average_path_utilization', 0.0),
                conflict_resolution_rate=self._calculate_conflict_resolution_rate(traffic_stats),
                average_task_completion_time=scheduler_stats.get('average_task_time', 0.0),
                vehicle_idle_ratio=self._calculate_vehicle_idle_ratio(scheduler_stats),
                system_throughput=throughput
            )
            
            # 添加到历史记录
            self.performance_history.append(metrics)
            
            # 限制历史长度
            if len(self.performance_history) > 100:
                self.performance_history = self.performance_history[-50:]
            
            # 输出关键指标
            print(f"📊 系统性能 - 效率: {metrics.overall_efficiency:.2%}, "
                  f"吞吐量: {metrics.system_throughput:.1f} 任务/小时, "
                  f"骨干利用率: {metrics.backbone_utilization:.2%}")
            
        except Exception as e:
            print(f"性能指标收集错误: {e}")
    
    def _calculate_conflict_resolution_rate(self, traffic_stats: Dict) -> float:
        """计算冲突解决率"""
        total_conflicts = traffic_stats.get('total_conflicts', 0)
        resolved_conflicts = traffic_stats.get('resolved_conflicts', 0)
        
        if total_conflicts > 0:
            return resolved_conflicts / total_conflicts
        return 1.0
    
    def _calculate_vehicle_idle_ratio(self, scheduler_stats: Dict) -> float:
        """计算车辆空闲率"""
        real_time = scheduler_stats.get('real_time', {})
        total_vehicles = real_time.get('total_vehicles', 0)
        idle_vehicles = real_time.get('idle_vehicles', 0)
        
        if total_vehicles > 0:
            return idle_vehicles / total_vehicles
        return 0.0
    
    def _check_and_trigger_optimization(self):
        """检查并触发优化"""
        if not self.performance_history:
            return
        
        # 获取最新性能指标
        latest_metrics = self.performance_history[-1]
        
        # 检查是否需要优化
        need_optimization = False
        optimization_reasons = []
        
        # 效率阈值检查
        if latest_metrics.overall_efficiency < self.system_config['optimization_threshold']:
            need_optimization = True
            optimization_reasons.append(f"低效率: {latest_metrics.overall_efficiency:.2%}")
        
        # 空闲率检查
        if latest_metrics.vehicle_idle_ratio > 0.3:
            need_optimization = True
            optimization_reasons.append(f"高空闲率: {latest_metrics.vehicle_idle_ratio:.2%}")
        
        # 冲突解决率检查
        if latest_metrics.conflict_resolution_rate < 0.8:
            need_optimization = True
            optimization_reasons.append(f"低冲突解决率: {latest_metrics.conflict_resolution_rate:.2%}")
        
        # 触发优化
        if need_optimization:
            print(f"🔧 触发系统优化 - 原因: {', '.join(optimization_reasons)}")
            self._trigger_system_optimization()
    
    def _trigger_system_optimization(self):
        """触发系统优化"""
        def optimization_task():
            try:
                # 调度器效率优化
                if hasattr(self.vehicle_scheduler, 'efficiency_optimizer'):
                    optimization_result = self.vehicle_scheduler.efficiency_optimizer.optimize_system()
                    print(f"📈 调度器优化完成: {optimization_result}")
                
                # 重新平衡负载
                self._rebalance_system_load()
                
                print("✅ 系统优化完成")
                
            except Exception as e:
                print(f"系统优化错误: {e}")
        
        # 在后台线程中执行优化
        optimization_thread = threading.Thread(target=optimization_task, daemon=True)
        optimization_thread.start()
    
    def _rebalance_system_load(self):
        """重新平衡系统负载"""
        # 检查骨干网络负载
        high_load_paths = []
        
        for path_id, path_data in self.backbone_network.bidirectional_paths.items():
            load_factor = path_data.get_load_factor()
            if load_factor > 0.8:
                high_load_paths.append((path_id, load_factor))
        
        if high_load_paths:
            print(f"🔄 检测到 {len(high_load_paths)} 条高负载路径，执行重平衡...")
            
            # 尝试将部分车辆切换到负载较低的路径
            for path_id, load_factor in high_load_paths:
                # 这里可以实现更复杂的负载重平衡逻辑
                print(f"  路径 {path_id} 负载: {load_factor:.2%}")
    
    def assign_priority_mission(self, vehicle_id: str = None, 
                              priority: TaskPriority = TaskPriority.URGENT) -> bool:
        """分配优先级任务"""
        if not self.is_running:
            print("❌ 系统未运行")
            return False
        
        success = self.vehicle_scheduler.assign_mission_intelligently(
            vehicle_id, "default", priority
        )
        
        if success:
            print(f"✅ 已分配 {priority.name} 优先级任务")
        else:
            print(f"❌ {priority.name} 优先级任务分配失败")
        
        return success
    
    def get_system_status(self) -> Dict[str, Any]:
        """获取系统状态"""
        if not self.is_running:
            return {'status': 'stopped', 'runtime': 0}
        
        runtime = time.time() - self.system_start_time
        
        # 获取各组件状态
        scheduler_stats = self.vehicle_scheduler.get_comprehensive_stats()
        traffic_stats = self.traffic_manager.get_statistics()
        backbone_status = self.backbone_network.get_network_status()
        
        # 最新性能指标
        latest_metrics = self.performance_history[-1] if self.performance_history else None
        
        return {
            'status': 'running',
            'runtime': runtime,
            'system_metrics': {
                'overall_efficiency': latest_metrics.overall_efficiency if latest_metrics else 0.0,
                'backbone_utilization': latest_metrics.backbone_utilization if latest_metrics else 0.0,
                'system_throughput': latest_metrics.system_throughput if latest_metrics else 0.0,
                'vehicle_idle_ratio': latest_metrics.vehicle_idle_ratio if latest_metrics else 0.0
            },
            'component_stats': {
                'scheduler': scheduler_stats,
                'traffic': traffic_stats,
                'backbone': backbone_status
            },
            'performance_history_count': len(self.performance_history)
        }
    
    def get_performance_report(self) -> Dict[str, Any]:
        """获取性能报告"""
        if not self.performance_history:
            return {'error': '无性能数据'}
        
        # 计算统计摘要
        efficiencies = [m.overall_efficiency for m in self.performance_history]
        throughputs = [m.system_throughput for m in self.performance_history]
        idle_ratios = [m.vehicle_idle_ratio for m in self.performance_history]
        
        return {
            'summary': {
                'monitoring_period': len(self.performance_history) * self.monitoring_interval / 60,  # 分钟
                'average_efficiency': sum(efficiencies) / len(efficiencies),
                'max_efficiency': max(efficiencies),
                'min_efficiency': min(efficiencies),
                'average_throughput': sum(throughputs) / len(throughputs),
                'max_throughput': max(throughputs),
                'average_idle_ratio': sum(idle_ratios) / len(idle_ratios)
            },
            'recent_metrics': self.performance_history[-10:],  # 最近10个数据点
            'efficiency_trend': 'improving' if len(efficiencies) > 1 and efficiencies[-1] > efficiencies[0] else 'stable'
        }
    
    def stop_system(self):
        """停止系统"""
        if not self.is_running:
            print("⚠️ 系统未运行")
            return
        
        print("🛑 停止增强调度系统...")
        
        with self.system_lock:
            self.is_running = False
        
        # 等待监控线程结束
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=5.0)
        
        # 清理组件
        if self.traffic_manager:
            self.traffic_manager.clear_all()
        
        if self.vehicle_scheduler:
            self.vehicle_scheduler.shutdown()
        
        print("✅ 系统已停止")
    
    def emergency_stop(self):
        """紧急停止"""
        print("🚨 执行紧急停止...")
        
        # 立即停止所有车辆
        for vehicle_id in self.env.vehicles.keys():
            if vehicle_id in self.env.vehicles:
                self.env.vehicles[vehicle_id]['status'] = 'idle'
                self.env.vehicles[vehicle_id]['path'] = None
        
        # 清除所有路径预留
        if self.traffic_manager:
            self.traffic_manager.clear_all()
        
        # 停止系统
        self.stop_system()
        
        print("🛑 紧急停止完成")
    
    def debug_system_info(self):
        """调试系统信息"""
        print("=== 增强调度系统调试信息 ===")
        
        if self.backbone_network:
            self.backbone_network.debug_network_info()
        
        if self.vehicle_scheduler:
            efficiency_report = self.vehicle_scheduler.get_efficiency_report()
            print(f"\n调度器效率报告:")
            print(f"  系统效率: {efficiency_report.get('system_efficiency', 0):.2%}")
            print(f"  优化历史: {len(efficiency_report.get('optimization_history', []))} 次")
        
        if self.traffic_manager:
            traffic_stats = self.traffic_manager.get_statistics()
            print(f"\n交通管理统计:")
            print(f"  活跃车辆: {traffic_stats.get('active_vehicles', 0)}")
            print(f"  已解决冲突: {traffic_stats.get('resolved_conflicts', 0)}")
            print(f"  骨干路径切换: {traffic_stats.get('backbone_switches', 0)}")
        
        print(f"\n系统运行时间: {time.time() - self.system_start_time:.1f} 秒")

# 使用示例和测试函数
def demo_enhanced_system():
    """演示增强系统的使用"""
    print("🎯 开始增强调度系统演示...")
    
    # 创建测试环境
    class TestEnvironment:
        def __init__(self):
            self.width = 200
            self.height = 200
            self.loading_points = [(50, 50, 0), (150, 50, 0)]
            self.unloading_points = [(50, 150, 0), (150, 150, 0)]
            self.parking_areas = [(100, 100, 0)]
            self.vehicles = {
                f'vehicle_{i}': {
                    'position': (20 + i * 30, 20, 0),
                    'max_load': 100,
                    'load': 0,
                    'status': 'idle'
                }
                for i in range(5)
            }
    
    # 创建测试环境
    env = TestEnvironment()
    
    # 创建增强系统
    system = EnhancedMineSchedulingSystem(env)
    
    try:
        # 初始化系统
        if not system.initialize_system(quality_threshold=0.6):
            print("❌ 系统初始化失败")
            return False
        
        # 启动系统
        if not system.start_system():
            print("❌ 系统启动失败")
            return False
        
        # 运行演示
        print("🏃 开始系统运行演示...")
        
        simulation_time = 0
        max_simulation_time = 300  # 5分钟演示
        
        while simulation_time < max_simulation_time and system.is_running:
            # 更新系统
            system.update_system(1.0)  # 1秒时间步长
            
            # 每30秒分配一个高优先级任务
            if simulation_time % 30 == 0 and simulation_time > 0:
                system.assign_priority_mission(priority=TaskPriority.HIGH)
            
            # 每60秒显示状态
            if simulation_time % 60 == 0:
                status = system.get_system_status()
                print(f"\n⏰ {simulation_time}秒 - 系统状态:")
                print(f"  效率: {status['system_metrics']['overall_efficiency']:.2%}")
                print(f"  吞吐量: {status['system_metrics']['system_throughput']:.1f} 任务/小时")
                print(f"  空闲率: {status['system_metrics']['vehicle_idle_ratio']:.2%}")
            
            simulation_time += 1
            time.sleep(0.1)  # 加速演示
        
        # 显示最终报告
        print("\n📊 最终性能报告:")
        report = system.get_performance_report()
        if 'summary' in report:
            summary = report['summary']
            print(f"  平均效率: {summary['average_efficiency']:.2%}")
            print(f"  最高效率: {summary['max_efficiency']:.2%}")
            print(f"  平均吞吐量: {summary['average_throughput']:.1f} 任务/小时")
            print(f"  效率趋势: {report['efficiency_trend']}")
        
        # 显示调试信息
        system.debug_system_info()
        
        return True
        
    except KeyboardInterrupt:
        print("\n🛑 用户中断，执行优雅停止...")
        system.stop_system()
        return True
        
    except Exception as e:
        print(f"\n❌ 演示过程中发生错误: {e}")
        system.emergency_stop()
        return False
        
    finally:
        # 确保系统关闭
        if system.is_running:
            system.stop_system()
        
        print("🎯 增强调度系统演示结束")

if __name__ == "__main__":
    # 运行演示
    demo_enhanced_system()