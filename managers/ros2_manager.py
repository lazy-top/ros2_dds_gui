import os
import subprocess
import threading
from dataclasses import dataclass
from typing import List, Dict, Optional
from PyQt5.QtCore import QTimer, QObject, pyqtSignal


@dataclass
class DDSServiceInfo:
    """DDS服务信息数据类"""
    hostname: str
    ros_domain_id: str
    dds_implementation: str
    nodes: List[str]

class ROS2Manager(QObject):
    """ROS2连接与节点管理器（单例模式）"""
    
    # 信号定义
    nodes_updated = pyqtSignal(list)
    topics_updated = pyqtSignal(list)
    services_updated = pyqtSignal(list)
    dds_services_updated = pyqtSignal(list)
    
    # 支持的DDS实现
    SUPPORTED_DDS_IMPLEMENTATIONS = {
        'rmw_fastrtps_cpp': 'Fast DDS',
        'rmw_cyclonedds_cpp': 'Cyclone DDS', 
        'rmw_connextdds': 'Connext DDS'
    }

    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ROS2Manager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            super().__init__()
            self.node = None
            self._discovered_services = []
            self._discovery_timer = QTimer()
            self._update_timer = QTimer()
            
            self._init_ros2_node()
            self._setup_timers()
            self._initialized = True

    def _init_ros2_node(self):
        """初始化ROS2节点"""
        try:
            self.node = Node('ros2_dds_gui')
        except Exception as e:
            print(f"Failed to initialize ROS2 node: {e}")

    def _setup_timers(self):
        """设置定时器用于定期更新"""
        # DDS服务发现定时器（每5秒）
        self._discovery_timer.timeout.connect(self._refresh_dds_services)
        self._discovery_timer.start(5000)
        
        # ROS2系统状态更新定时器（每2秒）
        self._update_timer.timeout.connect(self._update_system_state)
        self._update_timer.start(2000)

    def get_local_dds_config(self) -> Dict[str, str]:
        """获取当前本地DDS配置"""
        return {
            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
            'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', 'Not set')
        }

    def switch_dds_config(self, domain_id: str, rmw_implementation: str) -> bool:
        """切换DDS配置"""
        try:
            # 验证域ID
            domain_id_int = int(domain_id)
            if not (0 <= domain_id_int <= 232):
                raise ValueError("Domain ID must be between 0 and 232")
            
            # 验证DDS实现是否支持
            if rmw_implementation not in self.SUPPORTED_DDS_IMPLEMENTATIONS:
                raise ValueError(f"Unsupported DDS implementation: {rmw_implementation}")
            
            # 设置环境变量
            os.environ['ROS_DOMAIN_ID'] = domain_id
            os.environ['RMW_IMPLEMENTATION'] = rmw_implementation
            
            print(f"DDS config switched: DOMAIN_ID={domain_id}, RMW={rmw_implementation}")
            return True
            
        except Exception as e:
            print(f"Failed to switch DDS config: {e}")
            return False

    def _refresh_dds_services(self):
        """刷新发现的DDS服务"""
        try:
            discovered = []
            current_domain = os.environ.get('ROS_DOMAIN_ID', '0')
            
            # 获取当前域内的节点
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                nodes = [node.strip() for node in result.stdout.split('\n') if node.strip()]
                
                # 创建本地服务信息
                local_service = DDSServiceInfo(
                    hostname="localhost",
                    ros_domain_id=current_domain,
                    dds_implementation=os.environ.get('RMW_IMPLEMENTATION', 'Unknown'),
                    nodes=nodes
                )
                discovered.append(local_service)
            
            self._discovered_services = discovered
            self.dds_services_updated.emit(discovered)
            
        except Exception as e:
            print(f"Error refreshing DDS services: {e}")

    def _update_system_state(self):
        """更新ROS2系统状态"""
        if self.node is None:
            return
            
        try:
            # 获取节点列表
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                nodes = [node.strip() for node in result.stdout.split('\n') if node.strip()]
                self.nodes_updated.emit(nodes)
            
            # 获取话题列表
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                topics = [topic.strip() for topic in result.stdout.split('\n') if topic.strip()]
                self.topics_updated.emit(topics)
            
            # 获取服务列表
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                services = [service.strip() for service in result.stdout.split('\n') if service.strip()]
                self.services_updated.emit(services)
                
        except Exception as e:
            print(f"Error updating system state: {e}")

    def get_node_info(self, node_name: str) -> Dict:
        """获取节点详细信息"""
        try:
            result = subprocess.run(['ros2', 'node', 'info', node_name], 
                                  capture_output=True, text=True)
            return {'raw_info': result.stdout} if result.returncode == 0 else {}
        except Exception as e:
            print(f"Error getting node info: {e}")
            return {}

    def get_topic_info(self, topic_name: str) -> Dict:
        """获取话题详细信息"""
        try:
            result = subprocess.run(['ros2', 'topic', 'info', topic_name, '--verbose'], 
                                  capture_output=True, text=True)
            return {'raw_info': result.stdout} if result.returncode == 0 else {}
        except Exception as e:
            print(f"Error getting topic info: {e}")
            return {}

    def shutdown(self):
        """清理资源"""
        self._discovery_timer.stop()
        self._update_timer.stop()
        if self.node:
            self.node.destroy_node()