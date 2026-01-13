from dataclasses import dataclass
import os
import subprocess
from typing import List, Dict, Optional
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
import time

@dataclass
class DDSServiceInfo:
    hostname: str
    ros_domain_id: str
    dds_implementation: str
    nodes: List[str]
    topics: List[str]


class DDSManager(QObject):
    """DDS管理器 - 管理DDS配置、发现和切换"""
    
    # 信号定义
    nodes_updated = pyqtSignal(list)
    services_updated = pyqtSignal(list)
    config_changed = pyqtSignal(dict)
    
    # 支持的DDS实现
    SUPPORTED_DDS_IMPLEMENTATIONS = {
        'rmw_fastrtps_cpp': 'Fast DDS (默认)',
        'rmw_cyclonedds_cpp': 'Cyclone DDS', 
        'rmw_connextdds': 'Connext DDS (需许可)'
    }
    
    def __init__(self):
        super().__init__()
        self._discovered_services = []
        self._discovery_timer = QTimer()
        self._current_config = self.get_current_config()
        self._setup_timers()

    def _setup_timers(self):
        """设置定时器用于定期更新"""
        self._discovery_timer.timeout.connect(self._refresh_dds_services)
        self._discovery_timer.start(10000)  # 每10秒发现一次
    
    def _refresh_dds_services(self):
        """刷新DDS服务信息"""
        try:
            services = self.discover_remote_services()
            if services:
                self._discovered_services = services
                self.services_updated.emit(services)
        except Exception as e:
            print(f"刷新DDS服务失败: {e}")
    def get_current_config(self) -> Dict:
        """获取当前DDS配置
        
        Returns:
            Dict: 当前DDS配置信息
        """
        return {
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '未设置'),
            'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'status': '活跃',
            'timestamp': time.time()
        }
    
    def get_local_dds_config(self) -> DDSServiceInfo:
        """获取本地DDS配置信息
        
        Returns:
            DDSServiceInfo: 本地DDS服务信息
        """
        try:
            # 获取本地节点列表
            nodes = self._get_local_nodes()
            # 获取本地话题列表
            topics = self._get_local_topics()
            
            return DDSServiceInfo(
                hostname="localhost",
                ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                dds_implementation=os.environ.get('RMW_IMPLEMENTATION', '未设置'),
                nodes=nodes,
                topics=topics
            )
        except Exception as e:
            print(f"获取本地DDS配置失败: {e}")
            return DDSServiceInfo(
                hostname="localhost",
                ros_domain_id="0",
                dds_implementation="未知",
                nodes=[],
                topics=[]
            )
    
    def _get_local_nodes(self) -> List[str]:
        """获取本地节点列表"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'],
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                return [node.strip() for node in result.stdout.split('\n') if node.strip()]
        except Exception as e:
            print(f"获取本地节点失败: {e}")
        return []
    
    def _get_local_topics(self) -> List[str]:
        """获取本地话题列表"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                return [topic.strip() for topic in result.stdout.split('\n') if topic.strip()]
        except Exception as e:
            print(f"获取本地话题失败: {e}")
        return []
    def get_discovered_services(self) -> List[DDSServiceInfo]:
        """获取已发现的DDS服务信息
        
        Returns:
            List[DDSServiceInfo]: 已发现的DDS服务列表
        """
        return self._discovered_services
    
    def discover_remote_services(self) -> List[DDSServiceInfo]:
        """发现远程DDS服务
        
        Returns:
            List[DDSServiceInfo]: 发现的远程服务列表
        """
        # 这里实现DDS服务发现逻辑
        # 可以通过扫描不同域ID或使用DDS内置发现机制
        discovered = []
        
        try:
            # 扫描常用的域ID (0-10)
            for domain_id in range(0, 11):
                if domain_id == int(os.environ.get('ROS_DOMAIN_ID', '0')):
                    continue  # 跳过当前域
                
                service_info = self._scan_domain(domain_id)
                if service_info and (service_info.nodes or service_info.topics):
                    discovered.append(service_info)
        except Exception as e:
            print(f"发现远程服务失败: {e}")
        
        return discovered
    
    def _scan_domain(self, domain_id: int) -> Optional[DDSServiceInfo]:
        """扫描指定域ID的服务
        
        Args:
            domain_id: 要扫描的域ID
            
        Returns:
            Optional[DDSServiceInfo]: 发现的服务信息
        """
        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = str(domain_id)
            
            # 快速检查是否有节点
            result = subprocess.run(['ros2', 'node', 'list'],
                                  env=env, capture_output=True, 
                                  text=True, timeout=2)
            
            if result.returncode == 0:
                nodes = [n.strip() for n in result.stdout.split('\n') if n.strip()]
                if nodes:
                    # 获取话题
                    topic_result = subprocess.run(['ros2', 'topic', 'list'],
                                                 env=env, capture_output=True,
                                                 text=True, timeout=2)
                    topics = []
                    if topic_result.returncode == 0:
                        topics = [t.strip() for t in topic_result.stdout.split('\n') if t.strip()]
                    
                    return DDSServiceInfo(
                        hostname=f"domain_{domain_id}",
                        ros_domain_id=str(domain_id),
                        dds_implementation=env.get('RMW_IMPLEMENTATION', '未知'),
                        nodes=nodes,
                        topics=topics
                    )
        except (subprocess.TimeoutExpired, Exception) as e:
            pass  # 该域没有服务或不可达
        
        return None

    def switch_dds_config(self, domain_id: str, rmw_implementation: str) -> bool:
        """切换DDS配置
        
        Args:
            domain_id: 域ID (0-232)
            rmw_implementation: DDS实现标识
            
        Returns:
            bool: 是否切换成功
        """
        try:
            # 验证域ID
            domain_id_int = int(domain_id)
            if not (0 <= domain_id_int <= 232):
                raise ValueError("域ID必须在0-232之间")
            
            # 验证DDS实现是否支持
            if rmw_implementation not in self.SUPPORTED_DDS_IMPLEMENTATIONS:
                raise ValueError(f"不支持的DDS实现: {rmw_implementation}")
            
            # 检查DDS实现是否可用
            if not self.check_dds_availability(rmw_implementation):
                raise ValueError(f"DDS实现 {rmw_implementation} 不可用")
            
            # 应用配置
            os.environ['ROS_DOMAIN_ID'] = domain_id
            os.environ['RMW_IMPLEMENTATION'] = rmw_implementation
            
            # 更新当前配置
            self._current_config = self.get_current_config()
            
            # 发射配置改变信号
            self.config_changed.emit(self._current_config)
            
            return True
        except Exception as e:
            print(f"切换DDS配置失败: {e}")
            return False
    
    def check_dds_availability(self, rmw_implementation: str) -> bool:
        """检查DDS实现是否可用
        
        Args:
            rmw_implementation: DDS实现标识
            
        Returns:
            bool: DDS实现是否可用
        """
        try:
            env = os.environ.copy()
            env['RMW_IMPLEMENTATION'] = rmw_implementation
            env['ROS_DOMAIN_ID'] = '0'
            
            result = subprocess.run(['ros2', 'node', 'list'],
                                  env=env, capture_output=True,
                                  text=True, timeout=3)
            
            # 检查是否有明确的错误信息
            if "RMW implementation not found" in result.stderr:
                return False
            
            # 即使返回码非0，只要不是找不到实现，就认为可用
            return True
        except subprocess.TimeoutExpired:
            # 超时不一定表示不可用
            return True
        except Exception as e:
            print(f"检查DDS可用性异常: {e}")
            return False
    
    def test_dds_connection(self) -> Dict:
        """测试DDS连接
        
        Returns:
            Dict: 测试结果信息
        """
        result = {
            'success': False,
            'node_count': 0,
            'topic_count': 0,
            'messages': []
        }
        
        try:
            # 测试节点发现
            node_result = subprocess.run(['ros2', 'node', 'list'],
                                       capture_output=True, text=True, timeout=5)
            
            if node_result.returncode == 0:
                nodes = [n.strip() for n in node_result.stdout.split('\n') if n.strip()]
                result['node_count'] = len(nodes)
                result['messages'].append(f"节点发现: {len(nodes)} 个节点")
                
                # 测试话题发现
                topic_result = subprocess.run(['ros2', 'topic', 'list'],
                                            capture_output=True, text=True, timeout=5)
                
                if topic_result.returncode == 0:
                    topics = [t.strip() for t in topic_result.stdout.split('\n') if t.strip()]
                    result['topic_count'] = len(topics)
                    result['messages'].append(f"话题发现: {len(topics)} 个话题")
                    result['success'] = True
                else:
                    result['messages'].append("话题发现失败")
            else:
                result['messages'].append("节点发现失败")
        except Exception as e:
            result['messages'].append(f"连接测试异常: {e}")
        
        return result
    
    def get_supported_implementations(self) -> Dict[str, str]:
        """获取支持的DDS实现列表
        
        Returns:
            Dict[str, str]: DDS实现标识到名称的映射
        """
        return self.SUPPORTED_DDS_IMPLEMENTATIONS.copy()
    
    def set_domain_id(self, domain_id: int) -> bool:
        """设置域ID
        
        Args:
            domain_id: 域ID (0-232)
            
        Returns:
            bool: 是否设置成功
        """
        try:
            if not (0 <= domain_id <= 232):
                raise ValueError("域ID必须在0-232之间")
            
            os.environ['ROS_DOMAIN_ID'] = str(domain_id)
            self._current_config = self.get_current_config()
            self.config_changed.emit(self._current_config)
            return True
        except Exception as e:
            print(f"设置域ID失败: {e}")
            return False
    
    def set_rmw_implementation(self, rmw_implementation: str) -> bool:
        """设置DDS实现
        
        Args:
            rmw_implementation: DDS实现标识
            
        Returns:
            bool: 是否设置成功
        """
        try:
            if rmw_implementation not in self.SUPPORTED_DDS_IMPLEMENTATIONS:
                raise ValueError(f"不支持的DDS实现: {rmw_implementation}")
            
            os.environ['RMW_IMPLEMENTATION'] = rmw_implementation
            self._current_config = self.get_current_config()
            self.config_changed.emit(self._current_config)
            return True
        except Exception as e:
            print(f"设置DDS实现失败: {e}")
            return False
    
    def refresh_services(self):
        """手动刷新DDS服务发现"""
        self._refresh_dds_services()
    
    def start_discovery(self):
        """启动服务发现"""
        if not self._discovery_timer.isActive():
            self._discovery_timer.start(10000)
    
    def stop_discovery(self):
        """停止服务发现"""
        if self._discovery_timer.isActive():
            self._discovery_timer.stop()

