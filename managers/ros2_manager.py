import subprocess
import os
from typing import Dict, Optional


class ROS2Manager:
    """ROS2系统管理器 - 检测ROS2环境状态和版本信息"""
    
    def __init__(self):
        self._cached_version = None
        self._cached_distro = None
        
    def get_ros2_version(self) -> Optional[str]:
        """获取ROS2版本信息
        
        Returns:
            Optional[str]: ROS2版本字符串
        """
        try:
            result = subprocess.run(['ros2', '--version'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self._cached_version = result.stdout.strip()
                return self._cached_version
        except Exception as e:
            print(f"获取ROS2版本失败: {e}")
        return None
    
    def get_ros2_distro(self) -> Optional[str]:
        """获取ROS2发行版名称
        
        Returns:
            Optional[str]: ROS2发行版名称（如 humble, iron, jazzy）
        """
        # 从环境变量获取
        distro = os.environ.get('ROS_DISTRO', None)
        if distro:
            self._cached_distro = distro
            return distro
        return None
    
    def check_ros2_daemon_status(self) -> bool:
        """检查ROS2 daemon是否运行
        
        Returns:
            bool: daemon是否运行
        """
        try:
            result = subprocess.run(['ros2', 'daemon', 'status'],
                                  capture_output=True, text=True, timeout=5)
            # 如果能正常执行，daemon 通常是运行的
            return result.returncode == 0
        except Exception as e:
            print(f"检查ROS2 daemon状态失败: {e}")
            return False
    
    def check_dds_service_status(self) -> Dict:
        """检查DDS服务状态
        
        Returns:
            Dict: DDS服务状态信息
        """
        status = {
            'running': False,
            'node_count': 0,
            'topic_count': 0,
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '未设置'),
            'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'message': ''
        }
        
        try:
            # 检查是否能获取节点列表
            node_result = subprocess.run(['ros2', 'node', 'list'],
                                       capture_output=True, text=True, timeout=5)
            
            if node_result.returncode == 0:
                nodes = [n.strip() for n in node_result.stdout.split('\n') if n.strip()]
                status['node_count'] = len(nodes)
                status['running'] = True
                
                # 检查话题数量
                topic_result = subprocess.run(['ros2', 'topic', 'list'],
                                            capture_output=True, text=True, timeout=5)
                if topic_result.returncode == 0:
                    topics = [t.strip() for t in topic_result.stdout.split('\n') if t.strip()]
                    status['topic_count'] = len(topics)
                
                status['message'] = f"正常运行 - {status['node_count']}节点, {status['topic_count']}话题"
            else:
                status['message'] = "DDS服务未响应"
                
        except subprocess.TimeoutExpired:
            status['message'] = "检测超时"
        except Exception as e:
            status['message'] = f"检测失败: {str(e)}"
        
        return status
    
    def get_full_status(self) -> Dict:
        """获取完整的ROS2状态信息
        
        Returns:
            Dict: 完整状态信息
        """
        return {
            'ros2_version': self.get_ros2_version(),
            'ros2_distro': self.get_ros2_distro(),
            'daemon_running': self.check_ros2_daemon_status(),
            'dds_status': self.check_dds_service_status()
        }
    
    def start_ros2_daemon(self) -> bool:
        """启动ROS2 daemon
        
        Returns:
            bool: 是否启动成功
        """
        try:
            result = subprocess.run(['ros2', 'daemon', 'start'],
                                  capture_output=True, text=True, timeout=10)
            return result.returncode == 0
        except Exception as e:
            print(f"启动ROS2 daemon失败: {e}")
            return False
    
    def stop_ros2_daemon(self) -> bool:
        """停止ROS2 daemon
        
        Returns:
            bool: 是否停止成功
        """
        try:
            result = subprocess.run(['ros2', 'daemon', 'stop'],
                                  capture_output=True, text=True, timeout=10)
            return result.returncode == 0
        except Exception as e:
            print(f"停止ROS2 daemon失败: {e}")
            return False
    
    def get_rmw_implementation(self) -> str:
        """获取当前RMW实现
        
        Returns:
            str: RMW实现名称
        """
        return os.environ.get('RMW_IMPLEMENTATION', '默认 (FastDDS)')
    
    def get_domain_id(self) -> str:
        """获取当前域ID
        
        Returns:
            str: 域ID
        """
        return os.environ.get('ROS_DOMAIN_ID', '0')
