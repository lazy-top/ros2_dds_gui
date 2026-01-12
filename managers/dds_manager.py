from dataclasses import dataclass
import os
from typing import List
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
import time
from utilities.dds_discovery_service import DDSDiscoveryService

@dataclass
class DDSServiceInfo:
    hostname: str
    ros_domain_id: str
    dds_implementation: str
    nodes: List[str]
    topics: List[str]


class DDSManager:
    # 信号定义
    nodes_updated = pyqtSignal(list)
    # 支持的DDS实现
    SUPPORTED_DDS_IMPLEMENTATIONS = {
        'rmw_fastrtps_cpp': 'Fast DDS',
        'rmw_cyclonedds_cpp': 'Cyclone DDS', 
        'rmw_connextdds': 'Connext DDS'
    }
    def __init__(self):
        self._discovered_services = []

    def get_local_dds_config(self) -> DDSServiceInfo:
        return DDSServiceInfo(
            hostname="localhost",
            ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
            dds_implementation=os.environ.get('RMW_IMPLEMENTATION', 'Not set'),
            nodes=["node1", "node2"],
            topics=["topic1", "topic2"]
        )
    def get_discovered_services(self) -> List[DDSServiceInfo]:
        """获取在线的DDS服务信息"""
        # 创建发现服务
        discovery = DDSDiscoveryService(domain_id=0)

        try:
            # 启动发现
            discovery.start_discovery()
            
            # 等待发现过程
            time.sleep(10)
            
            # 获取发现的DDS服务
            _discovered_services = discovery.get_discovered_services()
        finally:
            # 清理资源
            discovery.stop_discovery()
        return self._discovered_services
