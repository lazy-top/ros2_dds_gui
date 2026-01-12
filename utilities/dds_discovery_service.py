import threading
import time
from dataclasses import dataclass
from typing import List, Dict
import socket

@dataclass
class DDSServiceInfo:
    hostname: str
    ros_domain_id: str
    dds_implementation: str
    nodes: List[str]
    topics: List[str]

class DDSDiscoveryService:
    def __init__(self, domain_id=0):
        self._discovered_services = []
        self.domain_id = domain_id
        self.is_running = False
        self.discovery_thread = None
        
    def start_discovery(self):
        """启动DDS服务发现"""
        self.is_running = True
        self.discovery_thread = threading.Thread(target=self._discovery_loop)
        self.discovery_thread.daemon = True
        self.discovery_thread.start()
    
    def stop_discovery(self):
        """停止DDS服务发现"""
        self.is_running = False
        if self.discovery_thread:
            self.discovery_thread.join()
    
    def _discovery_loop(self):
        """发现循环，持续监听网络中的DDS服务"""
        while self.is_running:
            try:
                # 使用多播监听DDS发现消息
                discovered_services = self._listen_multicast()
                self._update_services(discovered_services)
            except Exception as e:
                print(f"发现过程中出错: {e}")
            time.sleep(2)  # 2秒检查一次
    
    def _listen_multicast(self) -> List[DDSServiceInfo]:
        """监听多播地址获取DDS服务信息"""
        services = []
        
        try:
            # DDS默认多播地址：224.0.0.1 + 域ID[4](@ref)
            multicast_group = f"239.255.0.{self.domain_id}"
            multicast_port = 7400  # DDS默认端口
            
            # 创建socket监听多播消息
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', multicast_port))
            
            # 加入多播组
            mreq = socket.inet_aton(multicast_group) + socket.inet_aton('0.0.0.0')
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            
            # 设置超时，非阻塞读取
            sock.settimeout(1.0)
            
            try:
                while True:
                    data, addr = sock.recvfrom(1024)
                    service_info = self._parse_dds_packet(data, addr[0])
                    if service_info:
                        services.append(service_info)
            except socket.timeout:
                pass
                
        except Exception as e:
            print(f"多播监听错误: {e}")
        
        return services
    
    def _parse_dds_packet(self, data: bytes, ip_address: str) -> DDSServiceInfo:
        """解析DDS发现数据包"""
        try:
            # 这里需要根据具体的DDS实现协议来解析数据包
            # 以下为示例解析逻辑
            
            # 获取主机名
            hostname = self._resolve_hostname(ip_address)
            
            # 从数据包中提取域ID（实际需要根据DDS协议解析）
            domain_id = str(self.domain_id)
            
            # 检测DDS实现类型（FastDDS、CycloneDDS等）
            dds_impl = self._detect_dds_implementation(data)
            
            # 解析节点和主题信息
            nodes, topics = self._parse_endpoint_info(data)
            
            return DDSServiceInfo(
                hostname=hostname,
                ros_domain_id=domain_id,
                dds_implementation=dds_impl,
                nodes=nodes,
                topics=topics
            )
        except Exception as e:
            print(f"解析DDS数据包错误: {e}")
            return None
    
    def _resolve_hostname(self, ip_address: str) -> str:
        """通过IP地址解析主机名"""
        try:
            hostname = socket.gethostbyaddr(ip_address)[0]
            return hostname
        except:
            return ip_address  # 解析失败返回IP地址
    
    def _detect_dds_implementation(self, data: bytes) -> str:
        """检测DDS实现类型"""
        # 通过数据包特征识别不同的DDS实现
        if len(data) > 4:
            # 这里需要根据实际协议特征进行识别
            if data[0:2] == b'RT':
                return "FastDDS"
            elif data[0:2] == b'CD':
                return "CycloneDDS"
        return "Unknown"
    
    def _parse_endpoint_info(self, data: bytes) -> tuple:
        """解析端点信息（节点和主题）"""
        nodes = []
        topics = []
        
        # 实际实现需要根据DDS协议解析SPDP和SEDP消息[2](@ref)
        # 这里返回示例数据
        return nodes, topics
    
    def _update_services(self, new_services: List[DDSServiceInfo]):
        """更新发现的DDS服务列表"""
        current_services_map = {service.hostname: service for service in self._discovered_services}
        
        for service in new_services:
            current_services_map[service.hostname] = service
        
        self._discovered_services = list(current_services_map.values())
    
    def get_discovered_services(self) -> List[DDSServiceInfo]:
        """获取在线的DDS服务信息"""
        return self._discovered_services.copy()