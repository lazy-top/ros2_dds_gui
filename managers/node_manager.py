import subprocess
import json
from typing import List, Dict, Optional

class NodeManager:
    """ROS2节点管理器 - 提供节点操作的核心功能"""
    
    def __init__(self):
        self.nodes_cache = {}
        
    def get_nodes(self) -> List[str]:
        """获取所有活跃的ROS2节点列表
        
        Returns:
            List[str]: 节点名称列表
        """
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                nodes = [node.strip() for node in result.stdout.split('\n') if node.strip()]
                return nodes
            return []
        except Exception as e:
            print(f"获取节点列表失败: {e}")
            return []
    
    def get_node_info(self, node_name: str) -> Optional[str]:
        """获取指定节点的详细信息原始输出
        
        Args:
            node_name: 节点名称
            
        Returns:
            Optional[str]: 节点信息的原始文本输出
        """
        try:
            result = subprocess.run(['ros2', 'node', 'info', node_name],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout
            return None
        except Exception as e:
            print(f"获取节点 {node_name} 信息失败: {e}")
            return None
    
    def parse_node_info(self, info_text: str, node_name: str) -> Dict:
        """解析节点信息文本
        
        Args:
            info_text: ros2 node info 命令的输出文本
            node_name: 节点名称
            
        Returns:
            Dict: 包含节点详细信息的字典
        """
        info = {
            'name': node_name,
            'type': '未知',
            'status': '活跃',
            'subscribers': [],
            'publishers': [],
            'services': [],
            'service_servers': [],
            'service_clients': [],
            'actions': [],
            'action_servers': [],
            'action_clients': [],
            'topics': [],
            'topic_count': 0,
            'service_count': 0,
            'action_count': 0
        }
        
        if not info_text:
            return info
        
        lines = info_text.split('\n')
        current_section = ''
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
            
            # 识别不同的信息段落
            if 'Subscribers:' in line:
                current_section = 'subscribers'
            elif 'Publishers:' in line:
                current_section = 'publishers'
            elif 'Service Servers:' in line:
                current_section = 'service_servers'
            elif 'Service Clients:' in line:
                current_section = 'service_clients'
            elif 'Action Servers:' in line:
                current_section = 'action_servers'
            elif 'Action Clients:' in line:
                current_section = 'action_clients'
            elif current_section and (line.startswith('/') or line.startswith('~')):
                # 这是一个话题/服务/动作名称
                topic_name = line.split(':')[0].strip()
                
                if current_section == 'subscribers':
                    info['subscribers'].append(topic_name)
                    if topic_name not in info['topics']:
                        info['topics'].append(topic_name)
                elif current_section == 'publishers':
                    info['publishers'].append(topic_name)
                    if topic_name not in info['topics']:
                        info['topics'].append(topic_name)
                elif current_section == 'service_servers':
                    info['service_servers'].append(topic_name)
                    if topic_name not in info['services']:
                        info['services'].append(topic_name)
                elif current_section == 'service_clients':
                    info['service_clients'].append(topic_name)
                    if topic_name not in info['services']:
                        info['services'].append(topic_name)
                elif current_section == 'action_servers':
                    info['action_servers'].append(topic_name)
                    if topic_name not in info['actions']:
                        info['actions'].append(topic_name)
                elif current_section == 'action_clients':
                    info['action_clients'].append(topic_name)
                    if topic_name not in info['actions']:
                        info['actions'].append(topic_name)
        
        info['topic_count'] = len(info['topics'])
        info['service_count'] = len(info['services'])
        info['action_count'] = len(info['actions'])
        
        return info
    
    def get_parsed_node_info(self, node_name: str) -> Dict:
        """获取并解析节点信息
        
        Args:
            node_name: 节点名称
            
        Returns:
            Dict: 解析后的节点信息字典
        """
        info_text = self.get_node_info(node_name)
        if info_text:
            parsed_info = self.parse_node_info(info_text, node_name)
            self.nodes_cache[node_name] = parsed_info
            return parsed_info
        return self.get_empty_node_info(node_name)
    
    def get_all_nodes_info(self) -> Dict[str, Dict]:
        """获取所有节点的详细信息
        
        Returns:
            Dict[str, Dict]: 节点名称到节点信息的映射
        """
        nodes = self.get_nodes()
        all_info = {}
        
        for node_name in nodes:
            info = self.get_parsed_node_info(node_name)
            all_info[node_name] = info
        
        return all_info
    
    def kill_node(self, node_name: str) -> bool:
        """终止指定的节点
        
        Args:
            node_name: 要终止的节点名称
            
        Returns:
            bool: 是否成功发送终止信号
        """
        try:
            result = subprocess.run(['ros2', 'daemon', 'stop', node_name],
                                  capture_output=True, text=True, timeout=5)
            # ros2 daemon stop 可能不适用于所有节点，尝试使用 lifecycle
            if result.returncode != 0:
                # 尝试使用 pkill
                node_simple_name = node_name.split('/')[-1]
                subprocess.run(['pkill', '-f', node_simple_name], 
                             capture_output=True, timeout=5)
            return True
        except Exception as e:
            print(f"终止节点 {node_name} 失败: {e}")
            return False
    
    def node_exists(self, node_name: str) -> bool:
        """检查节点是否存在
        
        Args:
            node_name: 节点名称
            
        Returns:
            bool: 节点是否存在
        """
        nodes = self.get_nodes()
        return node_name in nodes
    
    def get_node_topics(self, node_name: str) -> List[str]:
        """获取节点的所有话题（包括发布和订阅）
        
        Args:
            node_name: 节点名称
            
        Returns:
            List[str]: 话题列表
        """
        info = self.get_parsed_node_info(node_name)
        return info.get('topics', [])
    
    def get_node_services(self, node_name: str) -> List[str]:
        """获取节点的所有服务
        
        Args:
            node_name: 节点名称
            
        Returns:
            List[str]: 服务列表
        """
        info = self.get_parsed_node_info(node_name)
        return info.get('services', [])
    
    def get_node_actions(self, node_name: str) -> List[str]:
        """获取节点的所有动作
        
        Args:
            node_name: 节点名称
            
        Returns:
            List[str]: 动作列表
        """
        info = self.get_parsed_node_info(node_name)
        return info.get('actions', [])
    
    def get_empty_node_info(self, node_name: str) -> Dict:
        """获取空的节点信息模板
        
        Args:
            node_name: 节点名称
            
        Returns:
            Dict: 空的节点信息字典
        """
        return {
            'name': node_name,
            'type': '未知',
            'status': '不活跃',
            'subscribers': [],
            'publishers': [],
            'services': [],
            'actions': [],
            'topics': [],
            'topic_count': 0,
            'service_count': 0,
            'action_count': 0
        }
    
    def format_node_info_json(self, node_name: str) -> str:
        """将节点信息格式化为JSON字符串
        
        Args:
            node_name: 节点名称
            
        Returns:
            str: JSON格式的节点信息
        """
        info = self.get_parsed_node_info(node_name)
        return json.dumps(info, indent=2, ensure_ascii=False)
    
    def clear_cache(self):
        """清空节点信息缓存"""
        self.nodes_cache.clear()
    
    def refresh_node_cache(self, node_name: str):
        """刷新指定节点的缓存
        
        Args:
            node_name: 节点名称
        """
        if node_name in self.nodes_cache:
            del self.nodes_cache[node_name]
        self.get_parsed_node_info(node_name)
