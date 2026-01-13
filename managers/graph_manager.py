import subprocess
import json
from typing import List, Dict, Tuple, Optional, Set
import networkx as nx


class GraphManager:
    """ROS2计算图管理器 - 管理节点、话题和它们之间的连接关系"""
    
    def __init__(self):
        self.graph = nx.DiGraph()
        self.nodes_cache = {}
        self.topics_cache = {}
        self.relationships = []
        
    def get_ros2_graph(self) -> nx.DiGraph:
        """获取完整的ROS2计算图
        
        Returns:
            nx.DiGraph: NetworkX有向图对象
        """
        self.graph.clear()
        
        try:
            # 获取所有节点和话题
            nodes = self._get_all_nodes()
            topics = self._get_all_topics()
            
            # 添加节点到图中
            for node in nodes:
                self.graph.add_node(node, node_type='ros_node', label=node)
            
            # 添加话题到图中
            for topic in topics:
                self.graph.add_node(topic, node_type='topic', label=topic)
            
            # 构建节点-话题连接关系
            for node in nodes:
                self._build_node_connections(node)
            
            return self.graph
            
        except Exception as e:
            print(f"获取ROS2计算图失败: {e}")
            return self.graph
    
    def _get_all_nodes(self) -> List[str]:
        """获取所有ROS2节点
        
        Returns:
            List[str]: 节点名称列表
        """
        try:
            result = subprocess.run(['ros2', 'node', 'list'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return [n.strip() for n in result.stdout.split('\n') if n.strip()]
        except Exception as e:
            print(f"获取节点列表失败: {e}")
        return []
    
    def _get_all_topics(self) -> List[str]:
        """获取所有话题
        
        Returns:
            List[str]: 话题名称列表
        """
        try:
            result = subprocess.run(['ros2', 'topic', 'list'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return [t.strip() for t in result.stdout.split('\n') if t.strip()]
        except Exception as e:
            print(f"获取话题列表失败: {e}")
        return []
    
    def _build_node_connections(self, node_name: str):
        """构建节点的连接关系
        
        Args:
            node_name: 节点名称
        """
        try:
            result = subprocess.run(['ros2', 'node', 'info', node_name],
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self._parse_node_connections(node_name, result.stdout)
        except Exception as e:
            print(f"获取节点 {node_name} 连接信息失败: {e}")
    
    def _parse_node_connections(self, node_name: str, info_text: str):
        """解析节点连接信息
        
        Args:
            node_name: 节点名称
            info_text: 节点信息文本
        """
        lines = info_text.split('\n')
        current_section = ''
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
            
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
                # 提取话题/服务/动作名称
                topic_name = line.split(':')[0].strip()
                
                if current_section == 'publishers':
                    # 节点发布到话题
                    if self.graph.has_node(topic_name):
                        self.graph.add_edge(node_name, topic_name, 
                                          relationship='publishes',
                                          edge_type='publish')
                elif current_section == 'subscribers':
                    # 话题到节点（订阅）
                    if self.graph.has_node(topic_name):
                        self.graph.add_edge(topic_name, node_name,
                                          relationship='subscribes',
                                          edge_type='subscribe')
                elif current_section == 'service_servers':
                    # 服务服务器
                    if not self.graph.has_node(topic_name):
                        self.graph.add_node(topic_name, node_type='service', label=topic_name)
                    self.graph.add_edge(node_name, topic_name,
                                      relationship='serves',
                                      edge_type='service_server')
                elif current_section == 'service_clients':
                    # 服务客户端
                    if not self.graph.has_node(topic_name):
                        self.graph.add_node(topic_name, node_type='service', label=topic_name)
                    self.graph.add_edge(node_name, topic_name,
                                      relationship='calls',
                                      edge_type='service_client')
    
    def get_node_publishers(self, node_name: str) -> List[str]:
        """获取节点发布的所有话题
        
        Args:
            node_name: 节点名称
            
        Returns:
            List[str]: 发布的话题列表
        """
        if not self.graph.has_node(node_name):
            return []
        
        publishers = []
        for successor in self.graph.successors(node_name):
            edge_data = self.graph.get_edge_data(node_name, successor)
            if edge_data and edge_data.get('relationship') == 'publishes':
                publishers.append(successor)
        
        return publishers
    
    def get_node_subscribers(self, node_name: str) -> List[str]:
        """获取节点订阅的所有话题
        
        Args:
            node_name: 节点名称
            
        Returns:
            List[str]: 订阅的话题列表
        """
        if not self.graph.has_node(node_name):
            return []
        
        subscribers = []
        for predecessor in self.graph.predecessors(node_name):
            edge_data = self.graph.get_edge_data(predecessor, node_name)
            if edge_data and edge_data.get('relationship') == 'subscribes':
                subscribers.append(predecessor)
        
        return subscribers
    
    def get_topic_publishers(self, topic_name: str) -> List[str]:
        """获取话题的所有发布者节点
        
        Args:
            topic_name: 话题名称
            
        Returns:
            List[str]: 发布者节点列表
        """
        if not self.graph.has_node(topic_name):
            return []
        
        publishers = []
        for predecessor in self.graph.predecessors(topic_name):
            edge_data = self.graph.get_edge_data(predecessor, topic_name)
            if edge_data and edge_data.get('relationship') == 'publishes':
                publishers.append(predecessor)
        
        return publishers
    
    def get_topic_subscribers(self, topic_name: str) -> List[str]:
        """获取话题的所有订阅者节点
        
        Args:
            topic_name: 话题名称
            
        Returns:
            List[str]: 订阅者节点列表
        """
        if not self.graph.has_node(topic_name):
            return []
        
        subscribers = []
        for successor in self.graph.successors(topic_name):
            edge_data = self.graph.get_edge_data(topic_name, successor)
            if edge_data and edge_data.get('relationship') == 'subscribes':
                subscribers.append(successor)
        
        return subscribers
    
    def get_node_info(self, node_name: str) -> Dict:
        """获取节点的详细信息
        
        Args:
            node_name: 节点名称
            
        Returns:
            Dict: 节点详细信息
        """
        info = {
            'name': node_name,
            'type': 'node',
            'publishers': self.get_node_publishers(node_name),
            'subscribers': self.get_node_subscribers(node_name),
            'connections': self.graph.degree(node_name) if self.graph.has_node(node_name) else 0
        }
        return info
    
    def get_topic_info(self, topic_name: str) -> Dict:
        """获取话题的详细信息
        
        Args:
            topic_name: 话题名称
            
        Returns:
            Dict: 话题详细信息
        """
        info = {
            'name': topic_name,
            'type': 'topic',
            'publishers': self.get_topic_publishers(topic_name),
            'subscribers': self.get_topic_subscribers(topic_name),
            'publisher_count': len(self.get_topic_publishers(topic_name)),
            'subscriber_count': len(self.get_topic_subscribers(topic_name))
        }
        return info
    
    def get_all_nodes_info(self) -> List[Dict]:
        """获取所有节点的信息列表
        
        Returns:
            List[Dict]: 节点信息列表
        """
        nodes_info = []
        for node in self.graph.nodes():
            node_data = self.graph.nodes[node]
            if node_data.get('node_type') == 'ros_node':
                nodes_info.append(self.get_node_info(node))
        return nodes_info
    
    def get_all_topics_info(self) -> List[Dict]:
        """获取所有话题的信息列表
        
        Returns:
            List[Dict]: 话题信息列表
        """
        topics_info = []
        for node in self.graph.nodes():
            node_data = self.graph.nodes[node]
            if node_data.get('node_type') == 'topic':
                topics_info.append(self.get_topic_info(node))
        return topics_info
    
    def get_graph_statistics(self) -> Dict:
        """获取图的统计信息
        
        Returns:
            Dict: 统计信息
        """
        nodes = [n for n in self.graph.nodes() 
                if self.graph.nodes[n].get('node_type') == 'ros_node']
        topics = [n for n in self.graph.nodes() 
                 if self.graph.nodes[n].get('node_type') == 'topic']
        
        return {
            'total_nodes': len(nodes),
            'total_topics': len(topics),
            'total_edges': self.graph.number_of_edges(),
            'total_items': self.graph.number_of_nodes(),
            'is_connected': nx.is_weakly_connected(self.graph) if len(self.graph.nodes()) > 0 else False
        }
    
    def filter_graph(self, filter_type: str) -> nx.DiGraph:
        """根据过滤条件返回过滤后的图
        
        Args:
            filter_type: 过滤类型 ('all', 'nodes_only', 'topics_only', 'custom')
            
        Returns:
            nx.DiGraph: 过滤后的图
        """
        if filter_type == 'all' or filter_type == '显示全部':
            return self.graph
        
        filtered_graph = nx.DiGraph()
        
        if filter_type == 'nodes_only' or filter_type == '仅显示节点':
            # 只显示节点
            for node in self.graph.nodes():
                node_data = self.graph.nodes[node]
                if node_data.get('node_type') == 'ros_node':
                    filtered_graph.add_node(node, **node_data)
        
        elif filter_type == 'topics_only' or filter_type == '仅显示话题':
            # 只显示话题
            for node in self.graph.nodes():
                node_data = self.graph.nodes[node]
                if node_data.get('node_type') == 'topic':
                    filtered_graph.add_node(node, **node_data)
        
        return filtered_graph
    
    def get_layout_positions(self, layout_type: str = 'spring') -> Dict:
        """获取图的布局位置
        
        Args:
            layout_type: 布局类型 ('spring', 'circular', 'hierarchical', 'kamada_kawai')
            
        Returns:
            Dict: 节点位置字典 {node_name: (x, y)}
        """
        if len(self.graph.nodes()) == 0:
            return {}
        
        try:
            if layout_type == 'spring' or layout_type == '自动布局' or layout_type == '力导向布局':
                return nx.spring_layout(self.graph, k=1, iterations=50)
            elif layout_type == 'circular' or layout_type == '环形布局':
                return nx.circular_layout(self.graph)
            elif layout_type == 'hierarchical' or layout_type == '分层布局':
                # 使用层次布局
                try:
                    return nx.multipartite_layout(self.graph)
                except:
                    # 如果失败，使用shell布局作为备选
                    return nx.shell_layout(self.graph)
            elif layout_type == 'kamada_kawai':
                return nx.kamada_kawai_layout(self.graph)
            else:
                return nx.spring_layout(self.graph)
        except Exception as e:
            print(f"生成布局失败: {e}")
            # 返回简单的线性布局
            return {node: (i, 0) for i, node in enumerate(self.graph.nodes())}
    
    def get_shortest_path(self, source: str, target: str) -> Optional[List[str]]:
        """获取两个节点之间的最短路径
        
        Args:
            source: 源节点
            target: 目标节点
            
        Returns:
            Optional[List[str]]: 路径节点列表，如果不存在则返回None
        """
        try:
            if self.graph.has_node(source) and self.graph.has_node(target):
                return nx.shortest_path(self.graph, source, target)
        except nx.NetworkXNoPath:
            return None
        except Exception as e:
            print(f"计算最短路径失败: {e}")
        return None
    
    def get_connected_components(self) -> List[Set[str]]:
        """获取图的连通分量
        
        Returns:
            List[Set[str]]: 连通分量列表
        """
        try:
            return list(nx.weakly_connected_components(self.graph))
        except Exception as e:
            print(f"获取连通分量失败: {e}")
            return []
    
    def export_to_dot(self) -> str:
        """导出图为DOT格式
        
        Returns:
            str: DOT格式的图描述
        """
        try:
            from networkx.drawing.nx_pydot import to_pydot
            pydot_graph = to_pydot(self.graph)
            return pydot_graph.to_string()
        except ImportError:
            # 如果没有pydot，手动生成简单的DOT格式
            dot_str = "digraph ROS2_Graph {\n"
            
            # 添加节点
            for node in self.graph.nodes():
                node_data = self.graph.nodes[node]
                node_type = node_data.get('node_type', 'unknown')
                dot_str += f'  "{node}" [type="{node_type}"];\n'
            
            # 添加边
            for source, target in self.graph.edges():
                edge_data = self.graph.get_edge_data(source, target)
                relationship = edge_data.get('relationship', 'connected')
                dot_str += f'  "{source}" -> "{target}" [label="{relationship}"];\n'
            
            dot_str += "}\n"
            return dot_str
        except Exception as e:
            print(f"导出DOT格式失败: {e}")
            return ""
    
    def export_to_json(self) -> str:
        """导出图为JSON格式
        
        Returns:
            str: JSON格式的图描述
        """
        try:
            from networkx.readwrite import json_graph
            graph_data = json_graph.node_link_data(self.graph)
            return json.dumps(graph_data, indent=2)
        except Exception as e:
            print(f"导出JSON格式失败: {e}")
            return "{}"
    
    def clear_cache(self):
        """清空缓存"""
        self.nodes_cache.clear()
        self.topics_cache.clear()
        self.relationships.clear()
    
    def refresh(self):
        """刷新图数据"""
        self.clear_cache()
        return self.get_ros2_graph()
