import subprocess
import json
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView,
                             QGraphicsScene, QGraphicsItem, QGroupBox, QPushButton,
                             QLabel, QComboBox, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont
from PyQt5.QtSvg import QSvgRenderer
import networkx as nx
# import matplotlib.pyplot as plt
from io import BytesIO

class GraphView(QWidget):
    """拓扑图查看器 - 可视化ROS2节点和话题的连接关系"""
    
    def __init__(self, dds_manager):
        super().__init__()
        self.dds_manager = dds_manager
        self.graph = nx.DiGraph()
        self.node_positions = {}
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 控制工具栏
        control_layout = QHBoxLayout()
        
        self.refresh_btn = QPushButton("刷新拓扑图")
        self.auto_refresh_check = QCheckBox("自动刷新")
        self.auto_refresh_check.setChecked(True)
        self.layout_combo = QComboBox()
        self.layout_combo.addItems(["自动布局", "分层布局", "环形布局", "力导向布局"])
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["显示全部", "仅显示节点", "仅显示话题", "自定义过滤"])
        
        control_layout.addWidget(self.refresh_btn)
        control_layout.addWidget(self.auto_refresh_check)
        control_layout.addWidget(QLabel("布局算法:"))
        control_layout.addWidget(self.layout_combo)
        control_layout.addWidget(QLabel("显示过滤:"))
        control_layout.addWidget(self.filter_combo)
        control_layout.addStretch()
        
        # 图形视图
        view_group = QGroupBox("ROS2计算图拓扑")
        view_layout = QVBoxLayout()
        
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        # self.view.setRenderHint(QPainter.Antialiasing)
        
        # 状态信息
        self.status_label = QLabel("就绪")
        
        view_layout.addWidget(self.view)
        view_layout.addWidget(self.status_label)
        view_group.setLayout(view_layout)
        
        layout.addLayout(control_layout)
        layout.addWidget(view_group)
        
        self.setLayout(layout)
        
        # 连接信号
        self.refresh_btn.clicked.connect(self.refresh_graph)
        self.layout_combo.currentTextChanged.connect(self.apply_layout)
        self.filter_combo.currentTextChanged.connect(self.apply_filter)
        
        self.refresh_graph()
    
    def setup_timers(self):
        """设置定时器"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_graph)
        self.auto_refresh_check.toggled.connect(self.toggle_auto_refresh)
        self.toggle_auto_refresh(True)
    
    def toggle_auto_refresh(self, enabled):
        """切换自动刷新"""
        if enabled:
            self.refresh_timer.start(10000)  # 10秒刷新
        else:
            self.refresh_timer.stop()
    
    def refresh_graph(self):
        """刷新拓扑图"""
        try:
            self.status_label.setText("正在生成拓扑图...")
            
            # 使用ros2命令获取计算图信息
            dot_data = self.get_ros2_graph_dot()
            if dot_data:
                self.parse_dot_data(dot_data)
                self.visualize_graph()
                self.status_label.setText(f"拓扑图已更新 - 节点: {len(self.get_nodes())}, 连接: {len(self.get_edges())}")
            else:
                self.status_label.setText("无法生成拓扑图")
                
        except Exception as e:
            self.status_label.setText(f"生成拓扑图时出错: {e}")
    
    def get_ros2_graph_dot(self):
        """获取ROS2计算图的DOT格式数据"""
        try:
            # 使用ros2 run rqt_graph rqt_graph --dot命令获取DOT数据
            result = subprocess.run([
                'ros2', 'run', 'rqt_graph', 'rqt_graph',
                '--dot'
            ], capture_output=True, text=True, timeout=10.0)
            
            if result.returncode == 0:
                return result.stdout
            else:
                # 尝试备用方法
                return self.get_ros2_graph_fallback()
                
        except Exception as e:
            print(f"获取DOT数据失败: {e}")
            return None
    
    def get_ros2_graph_fallback(self):
        """备用方法获取计算图信息"""
        try:
            # 手动构建节点-话题关系图
            self.graph.clear()
            
            # 获取所有节点
            node_result = subprocess.run(['ros2', 'node', 'list'], 
                                       capture_output=True, text=True)
            if node_result.returncode != 0:
                return None
            
            nodes = [n.strip() for n in node_result.stdout.split('\n') if n.strip()]
            
            # 获取所有话题
            topic_result = subprocess.run(['ros2', 'topic', 'list'], 
                                        capture_output=True, text=True)
            if topic_result.returncode != 0:
                return None
            
            topics = [t.strip() for t in topic_result.stdout.split('\n') if t.strip()]
            
            # 构建图结构
            for node in nodes:
                self.graph.add_node(node, type='node', label=node)
                
                # 获取节点的发布和订阅信息
                try:
                    info_result = subprocess.run(['ros2', 'node', 'info', node], 
                                               capture_output=True, text=True)
                    if info_result.returncode == 0:
                        self.parse_node_info(node, info_result.stdout)
                except:
                    pass
            
            for topic in topics:
                self.graph.add_node(topic, type='topic', label=topic)
            
            return "手动构建的计算图"
            
        except Exception as e:
            print(f"备用方法失败: {e}")
            return None
    
    def parse_node_info(self, node_name, info_text):
        """解析节点信息并构建图连接"""
        lines = info_text.split('\n')
        current_section = ''
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            if line.startswith('Publishers:'):
                current_section = 'publishers'
            elif line.startswith('Subscribers:'):
                current_section = 'subscribers'
            elif line.startswith('Service Servers:'):
                current_section = 'services'
            elif current_section and line.startswith('/'):
                # 这是一个话题连接
                topic_name = line.split()[0] if ' ' in line else line
                
                if current_section == 'publishers':
                    # 节点发布到话题
                    self.graph.add_edge(node_name, topic_name, 
                                      relationship='publishes')
                elif current_section == 'subscribers':
                    # 节点订阅话题
                    self.graph.add_edge(topic_name, node_name, 
                                      relationship='subscribes')
    
    def parse_dot_data(self, dot_data):
        """解析DOT格式数据"""
        # 这里是简化的DOT解析，实际应用中可能需要更复杂的解析逻辑
        # 可以使用第三方库如pydot来完整解析
        self.graph.clear()
        
        # 简化解析：提取节点和边
        lines = dot_data.split('\n')
        for line in lines:
            line = line.strip()
            if '->' in line and not line.startswith('//'):
                # 这是一个边定义
                parts = line.split('->')
                if len(parts) == 2:
                    source = parts[0].strip()
                    target = parts[1].split('[')[0].strip()
                    
                    # 清理节点名称
                    source = source.replace('"', '')
                    target = target.replace('"', '')
                    
                    self.graph.add_edge(source, target)
    
    def visualize_graph(self):
        """可视化图结构"""
        self.scene.clear()
        
        if len(self.graph.nodes()) == 0:
            self.scene.addText("无节点数据", QFont("Arial", 16))
            return
        
        # 应用布局算法
        layout_name = self.layout_combo.currentText()
        if layout_name == "自动布局":
            pos = nx.spring_layout(self.graph, k=1, iterations=50)
        elif layout_name == "分层布局":
            pos = nx.multipartite_layout(self.graph)
        elif layout_name == "环形布局":
            pos = nx.circular_layout(self.graph)
        else:  # 力导向布局
            pos = nx.spring_layout(self.graph)
        
        # 绘制节点
        node_items = {}
        for node in self.graph.nodes():
            x, y = pos[node]
            item = self.create_node_item(node, x * 500, y * 500)
            node_items[node] = item
            self.scene.addItem(item)
        
        # 绘制边
        for edge in self.graph.edges():
            source, target = edge
            if source in node_items and target in node_items:
                source_item = node_items[source]
                target_item = node_items[target]
                
                line = self.create_edge_item(source_item, target_item)
                self.scene.addItem(line)
    
    def create_node_item(self, node_name, x, y):
        """创建节点图形项"""
        from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsTextItem
        
        # 创建节点椭圆
        radius = 30
        ellipse = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
        ellipse.setPos(x, y)
        
        # 根据节点类型设置颜色
        if 'topic' in node_name.lower() or node_name.startswith('/'):
            # 话题节点 - 蓝色
            ellipse.setBrush(QBrush(QColor(173, 216, 230)))
        else:
            # 普通节点 - 绿色
            ellipse.setBrush(QBrush(QColor(144, 238, 144)))
        
        ellipse.setPen(QPen(Qt.black, 2))
        
        # 添加节点标签
        text = QGraphicsTextItem(node_name)
        text.setPos(x - radius, y + radius + 5)
        text.setFont(QFont("Arial", 8))
        
        # 将椭圆和文本组合
        group = QGraphicsItemGroup()
        group.addToGroup(ellipse)
        group.addToGroup(text)
        
        return group
    
    def create_edge_item(self, source_item, target_item):
        """创建边图形项"""
        from PyQt5.QtCore import QLineF
        from PyQt5.QtWidgets import QGraphicsLineItem
        
        # 获取位置
        source_pos = source_item.scenePos()
        target_pos = target_item.scenePos()
        
        # 创建连线
        line = QGraphicsLineItem(QLineF(source_pos, target_pos))
        line.setPen(QPen(Qt.darkGray, 2, Qt.SolidLine))
        
        return line
    
    def apply_layout(self):
        """应用布局算法"""
        self.visualize_graph()
    
    def apply_filter(self):
        """应用显示过滤"""
        # 这里可以实现基于过滤条件的图形显示
        self.visualize_graph()
    
    def get_nodes(self):
        """获取图中所有节点"""
        return list(self.graph.nodes())
    
    def get_edges(self):
        """获取图中所有边"""
        return list(self.graph.edges())
    
    def export_graph(self, filename):
        """导出图为图片"""
        try:
            # 使用matplotlib导出
            plt.figure(figsize=(12, 8))
            pos = nx.spring_layout(self.graph)
            nx.draw(self.graph, pos, with_labels=True, node_color='lightblue', 
                   node_size=500, font_size=8)
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            plt.close()
            return True
        except Exception as e:
            print(f"导出图失败: {e}")
            return False