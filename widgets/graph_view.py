import json
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView,
                             QGraphicsScene, QGraphicsItem, QGroupBox, QPushButton,
                             QLabel, QComboBox, QCheckBox, QGraphicsEllipseItem, 
                             QGraphicsTextItem, QGraphicsItemGroup, QGraphicsLineItem)
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QLineF
from PyQt5.QtGui import QPen, QBrush, QColor, QFont
import networkx as nx

class GraphView(QWidget):
    """拓扑图查看器 - 可视化ROS2节点和话题的连接关系"""
    
    def __init__(self, graph_manager, dds_manager):
        super().__init__()
        self.dds_manager = dds_manager
        self.graph_manager = graph_manager
        self.graph = None
        self.node_positions = {}
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 控制工具栏
        control_layout = QHBoxLayout()
        
        self.layout_combo = QComboBox()
        self.layout_combo.addItems(["自动布局", "分层布局", "环形布局", "力导向布局"])
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["显示全部", "仅显示节点", "仅显示话题", "自定义过滤"])
        self.auto_refresh_check = QCheckBox("自动刷新")
        self.auto_refresh_check.setChecked(False)
        self.refresh_btn = QPushButton("刷新拓扑图")
        
        control_layout.addWidget(QLabel("布局算法:"))
        control_layout.addWidget(self.layout_combo)
        control_layout.addWidget(QLabel("显示过滤:"))
        control_layout.addWidget(self.filter_combo)
        control_layout.addStretch()
        control_layout.addWidget(self.auto_refresh_check)
        control_layout.addWidget(self.refresh_btn)
        
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
        self.toggle_auto_refresh(False)
    
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
            
            # 使用 graph_manager 获取计算图
            self.graph = self.graph_manager.get_ros2_graph()
            
            if self.graph and len(self.graph.nodes()) > 0:
                self.visualize_graph()
                stats = self.graph_manager.get_graph_statistics()
                self.status_label.setText(
                    f"拓扑图已更新 - 节点: {stats['total_nodes']}, "
                    f"话题: {stats['total_topics']}, "
                    f"连接: {stats['total_edges']}"
                )
            else:
                self.status_label.setText("无法生成拓扑图或没有活跃节点")
                
        except Exception as e:
            self.status_label.setText(f"生成拓扑图时出错: {e}")
    

    
    def visualize_graph(self):
        """可视化图结构"""
        self.scene.clear()
        
        if not self.graph or len(self.graph.nodes()) == 0:
            self.scene.addText("无节点数据", QFont("Arial", 16))
            return
        
        # 应用过滤
        filter_type = self.filter_combo.currentText()
        display_graph = self.graph_manager.filter_graph(filter_type)
        
        if len(display_graph.nodes()) == 0:
            self.scene.addText("过滤后无节点", QFont("Arial", 16))
            return
        
        # 使用 graph_manager 获取布局位置
        layout_name = self.layout_combo.currentText()
        pos = self.graph_manager.get_layout_positions(layout_name)
        
        # 绘制节点
        node_items = {}
        for node in display_graph.nodes():
            if node in pos:
                x, y = pos[node]
                node_data = display_graph.nodes[node]
                item = self.create_node_item(node, node_data, x * 500, y * 500)
                node_items[node] = item
                self.scene.addItem(item)
        
        # 绘制边
        for edge in display_graph.edges():
            source, target = edge
            if source in node_items and target in node_items:
                source_item = node_items[source]
                target_item = node_items[target]
                edge_data = display_graph.get_edge_data(source, target)
                
                line = self.create_edge_item(source_item, target_item, edge_data)
                self.scene.addItem(line)
    
    def create_node_item(self, node_name, node_data, x, y):
        """创建节点图形项"""
        # 创建节点椭圆
        radius = 30
        ellipse = QGraphicsEllipseItem(-radius, -radius, radius*2, radius*2)
        ellipse.setPos(x, y)
        
        # 根据节点类型设置颜色
        node_type = node_data.get('node_type', 'unknown')
        if node_type == 'topic':
            # 话题节点 - 蓝色
            ellipse.setBrush(QBrush(QColor(173, 216, 230)))
        elif node_type == 'service':
            # 服务节点 - 黄色
            ellipse.setBrush(QBrush(QColor(255, 255, 153)))
        elif node_type == 'ros_node':
            # ROS节点 - 绿色
            ellipse.setBrush(QBrush(QColor(144, 238, 144)))
        else:
            # 未知类型 - 灰色
            ellipse.setBrush(QBrush(QColor(200, 200, 200)))
        
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
    
    def create_edge_item(self, source_item, target_item, edge_data=None):
        """创建边图形项"""
        # 获取位置
        source_pos = source_item.scenePos()
        target_pos = target_item.scenePos()
        
        # 根据边的类型设置样式
        pen = QPen(Qt.darkGray, 2, Qt.SolidLine)
        
        if edge_data:
            edge_type = edge_data.get('edge_type', '')
            if edge_type == 'publish':
                pen.setColor(QColor(0, 150, 0))  # 绿色表示发布
            elif edge_type == 'subscribe':
                pen.setColor(QColor(0, 0, 150))  # 蓝色表示订阅
            elif edge_type in ['service_server', 'service_client']:
                pen.setStyle(Qt.DashLine)  # 虚线表示服务
                pen.setColor(QColor(150, 0, 150))  # 紫色
        
        # 创建连线
        line = QGraphicsLineItem(QLineF(source_pos, target_pos))
        line.setPen(pen)
        
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
        if self.graph:
            return list(self.graph.nodes())
        return []
    
    def get_edges(self):
        """获取图中所有边"""
        if self.graph:
            return list(self.graph.edges())
        return []
    
    def export_graph(self, filename):
        """导出图为图片或文件"""
        try:
            # 根据文件扩展名选择导出格式
            if filename.endswith('.dot'):
                # 导出为DOT格式
                dot_str = self.graph_manager.export_to_dot()
                with open(filename, 'w') as f:
                    f.write(dot_str)
                return True
            elif filename.endswith('.json'):
                # 导出为JSON格式
                json_str = self.graph_manager.export_to_json()
                with open(filename, 'w') as f:
                    f.write(json_str)
                return True
            else:
                # 尝试使用matplotlib导出为图片
                try:
                    import matplotlib.pyplot as plt
                    plt.figure(figsize=(12, 8))
                    pos = self.graph_manager.get_layout_positions('spring')
                    nx.draw(self.graph, pos, with_labels=True, node_color='lightblue', 
                           node_size=500, font_size=8)
                    plt.savefig(filename, dpi=300, bbox_inches='tight')
                    plt.close()
                    return True
                except ImportError:
                    print("需要安装matplotlib才能导出图片")
                    return False
        except Exception as e:
            print(f"导出图失败: {e}")
            return False