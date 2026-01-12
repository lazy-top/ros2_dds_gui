import subprocess
import json
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, 
                             QTreeWidgetItem, QSplitter, QTextEdit, QHeaderView,
                             QPushButton, QLabel, QLineEdit, QComboBox,QCheckBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor

class NodeExplorer(QWidget):
    """节点浏览器 - 显示和管理ROS2节点"""
    
    def __init__(self, dds_manager):
        super().__init__()
        self.dds_manager = dds_manager
        self.nodes_info = {}
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 工具栏
        toolbar = QHBoxLayout()
        
        self.refresh_btn = QPushButton("刷新节点")
        self.auto_refresh_check = QCheckBox("自动刷新 (2秒)")
        self.auto_refresh_check.setChecked(True)
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("过滤节点名称...")
        self.detail_combo = QComboBox()
        self.detail_combo.addItems(["基本信息", "详细输出", "JSON格式"])
        
        toolbar.addWidget(self.refresh_btn)
        toolbar.addWidget(self.auto_refresh_check)
        toolbar.addWidget(QLabel("过滤:"))
        toolbar.addWidget(self.filter_edit)
        toolbar.addWidget(QLabel("详情级别:"))
        toolbar.addWidget(self.detail_combo)
        toolbar.addStretch()
        
        # 主内容区域
        splitter = QSplitter(Qt.Horizontal)
        
        # 节点树
        self.node_tree = QTreeWidget()
        self.node_tree.setHeaderLabels(["节点", "类型", "状态", "话题数", "服务数"])
        self.node_tree.header().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.node_tree.itemSelectionChanged.connect(self.on_node_selected)
        
        # 详细信息面板
        detail_widget = QWidget()
        detail_layout = QVBoxLayout(detail_widget)
        
        self.detail_text = QTextEdit()
        self.detail_text.setReadOnly(True)
        self.detail_text.setFont(QFont("Monospace", 9))
        
        # 动作按钮
        action_layout = QHBoxLayout()
        self.kill_btn = QPushButton("终止节点")
        self.info_btn = QPushButton("获取详细信息")
        self.graph_btn = QPushButton("显示连接图")
        
        action_layout.addWidget(self.kill_btn)
        action_layout.addWidget(self.info_btn)
        action_layout.addWidget(self.graph_btn)
        action_layout.addStretch()
        
        detail_layout.addLayout(action_layout)
        detail_layout.addWidget(self.detail_text)
        
        splitter.addWidget(self.node_tree)
        splitter.addWidget(detail_widget)
        splitter.setSizes([400, 600])
        
        layout.addLayout(toolbar)
        layout.addWidget(splitter)
        
        self.setLayout(layout)
        
        # 连接信号
        self.refresh_btn.clicked.connect(self.refresh_nodes)
        self.filter_edit.textChanged.connect(self.filter_nodes)
        self.detail_combo.currentTextChanged.connect(self.on_detail_level_changed)
        self.kill_btn.clicked.connect(self.kill_selected_node)
        self.info_btn.clicked.connect(self.get_detailed_info)
        self.graph_btn.clicked.connect(self.show_node_graph)
        
        self.refresh_nodes()
    
    def setup_timers(self):
        """设置定时器"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_nodes)
        self.auto_refresh_check.toggled.connect(self.toggle_auto_refresh)
        self.toggle_auto_refresh(True)
    
    def toggle_auto_refresh(self, enabled):
        """切换自动刷新"""
        if enabled:
            self.refresh_timer.start(2000)  # 2秒刷新
        else:
            self.refresh_timer.stop()
    
    def refresh_nodes(self):
        """刷新节点列表"""
        try:
            # 获取节点列表
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                nodes = [node.strip() for node in result.stdout.split('\n') if node.strip()]
                self.update_node_tree(nodes)
                self.get_detailed_node_info(nodes)
            else:
                self.detail_text.setText("无法获取节点列表，请检查ROS2环境")
                
        except Exception as e:
            self.detail_text.setText(f"刷新节点时发生错误: {e}")
    
    def update_node_tree(self, nodes):
        """更新节点树"""
        self.node_tree.clear()
        
        for node_name in nodes:
            # 从缓存获取节点信息
            info = self.nodes_info.get(node_name, {})
            
            item = QTreeWidgetItem(self.node_tree)
            item.setText(0, node_name)
            item.setText(1, info.get('type', '未知'))
            item.setText(2, info.get('status', '活跃'))
            item.setText(3, str(info.get('topic_count', 0)))
            item.setText(4, str(info.get('service_count', 0)))
            
            # 根据状态设置颜色
            if info.get('status') == '不活跃':
                for i in range(5):
                    item.setForeground(i, QColor(128, 128, 128))
        
        # 应用过滤
        self.apply_filter()
    
    def get_detailed_node_info(self, nodes):
        """获取节点的详细信息"""
        for node_name in nodes:
            try:
                # 获取节点信息
                result = subprocess.run(['ros2', 'node', 'info', node_name], 
                                      capture_output=True, text=True)
                
                if result.returncode == 0:
                    info = self.parse_node_info(result.stdout, node_name)
                    self.nodes_info[node_name] = info
                    
            except Exception as e:
                print(f"获取节点 {node_name} 信息时出错: {e}")
    
    def parse_node_info(self, info_text, node_name):
        """解析节点信息文本"""
        info = {
            'name': node_name,
            'type': '未知',
            'status': '活跃',
            'topics': [],
            'services': [],
            'actions': [],
            'topic_count': 0,
            'service_count': 0
        }
        
        lines = info_text.split('\n')
        current_section = ''
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            if line.startswith('Subscribers:'):
                current_section = 'subscribers'
            elif line.startswith('Publishers:'):
                current_section = 'publishers'
            elif line.startswith('Service Servers:'):
                current_section = 'services'
            elif line.startswith('Action Servers:'):
                current_section = 'actions'
            elif line.startswith('Action Clients:'):
                current_section = 'action_clients'
            elif current_section and line.startswith('/'):
                # 这是一个话题/服务/动作
                if current_section in ['subscribers', 'publishers']:
                    info['topics'].append(line)
                elif current_section == 'services':
                    info['services'].append(line)
                elif current_section in ['actions', 'action_clients']:
                    info['actions'].append(line)
        
        info['topic_count'] = len(info['topics'])
        info['service_count'] = len(info['services'])
        
        return info
    
    def on_node_selected(self):
        """节点选择变化事件"""
        selected_items = self.node_tree.selectedItems()
        if not selected_items:
            return
            
        item = selected_items[0]
        node_name = item.text(0)
        info = self.nodes_info.get(node_name, {})
        
        self.display_node_info(info)
    
    def display_node_info(self, info):
        """显示节点详细信息"""
        detail_level = self.detail_combo.currentText()
        
        if detail_level == "基本信息":
            text = self.format_basic_info(info)
        elif detail_level == "详细输出":
            text = self.format_detailed_info(info)
        else:  # JSON格式
            text = json.dumps(info, indent=2, ensure_ascii=False)
        
        self.detail_text.setText(text)
    
    def format_basic_info(self, info):
        """格式化基本信息"""
        return f"""节点名称: {info.get('name', '未知')}
状态: {info.get('status', '活跃')}
发布话题数: {info.get('topic_count', 0)}
服务数: {info.get('service_count', 0)}
动作数: {len(info.get('actions', []))}"""
    
    def format_detailed_info(self, info):
        """格式化详细信息"""
        text = f"节点: {info.get('name', '未知')}\n"
        text += "=" * 50 + "\n\n"
        
        text += "发布的话题:\n"
        for topic in info.get('topics', [])[:10]:  # 只显示前10个
            text += f"  {topic}\n"
        
        text += "\n服务:\n"
        for service in info.get('services', [])[:10]:
            text += f"  {service}\n"
        
        return text
    
    def filter_nodes(self):
        """过滤节点"""
        self.apply_filter()
    
    def apply_filter(self):
        """应用过滤条件"""
        filter_text = self.filter_edit.text().lower()
        
        for i in range(self.node_tree.topLevelItemCount()):
            item = self.node_tree.topLevelItem(i)
            node_name = item.text(0).lower()
            visible = filter_text in node_name if filter_text else True
            item.setHidden(not visible)
    
    def kill_selected_node(self):
        """终止选中的节点"""
        selected_items = self.node_tree.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "警告", "请先选择一个节点")
            return
            
        item = selected_items[0]
        node_name = item.text(0)
        
        reply = QMessageBox.question(self, "确认终止", 
                                    f"确定要终止节点 {node_name} 吗？")
        
        if reply == QMessageBox.Yes:
            try:
                # 使用ros2 node kill命令终止节点
                subprocess.run(['ros2', 'node', 'kill', node_name])
                self.log_message(f"已发送终止信号给节点: {node_name}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"终止节点失败: {e}")
    
    def get_detailed_info(self):
        """获取详细节点信息"""
        self.refresh_nodes()
    
    def show_node_graph(self):
        """显示节点连接图"""
        # 这里可以集成rqt_graph或其他图形化显示
        QMessageBox.information(self, "功能提示", 
                              "节点连接图功能将在后续版本中实现")
    
    def on_detail_level_changed(self):
        """详情级别改变事件"""
        self.on_node_selected()
    
    def log_message(self, message):
        """记录日志消息"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.detail_text.append(f"[{timestamp}] {message}")