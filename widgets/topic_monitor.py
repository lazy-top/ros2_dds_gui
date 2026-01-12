import yaml
import json
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QListWidget, 
                             QListWidgetItem, QTextEdit, QSplitter, QGroupBox,
                             QPushButton, QLineEdit, QComboBox, QCheckBox, QLabel,QMessageBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QTextCursor
# import rclpy
# from rclpy.node import Node
from std_msgs.msg import String
import threading

class TopicMonitor(QWidget):
    """话题监控器 - 实时监控ROS2话题消息"""
    
    message_received = pyqtSignal(dict)  # 消息接收信号
    
    def __init__(self, dds2_manager):
        super().__init__()
        self.dds2_manager = dds2_manager
        self.subscriptions = {}  # 活跃的订阅
        self.message_history = {}  # 消息历史
        self.max_history = 100  # 最大历史消息数
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 控制工具栏
        control_layout = QHBoxLayout()
        
        self.topic_combo = QComboBox()
        self.topic_combo.setEditable(True)
        self.topic_combo.setPlaceholderText("选择或输入话题名称...")
        
        self.refresh_topics_btn = QPushButton("刷新话题列表")
        self.subscribe_btn = QPushButton("订阅话题")
        self.unsubscribe_btn = QPushButton("取消订阅")
        self.clear_btn = QPushButton("清空消息")
        
        self.auto_refresh_check = QCheckBox("自动刷新话题列表")
        self.auto_refresh_check.setChecked(True)
        
        control_layout.addWidget(QLabel("话题:"))
        control_layout.addWidget(self.topic_combo, 1)
        control_layout.addWidget(self.refresh_topics_btn)
        control_layout.addWidget(self.subscribe_btn)
        control_layout.addWidget(self.unsubscribe_btn)
        control_layout.addWidget(self.clear_btn)
        control_layout.addWidget(self.auto_refresh_check)
        
        # 主内容区域
        splitter = QSplitter(Qt.Vertical)
        
        # 话题列表和基本信息
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        
        # 活跃话题列表
        topic_list_group = QGroupBox("活跃话题")
        topic_list_layout = QVBoxLayout()
        
        self.topic_list = QListWidget()
        self.topic_list.itemSelectionChanged.connect(self.on_topic_selected)
        
        topic_list_layout.addWidget(self.topic_list)
        topic_list_group.setLayout(topic_list_layout)
        
        # 话题详细信息
        topic_info_group = QGroupBox("话题信息")
        topic_info_layout = QVBoxLayout()
        
        self.topic_info_text = QTextEdit()
        self.topic_info_text.setReadOnly(True)
        self.topic_info_text.setMaximumHeight(150)
        
        topic_info_layout.addWidget(self.topic_info_text)
        topic_info_group.setLayout(topic_info_layout)
        
        top_layout.addWidget(topic_list_group, 1)
        top_layout.addWidget(topic_info_group, 1)
        
        # 消息显示区域
        bottom_widget = QWidget()
        bottom_layout = QVBoxLayout(bottom_widget)
        
        # 消息显示控制
        message_control_layout = QHBoxLayout()
        
        self.pause_display_check = QCheckBox("暂停显示")
        self.auto_scroll_check = QCheckBox("自动滚动")
        self.auto_scroll_check.setChecked(True)
        self.format_combo = QComboBox()
        self.format_combo.addItems(["原始格式", "JSON格式", "美化格式"])
        
        message_control_layout.addWidget(self.pause_display_check)
        message_control_layout.addWidget(self.auto_scroll_check)
        message_control_layout.addWidget(QLabel("显示格式:"))
        message_control_layout.addWidget(self.format_combo)
        message_control_layout.addStretch()
        
        self.message_text = QTextEdit()
        self.message_text.setReadOnly(True)
        self.message_text.setFont(QFont("Monospace", 9))
        
        bottom_layout.addLayout(message_control_layout)
        bottom_layout.addWidget(self.message_text)
        
        splitter.addWidget(top_widget)
        splitter.addWidget(bottom_widget)
        splitter.setSizes([300, 400])
        
        layout.addLayout(control_layout)
        layout.addWidget(splitter)
        
        self.setLayout(layout)
        
        # 连接信号
        self.refresh_topics_btn.clicked.connect(self.refresh_topics)
        self.subscribe_btn.clicked.connect(self.subscribe_to_topic)
        self.unsubscribe_btn.clicked.connect(self.unsubscribe_from_topic)
        self.clear_btn.clicked.connect(self.clear_messages)
        self.format_combo.currentTextChanged.connect(self.on_format_changed)
        
        # 连接消息接收信号
        self.message_received.connect(self.on_message_received)
        
        self.refresh_topics()
    
    def setup_timers(self):
        """设置定时器"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_topics)
        self.auto_refresh_check.toggled.connect(self.toggle_auto_refresh)
        self.toggle_auto_refresh(True)
    
    def toggle_auto_refresh(self, enabled):
        """切换自动刷新"""
        if enabled:
            self.refresh_timer.start(3000)  # 3秒刷新
        else:
            self.refresh_timer.stop()
    
    def refresh_topics(self):
        """刷新话题列表"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                topics = [topic.strip() for topic in result.stdout.split('\n') if topic.strip()]
                self.update_topic_combo(topics)
                self.update_topic_list(topics)
                
        except Exception as e:
            self.show_error(f"刷新话题列表失败: {e}")
    
    def update_topic_combo(self, topics):
        """更新话题下拉框"""
        current_text = self.topic_combo.currentText()
        self.topic_combo.clear()
        
        for topic in topics:
            self.topic_combo.addItem(topic)
        
        # 恢复当前文本
        if current_text and current_text in topics:
            self.topic_combo.setCurrentText(current_text)
        elif current_text:
            self.topic_combo.setEditText(current_text)
    
    def update_topic_list(self, topics):
        """更新话题列表"""
        self.topic_list.clear()
        
        for topic in topics:
            item = QListWidgetItem(topic)
            
            # 检查是否已订阅
            if topic in self.subscriptions:
                item.setBackground(QColor(200, 255, 200))  # 浅绿色背景
                item.setText(f"✓ {topic}")
            
            self.topic_list.addItem(item)
    
    def on_topic_selected(self):
        """话题选择变化事件"""
        selected_items = self.topic_list.selectedItems()
        if not selected_items:
            return
            
        topic_name = selected_items[0].text().lstrip('✓ ')
        self.show_topic_info(topic_name)
    
    def show_topic_info(self, topic_name):
        """显示话题详细信息"""
        try:
            # 获取话题类型
            type_result = subprocess.run(['ros2', 'topic', 'info', topic_name], 
                                       capture_output=True, text=True)
            
            # 获取话题详情
            detail_result = subprocess.run(['ros2', 'topic', 'info', topic_name, '--verbose'], 
                                        capture_output=True, text=True)
            
            info_text = f"话题: {topic_name}\n"
            info_text += "=" * 50 + "\n"
            info_text += type_result.stdout + "\n" if type_result.returncode == 0 else "无法获取话题信息\n"
            info_text += detail_result.stdout if detail_result.returncode == 0 else ""
            
            self.topic_info_text.setText(info_text)
            
        except Exception as e:
            self.topic_info_text.setText(f"获取话题信息时出错: {e}")
    
    def subscribe_to_topic(self):
        """订阅选中的话题"""
        topic_name = self.topic_combo.currentText().strip()
        if not topic_name:
            QMessageBox.warning(self, "警告", "请输入或选择要订阅的话题名称")
            return
        
        if topic_name in self.subscriptions:
            QMessageBox.information(self, "提示", f"已经订阅了话题: {topic_name}")
            return
        
        try:
            # 获取话题类型
            type_result = subprocess.run(['ros2', 'topic', 'type', topic_name], 
                                       capture_output=True, text=True)
            
            if type_result.returncode != 0:
                QMessageBox.warning(self, "错误", f"无法获取话题 {topic_name} 的类型")
                return
            
            msg_type = type_result.stdout.strip()
            
            # 创建订阅
            subscription = TopicSubscription(topic_name, msg_type, self)
            self.subscriptions[topic_name] = subscription
            
            # 初始化消息历史
            self.message_history[topic_name] = []
            
            self.update_topic_list_display()
            self.message_text.append(f"[{self.get_timestamp()}] 已订阅话题: {topic_name} ({msg_type})")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"订阅话题失败: {e}")
    
    def unsubscribe_from_topic(self):
        """取消订阅"""
        topic_name = self.topic_combo.currentText().strip()
        if not topic_name or topic_name not in self.subscriptions:
            QMessageBox.warning(self, "警告", "请选择已订阅的话题")
            return
        
        try:
            subscription = self.subscriptions.pop(topic_name)
            subscription.destroy()
            
            self.update_topic_list_display()
            self.message_text.append(f"[{self.get_timestamp()}] 已取消订阅话题: {topic_name}")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"取消订阅失败: {e}")
    
    def on_message_received(self, message_data):
        """处理接收到的消息"""
        if self.pause_display_check.isChecked():
            return
            
        topic_name = message_data['topic']
        message = message_data['message']
        timestamp = message_data['timestamp']
        
        # 保存到历史
        if topic_name in self.message_history:
            self.message_history[topic_name].append(message_data)
            # 限制历史数量
            if len(self.message_history[topic_name]) > self.max_history:
                self.message_history[topic_name].pop(0)
        
        # 显示消息
        display_format = self.format_combo.currentText()
        formatted_message = self.format_message(message, display_format)
        
        message_display = f"[{timestamp}] {topic_name}:\n{formatted_message}\n{'-'*80}\n"
        
        self.message_text.append(message_display)
        
        # 自动滚动
        if self.auto_scroll_check.isChecked():
            scrollbar = self.message_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
    
    def format_message(self, message, format_type):
        """格式化消息显示"""
        try:
            if format_type == "JSON格式":
                return json.dumps(message, indent=2, ensure_ascii=False)
            elif format_type == "美化格式":
                return yaml.dump(message, default_flow_style=False, allow_unicode=True)
            else:
                return str(message)
        except:
            return str(message)
    
    def clear_messages(self):
        """清空消息显示"""
        self.message_text.clear()
    
    def on_format_changed(self):
        """显示格式改变事件"""
        # 可以在这里重新格式化当前显示的消息
        pass
    
    def update_topic_list_display(self):
        """更新话题列表显示"""
        self.refresh_topics()
    
    def get_timestamp(self):
        """获取当前时间戳"""
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    def show_error(self, message):
        """显示错误信息"""
        self.message_text.append(f"[ERROR] {message}")

class TopicSubscription:
    """话题订阅封装类"""
    
    def __init__(self, topic_name, message_type, parent_monitor):
        self.topic_name = topic_name
        self.message_type = message_type
        self.parent = parent_monitor
        self.node = None
        self.subscription = None
        
        self.setup_subscription()
    
    def setup_subscription(self):
        """设置订阅"""
        try:
            # 创建临时节点进行订阅
            self.node = rclpy.create_node(f'monitor_{hash(self.topic_name)}')
            
            # 动态导入消息类型
            msg_module = self.import_message_type(self.message_type)
            if not msg_module:
                raise ImportError(f"无法导入消息类型: {self.message_type}")
            
            # 创建订阅
            self.subscription = self.node.create_subscription(
                msg_module,
                self.topic_name,
                self.message_callback,
                10  # QoS队列深度
            )
            
            # 在单独线程中旋转节点
            self.thread = threading.Thread(target=self.spin_node, daemon=True)
            self.thread.start()
            
        except Exception as e:
            print(f"创建订阅失败: {e}")
    
    def import_message_type(self, message_type):
        """动态导入消息类型"""
        try:
            parts = message_type.split('/')
            if len(parts) != 2:
                return None
                
            package_name = parts[0]
            message_name = parts[1]
            
            # 常见消息类型的映射
            common_messages = {
                'std_msgs/msg/String': 'std_msgs.msg.String',
                'std_msgs/msg/Int32': 'std_msgs.msg.Int32',
                'geometry_msgs/msg/Twist': 'geometry_msgs.msg.Twist',
                'sensor_msgs/msg/Image': 'sensor_msgs.msg.Image'
            }
            
            if message_type in common_messages:
                import importlib
                module_path = common_messages[message_type].rsplit('.', 1)[0]
                class_name = common_messages[message_type].rsplit('.', 1)[1]
                
                module = importlib.import_module(module_path)
                return getattr(module, class_name)
            
        except Exception as e:
            print(f"导入消息类型失败: {e}")
        
        return None
    
    def message_callback(self, msg):
        """消息回调函数"""
        try:
            # 将消息转换为字典
            message_dict = self.message_to_dict(msg)
            
            # 发射信号到主线程
            self.parent.message_received.emit({
                'topic': self.topic_name,
                'message': message_dict,
                'timestamp': self.parent.get_timestamp()
            })
            
        except Exception as e:
            print(f"处理消息时出错: {e}")
    
    def message_to_dict(self, msg):
        """将消息对象转换为字典"""
        if hasattr(msg, '__dict__'):
            return msg.__dict__
        else:
            # 对于ROS2消息，使用getattr获取所有字段
            result = {}
            for field in dir(msg):
                if not field.startswith('_'):
                    value = getattr(msg, field)
                    if not callable(value):
                        result[field] = value
            return result
    
    def spin_node(self):
        """旋转节点以接收消息"""
        try:
            while rclpy.ok() and self.subscription is not None:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except:
            pass
    
    def destroy(self):
        """销毁订阅"""
        if self.node:
            self.node.destroy_node()
            self.subscription = None