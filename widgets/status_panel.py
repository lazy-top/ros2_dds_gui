from PyQt5.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel, 
                             QFrame, QPushButton, QGroupBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette


class StatusPanel(QWidget):
    """状态面板 - 显示ROS2和DDS服务状态"""
    
    status_updated = pyqtSignal(dict)  # 状态更新信号
    
    def __init__(self, ros2_manager):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.init_ui()
        
        # 初始化时获取一次状态
        self.refresh_status()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(15)
        
        # DDS状态指示器
        self.dds_indicator = StatusIndicator("DDS服务")
        layout.addWidget(self.dds_indicator)
        
        # 分隔符
        separator1 = QFrame()
        separator1.setFrameShape(QFrame.VLine)
        separator1.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator1)
        
        # ROS2版本信息
        self.ros2_info_label = QLabel("ROS2: 检测中...")
        self.ros2_info_label.setStyleSheet("color: #666;")
        layout.addWidget(self.ros2_info_label)
        
        # 分隔符
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.VLine)
        separator2.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator2)
        
        # 域ID显示
        self.domain_label = QLabel("域ID: 0")
        self.domain_label.setStyleSheet("color: #666;")
        layout.addWidget(self.domain_label)
        
        # 分隔符
        separator3 = QFrame()
        separator3.setFrameShape(QFrame.VLine)
        separator3.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator3)
        
        # 节点/话题统计
        self.stats_label = QLabel("节点: 0 | 话题: 0")
        self.stats_label.setStyleSheet("color: #666;")
        layout.addWidget(self.stats_label)
        
        # 刷新按钮
        self.refresh_btn = QPushButton("刷新状态")
        self.refresh_btn.setFixedWidth(80)
        self.refresh_btn.clicked.connect(self.refresh_status)
        layout.addWidget(self.refresh_btn)
        
        layout.addStretch()
        self.setLayout(layout)
        
        # 设置最大高度
        self.setMaximumHeight(40)
        
    def refresh_status(self):
        """刷新状态信息"""
        try:
            # 获取完整状态
            full_status = self.ros2_manager.get_full_status()
            
            # 更新DDS状态指示器
            dds_status = full_status['dds_status']
            if dds_status['running']:
                self.dds_indicator.set_status(True, dds_status['message'])
            else:
                self.dds_indicator.set_status(False, dds_status['message'])
            
            # 更新ROS2版本信息
            distro = full_status['ros2_distro']
            version = full_status['ros2_version']
            if distro:
                ros2_text = f"ROS2: {distro.capitalize()}"
                if version:
                    ros2_text += f" ({version})"
                self.ros2_info_label.setText(ros2_text)
                self.ros2_info_label.setStyleSheet("color: #2e7d32;")  # 绿色
            else:
                self.ros2_info_label.setText("ROS2: 未检测到")
                self.ros2_info_label.setStyleSheet("color: #c62828;")  # 红色
            
            # 更新域ID
            domain_id = dds_status['domain_id']
            self.domain_label.setText(f"域ID: {domain_id}")
            
            # 更新节点/话题统计
            node_count = dds_status['node_count']
            topic_count = dds_status['topic_count']
            self.stats_label.setText(f"节点: {node_count} | 话题: {topic_count}")
            
            # 发射状态更新信号
            self.status_updated.emit(full_status)
            
        except Exception as e:
            self.dds_indicator.set_status(False, f"状态检测失败: {e}")
            self.ros2_info_label.setText("ROS2: 检测失败")
            self.ros2_info_label.setStyleSheet("color: #c62828;")
    
    def get_status_text(self) -> str:
        """获取当前状态的文本描述"""
        try:
            full_status = self.ros2_manager.get_full_status()
            dds_status = full_status['dds_status']
            
            text = f"DDS: {'运行中' if dds_status['running'] else '未运行'}"
            text += f" | ROS2: {full_status['ros2_distro'] or '未知'}"
            text += f" | 域ID: {dds_status['domain_id']}"
            text += f" | 节点: {dds_status['node_count']}"
            
            return text
        except:
            return "状态未知"


class StatusIndicator(QWidget):
    """状态指示器组件 - 显示一个带状态灯的标签"""
    
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.init_ui()
        
    def init_ui(self):
        """初始化界面"""
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # 状态灯
        self.indicator_light = QLabel("●")
        self.indicator_light.setStyleSheet("color: #9e9e9e; font-size: 12px;")  # 灰色
        layout.addWidget(self.indicator_light)
        
        # 名称标签
        self.name_label = QLabel(self.name + ":")
        self.name_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.name_label)
        
        # 状态文本
        self.status_label = QLabel("检测中...")
        self.status_label.setStyleSheet("color: #666;")
        layout.addWidget(self.status_label)
        
        self.setLayout(layout)
    
    def set_status(self, is_online: bool, message: str = ""):
        """设置状态
        
        Args:
            is_online: 是否在线
            message: 状态消息
        """
        if is_online:
            self.indicator_light.setStyleSheet("color: #4caf50; font-size: 12px;")  # 绿色
            self.status_label.setText(message if message else "在线")
            self.status_label.setStyleSheet("color: #2e7d32;")
        else:
            self.indicator_light.setStyleSheet("color: #f44336; font-size: 12px;")  # 红色
            self.status_label.setText(message if message else "离线")
            self.status_label.setStyleSheet("color: #c62828;")
