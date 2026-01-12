import os
import subprocess
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QLabel, QComboBox, QPushButton, QMessageBox, QTextEdit, QSpinBox, QListWidget)
from PyQt5.QtCore import pyqtSignal, QThread, QTimer, QDateTime

class DDSSelector(QWidget):
    """DDS实现选择器 - 支持动态切换底层DDS实现"""
    
    # 信号定义
    dds_config_changed = pyqtSignal(dict)  # 配置改变信号
    
    # 支持的DDS实现
    SUPPORTED_DDS_IMPLEMENTATIONS = {
        'rmw_fastrtps_cpp': 'Fast DDS (默认)',
        'rmw_cyclonedds_cpp': 'Cyclone DDS',
        'rmw_connextdds': 'Connext DDS (需许可)'
    }
    
    def __init__(self, dds_manager):
        super().__init__()
        self.dds_manager = dds_manager
        self.current_config = self.get_current_dds_config()
        self.init_ui()
        self.setup_connections()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 当前配置显示组
        current_group = QGroupBox("当前DDS配置")
        current_layout = QVBoxLayout()
        
        self.current_dds_label = QLabel("DDS实现: 检测中...")
        self.current_domain_label = QLabel("域ID: 检测中...")
        self.config_status_label = QLabel("状态: 检测中...")
        
        current_layout.addWidget(self.current_dds_label)
        current_layout.addWidget(self.current_domain_label)
        current_layout.addWidget(self.config_status_label)
        current_group.setLayout(current_layout)
        
        # 配置切换组
        switch_group = QGroupBox("切换DDS配置")
        switch_layout = QVBoxLayout()
        
        # DDS实现选择
        dds_layout = QHBoxLayout()
        dds_layout.addWidget(QLabel("选择DDS实现:"))
        self.dds_combo = QComboBox()
        for rmw, name in self.SUPPORTED_DDS_IMPLEMENTATIONS.items():
            self.dds_combo.addItem(name, rmw)
        dds_layout.addWidget(self.dds_combo)
        
        # 域ID设置
        domain_layout = QHBoxLayout()
        domain_layout.addWidget(QLabel("域ID (0-232):"))
        self.domain_spin = QSpinBox()
        self.domain_spin.setRange(0, 232)
        self.domain_spin.setValue(0)
        domain_layout.addWidget(self.domain_spin)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        self.apply_btn = QPushButton("应用配置")
        self.test_btn = QPushButton("测试连接")
        self.refresh_btn = QPushButton("刷新状态")
        
        button_layout.addWidget(self.apply_btn)
        button_layout.addWidget(self.test_btn)
        button_layout.addWidget(self.refresh_btn)
        
        switch_layout.addLayout(dds_layout)
        switch_layout.addLayout(domain_layout)
        switch_layout.addLayout(button_layout)
        switch_group.setLayout(switch_layout)
        
        # DDS服务发现组
        services_group = QGroupBox("发现的DDS服务")
        services_layout = QVBoxLayout()
        self.services_list = QListWidget()
        services_layout.addWidget(self.services_list)
        services_group.setLayout(services_layout)
        
        # 状态信息显示
        status_group = QGroupBox("操作日志")
        status_layout = QVBoxLayout()
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        status_layout.addWidget(self.status_text)
        status_group.setLayout(status_layout)
        
        layout.addWidget(current_group)
        layout.addWidget(switch_group)
        layout.addWidget(services_group)
        layout.addWidget(status_group)
        
        self.setLayout(layout)
        self.update_current_display()
        
    def setup_connections(self):
        """设置信号连接"""
        self.apply_btn.clicked.connect(self.apply_dds_config)
        self.test_btn.clicked.connect(self.test_dds_connection)
        self.refresh_btn.clicked.connect(self.refresh_dds_status)
        
    def get_current_dds_config(self):
        """获取当前DDS配置"""
        return {
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '未设置'),
            'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'status': '未知'
        }
    
    def update_current_display(self):
        """更新当前配置显示"""
        config = self.get_current_dds_config()
        dds_name = self.SUPPORTED_DDS_IMPLEMENTATIONS.get(
            config['rmw_implementation'], config['rmw_implementation']
        )
        
        self.current_dds_label.setText(f"DDS实现: {dds_name}")
        self.current_domain_label.setText(f"域ID: {config['domain_id']}")
        self.config_status_label.setText(f"状态: {config['status']}")
        
    def apply_dds_config(self):
        """应用新的DDS配置"""
        new_rmw = self.dds_combo.currentData()
        new_domain = str(self.domain_spin.value())
        
        try:
            # 验证域ID范围
            domain_int = int(new_domain)
            if not 0 <= domain_int <= 232:
                raise ValueError("域ID必须在0-232之间")
            
            # 检查DDS实现是否可用
            if not self.check_dds_availability(new_rmw):
                raise ValueError(f"DDS实现 {new_rmw} 不可用")
            
            # 设置环境变量
            os.environ['RMW_IMPLEMENTATION'] = new_rmw
            os.environ['ROS_DOMAIN_ID'] = new_domain
            
            self.log_message(f"✅ DDS配置已应用: {new_rmw}, 域ID: {new_domain}")
            self.log_message("⚠️ 注意: 需要重启相关节点才能使配置完全生效")
            
            # 发射配置改变信号
            self.dds_config_changed.emit({
                'rmw_implementation': new_rmw,
                'domain_id': new_domain,
                'timestamp': '刚刚'
            })
            
            self.update_current_display()
            
        except Exception as e:
            self.log_message(f"❌ 配置应用失败: {str(e)}")
            QMessageBox.warning(self, "配置错误", f"应用DDS配置时发生错误:\n{str(e)}")
    
    def check_dds_availability(self, rmw_implementation):
        """检查DDS实现是否可用"""
        try:
            # 临时设置环境变量并测试ROS2命令
            env = os.environ.copy()
            env['RMW_IMPLEMENTATION'] = rmw_implementation
            env['ROS_DOMAIN_ID'] = '0'  # 使用默认域进行测试
            
            result = subprocess.run([
                'ros2', 'node', 'list'
            ], env=env, capture_output=True, text=True, timeout=5.0)
            
            return result.returncode == 0 or "RMW implementation not found" not in result.stderr
            
        except subprocess.TimeoutExpired:
            self.log_message(f"⚠️ DDS实现 {rmw_implementation} 检查超时")
            return True  # 超时不一定表示不可用
        except Exception as e:
            self.log_message(f"⚠️ DDS实现检查异常: {e}")
            return False
    
    def test_dds_connection(self):
        """测试DDS连接"""
        self.log_message("🔍 开始DDS连接测试...")
        
        try:
            # 测试节点发现
            result = subprocess.run([
                'ros2', 'node', 'list'
            ], capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                nodes = [node for node in result.stdout.split('\n') if node.strip()]
                self.log_message(f"✅ 节点发现测试通过，发现 {len(nodes)} 个节点")
                
                # 测试话题列表
                topic_result = subprocess.run([
                    'ros2', 'topic', 'list'
                ], capture_output=True, text=True, timeout=5.0)
                
                if topic_result.returncode == 0:
                    topics = [topic for topic in topic_result.stdout.split('\n') if topic.strip()]
                    self.log_message(f"✅ 话题发现测试通过，发现 {len(topics)} 个话题")
                    self.log_message("🎉 DDS连接测试全部通过")
                else:
                    self.log_message("⚠️ 话题发现测试失败，但节点发现正常")
                    
            else:
                self.log_message("❌ 节点发现测试失败，请检查DDS配置")
                
        except Exception as e:
            self.log_message(f"❌ 连接测试异常: {e}")
    
    def refresh_dds_status(self):
        """刷新DDS状态"""
        self.log_message("🔄 刷新DDS状态...")
        self.update_current_display()
        
        # 检查当前DDS实现的可用性
        current_rmw = os.environ.get('RMW_IMPLEMENTATION', '')
        if current_rmw:
            is_available = self.check_dds_availability(current_rmw)
            status = "可用" if is_available else "不可用"
            self.log_message(f"DDS实现 {current_rmw} 状态: {status}")
        
        # 更新服务列表
        self.update_services_list()
    
    def update_services_list(self):
        """更新DDS服务列表"""
        self.services_list.clear()
        services = self.dds_manager.get_discovered_services()
        for service in services:
            item_text = f"远程 - {service.hostname} - {service.dds_implementation} ({len(service.nodes)}节点, {len(service.topics)}话题)"
            self.services_list.addItem(item_text)
        
        # 可以扩展显示发现的远程DDS服务
        # 这里预留接口供后续实现DDS服务发现功能
    
    def log_message(self, message):
        """添加日志消息"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.status_text.append(f"[{timestamp}] {message}")
        # 自动滚动到底部
        scrollbar = self.status_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())