import os
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                             QLabel, QComboBox, QPushButton, QMessageBox, QTextEdit, QSpinBox, QListWidget)
from PyQt5.QtCore import pyqtSignal, QThread, QTimer

class DDSSelector(QWidget):
    """DDS实现选择器 - 支持动态切换底层DDS实现"""
    
    # 信号定义
    dds_config_changed = pyqtSignal(dict)  # 配置改变信号
    
    def __init__(self, dds_manager):
        super().__init__()
        self.dds_manager = dds_manager
        self.current_config = self.dds_manager.get_current_config()
        self.init_ui()
        self.setup_connections()
        
        # 停止 dds_manager 的自动发现定时器
        self.dds_manager.stop_discovery()
        
        # 仅在初始化时获取一次服务列表
        self.update_services_list()
        
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
        # 使用 dds_manager 获取支持的实现
        for rmw, name in self.dds_manager.get_supported_implementations().items():
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
        self.discover_btn = QPushButton("发现服务")  # 新增发现服务按钮
        
        button_layout.addWidget(self.apply_btn)
        button_layout.addWidget(self.test_btn)
        button_layout.addWidget(self.refresh_btn)
        button_layout.addWidget(self.discover_btn)
        
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
        self.discover_btn.clicked.connect(self.discover_services)  # 连接发现服务按钮
        
        # 连接 dds_manager 的信号
        self.dds_manager.config_changed.connect(self.on_config_changed)
        self.dds_manager.services_updated.connect(self.on_services_updated)
        

    
    def update_current_display(self):
        """更新当前配置显示"""
        config = self.dds_manager.get_current_config()
        implementations = self.dds_manager.get_supported_implementations()
        dds_name = implementations.get(
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
            # 使用 dds_manager 切换配置
            success = self.dds_manager.switch_dds_config(new_domain, new_rmw)
            
            if success:
                self.log_message(f"✅ DDS配置已应用: {new_rmw}, 域ID: {new_domain}")
                self.log_message("⚠️ 注意: 需要重启相关节点才能使配置完全生效")
                
                # 发射配置改变信号
                self.dds_config_changed.emit({
                    'rmw_implementation': new_rmw,
                    'domain_id': new_domain,
                    'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                })
                
                self.update_current_display()
            else:
                self.log_message("❌ 配置应用失败")
                QMessageBox.warning(self, "配置错误", "应用DDS配置时发生错误")
                
        except Exception as e:
            self.log_message(f"❌ 配置应用失败: {str(e)}")
            QMessageBox.warning(self, "配置错误", f"应用DDS配置时发生错误:\n{str(e)}")
    

    
    def test_dds_connection(self):
        """测试DDS连接"""
        self.log_message("🔍 开始DDS连接测试...")
        
        try:
            # 使用 dds_manager 测试连接
            result = self.dds_manager.test_dds_connection()
            
            if result['success']:
                self.log_message(f"✅ 节点发现测试通过，发现 {result['node_count']} 个节点")
                self.log_message(f"✅ 话题发现测试通过，发现 {result['topic_count']} 个话题")
                self.log_message("🎉 DDS连接测试全部通过")
            else:
                self.log_message("❌ 连接测试失败")
                for msg in result['messages']:
                    self.log_message(f"  {msg}")
                    
        except Exception as e:
            self.log_message(f"❌ 连接测试异常: {e}")
    
    def refresh_dds_status(self):
        """刷新DDS状态"""
        self.log_message("🔄 刷新DDS状态...")
        self.update_current_display()
        
        # 检查当前DDS实现的可用性
        current_config = self.dds_manager.get_current_config()
        current_rmw = current_config['rmw_implementation']
        
        if current_rmw and current_rmw != '未设置':
            is_available = self.dds_manager.check_dds_availability(current_rmw)
            status = "可用" if is_available else "不可用"
            self.log_message(f"DDS实现 {current_rmw} 状态: {status}")
        
        # 仅刷新本地配置，不重新发现服务
        self.update_local_config_display()
    
    def discover_services(self):
        """手动发现DDS服务"""
        self.log_message("🔍 开始发现DDS服务...")
        
        try:
            # 手动触发服务发现
            self.dds_manager.refresh_services()
            # 更新服务列表显示
            self.update_services_list()
            self.log_message("✅ 服务发现完成")
        except Exception as e:
            self.log_message(f"❌ 服务发现失败: {e}")
    
    def update_local_config_display(self):
        """仅更新本地配置显示（不重新发现服务）"""
        # 只更新服务列表中的本地配置项
        if self.services_list.count() > 0:
            local_config = self.dds_manager.get_local_dds_config()
            local_text = f"本地 - {local_config.hostname} - {local_config.dds_implementation} ({len(local_config.nodes)}节点, {len(local_config.topics)}话题)"
            self.services_list.item(0).setText(local_text)
    
    def update_services_list(self):
        """更新DDS服务列表"""
        self.services_list.clear()
        
        # 获取本地配置
        local_config = self.dds_manager.get_local_dds_config()
        local_text = f"本地 - {local_config.hostname} - {local_config.dds_implementation} ({len(local_config.nodes)}节点, {len(local_config.topics)}话题)"
        self.services_list.addItem(local_text)
        
        # 获取发现的远程服务
        services = self.dds_manager.get_discovered_services()
        for service in services:
            item_text = f"远程 - {service.hostname} - {service.dds_implementation} ({len(service.nodes)}节点, {len(service.topics)}话题)"
            self.services_list.addItem(item_text)
        
        if not services:
            self.services_list.addItem("暂无发现远程服务")
    
    def on_config_changed(self, config):
        """处理配置改变事件"""
        self.update_current_display()
        self.log_message(f"🔔 配置已更新: {config['rmw_implementation']}, 域ID: {config['domain_id']}")
    
    def on_services_updated(self, services):
        """处理服务列表更新事件"""
        self.update_services_list()
        self.log_message(f"🔄 发现 {len(services)} 个远程服务")
    
    def log_message(self, message):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.append(f"[{timestamp}] {message}")
        # 自动滚动到底部
        scrollbar = self.status_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())