import subprocess
import json
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QListWidget,
                             QTextEdit, QSplitter, QGroupBox, QPushButton,
                             QLineEdit, QComboBox, QLabel, QTableWidget,
                             QTableWidgetItem, QHeaderView, QTabWidget,QCheckBox,QMessageBox)
from PyQt5.QtCore import QTimer, Qt
import yaml

class ServiceCaller(QWidget):
    """服务调用器 - 发现和调用ROS2服务"""
    
    def __init__(self, dds_manager):
        super().__init__()
        self.dds_manager = dds_manager
        self.services_info = {}
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout()
        
        # 工具栏
        toolbar = QHBoxLayout()
        
        self.refresh_btn = QPushButton("刷新服务列表")
        self.auto_refresh_check = QCheckBox("自动刷新")
        self.auto_refresh_check.setChecked(True)
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("过滤服务名称...")
        
        toolbar.addWidget(self.refresh_btn)
        toolbar.addWidget(self.auto_refresh_check)
        toolbar.addWidget(QLabel("过滤:"))
        toolbar.addWidget(self.filter_edit)
        toolbar.addStretch()
        
        # 主内容区域
        splitter = QSplitter(Qt.Horizontal)
        
        # 服务列表
        service_list_group = QGroupBox("可用服务")
        service_list_layout = QVBoxLayout()
        
        self.service_list = QListWidget()
        self.service_list.itemSelectionChanged.connect(self.on_service_selected)
        
        service_list_layout.addWidget(self.service_list)
        service_list_group.setLayout(service_list_layout)
        
        # 服务详情和调用界面
        detail_tabs = QTabWidget()
        
        # 服务信息标签
        info_tab = QWidget()
        info_layout = QVBoxLayout(info_tab)
        
        self.service_info_text = QTextEdit()
        self.service_info_text.setReadOnly(True)
        
        info_layout.addWidget(self.service_info_text)
        
        # 服务调用标签
        call_tab = QWidget()
        call_layout = QVBoxLayout(call_tab)
        
        # 请求参数输入
        request_group = QGroupBox("请求参数")
        request_layout = QVBoxLayout()
        
        self.request_format_combo = QComboBox()
        self.request_format_combo.addItems(["YAML", "JSON"])
        self.request_text = QTextEdit()
        self.request_text.setPlaceholderText("输入请求参数...")
        
        request_layout.addWidget(QLabel("参数格式:"))
        request_layout.addWidget(self.request_format_combo)
        request_layout.addWidget(self.request_text)
        request_group.setLayout(request_layout)
        
        # 调用按钮和结果
        call_control_layout = QHBoxLayout()
        self.call_btn = QPushButton("调用服务")
        self.clear_btn = QPushButton("清空结果")
        
        call_control_layout.addWidget(self.call_btn)
        call_control_layout.addWidget(self.clear_btn)
        call_control_layout.addStretch()
        
        # 响应结果
        response_group = QGroupBox("响应结果")
        response_layout = QVBoxLayout()
        
        self.response_text = QTextEdit()
        self.response_text.setReadOnly(True)
        
        response_layout.addWidget(self.response_text)
        response_group.setLayout(response_layout)
        
        call_layout.addWidget(request_group)
        call_layout.addLayout(call_control_layout)
        call_layout.addWidget(response_group)
        
        detail_tabs.addTab(info_tab, "服务信息")
        detail_tabs.addTab(call_tab, "调用服务")
        
        splitter.addWidget(service_list_group)
        splitter.addWidget(detail_tabs)
        splitter.setSizes([300, 700])
        
        layout.addLayout(toolbar)
        layout.addWidget(splitter)
        
        self.setLayout(layout)
        
        # 连接信号
        self.refresh_btn.clicked.connect(self.refresh_services)
        self.call_btn.clicked.connect(self.call_service)
        self.clear_btn.clicked.connect(self.clear_results)
        self.filter_edit.textChanged.connect(self.filter_services)
        
        self.refresh_services()
    
    def setup_timers(self):
        """设置定时器"""
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_services)
        self.auto_refresh_check.toggled.connect(self.toggle_auto_refresh)
        self.toggle_auto_refresh(True)
    
    def toggle_auto_refresh(self, enabled):
        """切换自动刷新"""
        if enabled:
            self.refresh_timer.start(5000)  # 5秒刷新
        else:
            self.refresh_timer.stop()
    
    def refresh_services(self):
        """刷新服务列表"""
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                services = [s.strip() for s in result.stdout.split('\n') if s.strip()]
                self.update_service_list(services)
                self.get_services_info(services)
            else:
                self.service_info_text.setText("无法获取服务列表")
                
        except Exception as e:
            self.service_info_text.setText(f"刷新服务列表失败: {e}")
    
    def update_service_list(self, services):
        """更新服务列表"""
        self.service_list.clear()
        
        for service in services:
            self.service_list.addItem(service)
        
        self.apply_filter()
    
    def get_services_info(self, services):
        """获取服务详细信息"""
        for service in services:
            try:
                result = subprocess.run(['ros2', 'service', 'type', service], 
                                      capture_output=True, text=True)
                
                if result.returncode == 0:
                    service_type = result.stdout.strip()
                    self.services_info[service] = {
                        'type': service_type,
                        'name': service
                    }
                    
            except Exception as e:
                print(f"获取服务 {service} 信息失败: {e}")
    
    def on_service_selected(self):
        """服务选择变化事件"""
        selected_items = self.service_list.selectedItems()
        if not selected_items:
            return
            
        service_name = selected_items[0].text()
        self.show_service_info(service_name)
    
    def show_service_info(self, service_name):
        """显示服务详细信息"""
        info = self.services_info.get(service_name, {})
        
        info_text = f"服务名称: {service_name}\n"
        info_text += "=" * 50 + "\n"
        info_text += f"服务类型: {info.get('type', '未知')}\n\n"
        
        # 显示服务接口信息
        if 'type' in info:
            try:
                # 尝试获取服务定义
                result = subprocess.run(['ros2', 'interface', 'show', info['type']], 
                                      capture_output=True, text=True)
                
                if result.returncode == 0:
                    info_text += "服务定义:\n"
                    info_text += result.stdout
                    
            except Exception as e:
                info_text += f"获取服务定义失败: {e}\n"
        
        self.service_info_text.setText(info_text)
    
    def call_service(self):
        """调用选中的服务"""
        selected_items = self.service_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "警告", "请先选择一个服务")
            return
        
        service_name = selected_items[0].text()
        request_text = self.request_text.toPlainText().strip()
        
        if not request_text:
            QMessageBox.warning(self, "警告", "请输入请求参数")
            return
        
        try:
            # 解析请求参数
            format_type = self.request_format_combo.currentText()
            if format_type == "YAML":
                request_data = yaml.safe_load(request_text)
            else:  # JSON
                request_data = json.loads(request_text)
            
            # 调用服务
            self.call_ros2_service(service_name, request_data)
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"调用服务失败: {e}")
    
    def call_ros2_service(self, service_name, request_data):
        """调用ROS2服务"""
        try:
            # 获取服务类型
            service_type = self.services_info.get(service_name, {}).get('type', '')
            if not service_type:
                raise ValueError("无法获取服务类型")
            
            # 构建请求文件
            import tempfile
            import os
            
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
                yaml.dump(request_data, f)
                temp_file = f.name
            
            try:
                # 调用ros2 service call命令
                result = subprocess.run([
                    'ros2', 'service', 'call',
                    service_name,
                    service_type,
                    request_data  # 直接传递请求数据
                ], capture_output=True, text=True, timeout=10.0)
                
                if result.returncode == 0:
                    self.response_text.setText(f"服务调用成功:\n{result.stdout}")
                else:
                    self.response_text.setText(f"服务调用失败:\n{result.stderr}")
                    
            finally:
                # 清理临时文件
                if os.path.exists(temp_file):
                    os.unlink(temp_file)
                
        except subprocess.TimeoutExpired:
            self.response_text.setText("服务调用超时")
        except Exception as e:
            self.response_text.setText(f"服务调用异常: {e}")
    
    def clear_results(self):
        """清空结果"""
        self.response_text.clear()
        self.request_text.clear()
    
    def filter_services(self):
        """过滤服务"""
        self.apply_filter()
    
    def apply_filter(self):
        """应用过滤"""
        filter_text = self.filter_edit.text().lower()
        
        for i in range(self.service_list.count()):
            item = self.service_list.item(i)
            service_name = item.text().lower()
            visible = filter_text in service_name if filter_text else True
            item.setHidden(not visible)