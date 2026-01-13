from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTabWidget, QStatusBar, QMenuBar, QMenu, QAction,
                             QSplitter, QDockWidget, QFrame)
from PyQt5.QtCore import Qt, QTimer
from managers.config_manager import ConfigManager
from managers.dds_manager import DDSManager
from managers.node_manager import NodeManager
from managers.topic_manager import TopicManager
from managers.graph_manager import GraphManager
from managers.service_manager import ServiceManager
from managers.ros2_manager import ROS2Manager


from widgets.dds_selector import DDSSelector
from widgets.node_explorer import NodeExplorer
from widgets.topic_monitor import TopicMonitor
from widgets.service_caller import ServiceCaller
from widgets.graph_view import GraphView
from widgets.status_panel import StatusPanel
class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self):
        super().__init__()
        self.dds_manager = DDSManager()
        self.config_manager = ConfigManager()
        self.node_manager = NodeManager()
        self.topic_manager = TopicManager()
        self.graph_manager = GraphManager()
        self.service_manager = ServiceManager()
        self.ros2_manager = ROS2Manager()
        
        self.init_ui()
    def init_ui(self):
        self.setWindowTitle("ROS2 DDS GUI Tool")
        self.setGeometry(100, 100, 1400, 900)

        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局 - 改为垂直布局以容纳底部状态面板
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 创建中心标签页
        self.tab_widget = QTabWidget()
        self.tab_widget.addTab(NodeExplorer(self.node_manager, self.dds_manager), "节点浏览器")
        self.tab_widget.addTab(TopicMonitor(self.topic_manager, self.dds_manager), "话题监控")
        self.tab_widget.addTab(ServiceCaller(self.service_manager, self.dds_manager), "服务调用")
        self.tab_widget.addTab(GraphView(self.graph_manager, self.dds_manager), "拓扑视图")
        main_layout.addWidget(self.tab_widget, 1)  # 比例1，占据主要空间
        
        # 创建底部状态面板容器
        bottom_container = QWidget()
        bottom_layout = QHBoxLayout(bottom_container)
        bottom_layout.setContentsMargins(5, 2, 5, 2)
        bottom_layout.setSpacing(0)
        
        # 左侧占位（可以放其他信息）
        bottom_layout.addStretch(1)
        
        # 右侧状态面板
        self.status_panel = StatusPanel(self.ros2_manager)
        bottom_layout.addWidget(self.status_panel)
        
        # 设置底部容器样式
        bottom_container.setStyleSheet("""
            QWidget {
                background-color: #f5f5f5;
                border-top: 1px solid #ddd;
            }
        """)
        bottom_container.setFixedHeight(45)
        
        main_layout.addWidget(bottom_container)

        # 创建左侧停靠窗口
        self.create_left_dock()

        # 创建状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")
        
        # 创建菜单栏
        self.create_menus()

    def create_left_dock(self):
        """创建左侧停靠窗口"""
        left_dock = QDockWidget("DDS配置", self)
        left_widget = QWidget()
        layout = QVBoxLayout(left_widget)
        
        # 添加DDS选择器(已整合域管理功能)
        self.dds_selector = DDSSelector(self.dds_manager)
        layout.addWidget(self.dds_selector)
        
        left_dock.setWidget(left_widget)
        left_dock.setFixedWidth(300)
        self.addDockWidget(Qt.LeftDockWidgetArea, left_dock)

    def create_menus(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu('文件')
        
        exit_action = QAction('退出', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # 查看菜单
        view_menu = menubar.addMenu('查看')
        # 可以添加显示/隐藏各种面板的选项
        
        # 工具菜单
        tools_menu = menubar.addMenu('工具')
        refresh_action = QAction('刷新', self)
        refresh_action.setShortcut('F5')
        refresh_action.triggered.connect(self.refresh_all)
        tools_menu.addAction(refresh_action)
    def refresh_all(self):
        """刷新所有组件"""
        # 刷新状态面板
        self.status_panel.refresh_status()
        self.status_bar.showMessage("已刷新")
        
    def shutdown(self):
        """关闭时清理资源"""
        pass
