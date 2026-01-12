import json
import os
from typing import Dict, Any
from PyQt5.QtCore import QSettings

class ConfigManager:
    """配置管理器"""
    
    def __init__(self):
        self.settings = QSettings('ROS2_DDS_GUI', 'Config')
        self.config_file = os.path.expanduser('~/.ros2_dds_gui/config.json')
        os.makedirs(os.path.dirname(self.config_file), exist_ok=True)

    def save_config(self, config: Dict[str, Any]):
        """保存配置到文件"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            print(f"Error saving config: {e}")

    def load_config(self) -> Dict[str, Any]:
        """从文件加载配置"""
        try:
            with open(self.config_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return self.get_default_config()
        except Exception as e:
            print(f"Error loading config: {e}")
            return self.get_default_config()

    def get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            'window_geometry': None,
            'window_state': None,
            'recent_domain_id': '0',
            'recent_dds_implementation': 'rmw_fastrtps_cpp',
            'update_interval': 2000,
            'auto_refresh': True
        }

    def save_window_state(self, geometry, state):
        """保存窗口状态"""
        self.settings.setValue('geometry', geometry)
        self.settings.setValue('windowState', state)

    def load_window_state(self):
        """加载窗口状态"""
        return (
            self.settings.value('geometry'),
            self.settings.value('windowState')
        )