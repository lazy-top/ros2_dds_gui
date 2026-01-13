import subprocess
import json
import threading
import time
from typing import List, Dict, Optional, Callable, Any
import importlib

try:
    import rclpy
    from rclpy.node import Node
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    print("警告: rclpy 未安装，部分功能可能不可用")
import rclpy
from rclpy.node import Node

class TopicManager:
    """ROS2话题管理器 - 提供话题操作的核心功能"""
    
    def __init__(self):
        self.topics_cache = {}
        self.subscriptions = {}
        self.rclpy_initialized = False
        self.executor = None
        self.executor_thread = None
        
        # 初始化rclpy
        if RCLPY_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.rclpy_initialized = True
            except Exception as e:
                print(f"初始化rclpy失败: {e}")
    
    def get_topics(self) -> List[str]:
        """获取所有活跃的ROS2话题列表
        
        Returns:
            List[str]: 话题名称列表
        """
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics = [topic.strip() for topic in result.stdout.split('\n') if topic.strip()]
                return topics
            return []
        except Exception as e:
            print(f"获取话题列表失败: {e}")
            return []
    
    def get_topic_type(self, topic_name: str) -> Optional[str]:
        """获取指定话题的消息类型
        
        Args:
            topic_name: 话题名称
            
        Returns:
            Optional[str]: 话题的消息类型，如 'std_msgs/msg/String'
        """
        try:
            result = subprocess.run(['ros2', 'topic', 'type', topic_name],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout.strip()
            return None
        except Exception as e:
            print(f"获取话题 {topic_name} 类型失败: {e}")
            return None
    
    def get_topic_info(self, topic_name: str, verbose: bool = False) -> Optional[str]:
        """获取话题的详细信息
        
        Args:
            topic_name: 话题名称
            verbose: 是否显示详细信息
            
        Returns:
            Optional[str]: 话题信息的原始文本输出
        """
        try:
            cmd = ['ros2', 'topic', 'info', topic_name]
            if verbose:
                cmd.append('--verbose')
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout
            return None
        except Exception as e:
            print(f"获取话题 {topic_name} 信息失败: {e}")
            return None
    
    def parse_topic_info(self, info_text: str, topic_name: str) -> Dict:
        """解析话题信息文本
        
        Args:
            info_text: ros2 topic info 命令的输出文本
            topic_name: 话题名称
            
        Returns:
            Dict: 包含话题详细信息的字典
        """
        info = {
            'name': topic_name,
            'type': '未知',
            'publisher_count': 0,
            'subscriber_count': 0,
            'publishers': [],
            'subscribers': [],
            'qos_profile': {}
        }
        
        if not info_text:
            return info
        
        lines = info_text.split('\n')
        current_section = ''
        
        for line in lines:
            line_stripped = line.strip()
            
            if 'Type:' in line:
                info['type'] = line.split('Type:')[-1].strip()
            elif 'Publisher count:' in line:
                try:
                    info['publisher_count'] = int(line.split(':')[-1].strip())
                except:
                    pass
            elif 'Subscription count:' in line or 'Subscriber count:' in line:
                try:
                    info['subscriber_count'] = int(line.split(':')[-1].strip())
                except:
                    pass
            elif 'Publishers:' in line:
                current_section = 'publishers'
            elif 'Subscribers:' in line or 'Subscriptions:' in line:
                current_section = 'subscribers'
            elif current_section and line_stripped and not line_stripped.startswith('QoS'):
                if current_section == 'publishers':
                    info['publishers'].append(line_stripped)
                elif current_section == 'subscribers':
                    info['subscribers'].append(line_stripped)
        
        return info
    
    def get_parsed_topic_info(self, topic_name: str) -> Dict:
        """获取并解析话题信息
        
        Args:
            topic_name: 话题名称
            
        Returns:
            Dict: 解析后的话题信息字典
        """
        info_text = self.get_topic_info(topic_name, verbose=True)
        if info_text:
            parsed_info = self.parse_topic_info(info_text, topic_name)
            self.topics_cache[topic_name] = parsed_info
            return parsed_info
        return self.get_empty_topic_info(topic_name)
    
    def get_all_topics_info(self) -> Dict[str, Dict]:
        """获取所有话题的详细信息
        
        Returns:
            Dict[str, Dict]: 话题名称到话题信息的映射
        """
        topics = self.get_topics()
        all_info = {}
        
        for topic_name in topics:
            info = self.get_parsed_topic_info(topic_name)
            all_info[topic_name] = info
        
        return all_info
    
    def echo_topic(self, topic_name: str, count: int = 1) -> Optional[str]:
        """获取话题的消息内容（使用 ros2 topic echo）
        
        Args:
            topic_name: 话题名称
            count: 要获取的消息数量
            
        Returns:
            Optional[str]: 消息内容文本
        """
        try:
            cmd = ['ros2', 'topic', 'echo', topic_name, '--once']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                return result.stdout
            return None
        except Exception as e:
            print(f"获取话题 {topic_name} 消息失败: {e}")
            return None
    
    def get_topic_hz(self, topic_name: str, window: int = 10) -> Optional[float]:
        """获取话题的发布频率
        
        Args:
            topic_name: 话题名称
            window: 计算频率的窗口大小（消息数量）
            
        Returns:
            Optional[float]: 话题频率（Hz）
        """
        try:
            cmd = ['ros2', 'topic', 'hz', topic_name]
            # 这个命令会持续运行，需要在一定时间后终止
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0 or result.stdout:
                # 解析输出获取频率信息
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'average rate:' in line.lower():
                        try:
                            hz = float(line.split(':')[-1].strip().split()[0])
                            return hz
                        except:
                            pass
            return None
        except subprocess.TimeoutExpired as e:
            # 超时是正常的，尝试从输出中解析
            if e.stdout:
                lines = e.stdout.decode().split('\n')
                for line in lines:
                    if 'average rate:' in line.lower():
                        try:
                            hz = float(line.split(':')[-1].strip().split()[0])
                            return hz
                        except:
                            pass
            return None
        except Exception as e:
            print(f"获取话题 {topic_name} 频率失败: {e}")
            return None
    
    def get_topic_bandwidth(self, topic_name: str) -> Optional[str]:
        """获取话题的带宽信息
        
        Args:
            topic_name: 话题名称
            
        Returns:
            Optional[str]: 带宽信息字符串
        """
        try:
            cmd = ['ros2', 'topic', 'bw', topic_name]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0 or result.stdout:
                return result.stdout.strip()
            return None
        except Exception as e:
            print(f"获取话题 {topic_name} 带宽失败: {e}")
            return None
    
    def publish_message(self, topic_name: str, message_type: str, message_data: str) -> bool:
        """发布消息到指定话题
        
        Args:
            topic_name: 话题名称
            message_type: 消息类型
            message_data: 消息数据（YAML格式字符串）
            
        Returns:
            bool: 是否发布成功
        """
        try:
            cmd = ['ros2', 'topic', 'pub', '--once', topic_name, message_type, message_data]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except Exception as e:
            print(f"发布消息到话题 {topic_name} 失败: {e}")
            return False
    
    def create_subscription(self, topic_name: str, message_type: str, 
                          callback: Callable, qos_depth: int = 10) -> Optional['TopicSubscription']:
        """创建话题订阅
        
        Args:
            topic_name: 话题名称
            message_type: 消息类型
            callback: 消息回调函数
            qos_depth: QoS队列深度
            
        Returns:
            Optional[TopicSubscription]: 订阅对象
        """
        if not self.rclpy_initialized:
            print("rclpy未初始化，无法创建订阅")
            return None
        
        try:
            subscription = TopicSubscription(
                topic_name=topic_name,
                message_type=message_type,
                callback=callback,
                qos_depth=qos_depth,
                manager=self
            )
            
            self.subscriptions[topic_name] = subscription
            return subscription
            
        except Exception as e:
            print(f"创建订阅失败: {e}")
            return None
    
    def destroy_subscription(self, topic_name: str) -> bool:
        """销毁话题订阅
        
        Args:
            topic_name: 话题名称
            
        Returns:
            bool: 是否成功销毁
        """
        if topic_name in self.subscriptions:
            try:
                subscription = self.subscriptions.pop(topic_name)
                subscription.destroy()
                return True
            except Exception as e:
                print(f"销毁订阅失败: {e}")
                return False
        return False
    
    def import_message_type(self, message_type: str) -> Optional[type]:
        """动态导入消息类型
        
        Args:
            message_type: 消息类型字符串，如 'std_msgs/msg/String'
            
        Returns:
            Optional[type]: 消息类型类
        """
        try:
            # 解析消息类型
            parts = message_type.split('/')
            if len(parts) < 2:
                return None
            
            package_name = parts[0]
            msg_name = parts[-1]
            
            # 构建模块路径
            if len(parts) == 3:  # 例如: std_msgs/msg/String
                module_path = f"{package_name}.{parts[1]}"
            else:  # 例如: std_msgs/String
                module_path = f"{package_name}.msg"
            
            # 动态导入
            module = importlib.import_module(module_path)
            msg_class = getattr(module, msg_name)
            return msg_class
            
        except Exception as e:
            print(f"导入消息类型 {message_type} 失败: {e}")
            return None
    
    def get_empty_topic_info(self, topic_name: str) -> Dict:
        """获取空的话题信息模板
        
        Args:
            topic_name: 话题名称
            
        Returns:
            Dict: 空的话题信息字典
        """
        return {
            'name': topic_name,
            'type': '未知',
            'publisher_count': 0,
            'subscriber_count': 0,
            'publishers': [],
            'subscribers': [],
            'qos_profile': {}
        }
    
    def format_topic_info_json(self, topic_name: str) -> str:
        """将话题信息格式化为JSON字符串
        
        Args:
            topic_name: 话题名称
            
        Returns:
            str: JSON格式的话题信息
        """
        info = self.get_parsed_topic_info(topic_name)
        return json.dumps(info, indent=2, ensure_ascii=False)
    
    def clear_cache(self):
        """清空话题信息缓存"""
        self.topics_cache.clear()
    
    def refresh_topic_cache(self, topic_name: str):
        """刷新指定话题的缓存
        
        Args:
            topic_name: 话题名称
        """
        if topic_name in self.topics_cache:
            del self.topics_cache[topic_name]
        self.get_parsed_topic_info(topic_name)
    
    def topic_exists(self, topic_name: str) -> bool:
        """检查话题是否存在
        
        Args:
            topic_name: 话题名称
            
        Returns:
            bool: 话题是否存在
        """
        topics = self.get_topics()
        return topic_name in topics
    
    def shutdown(self):
        """关闭管理器，清理资源"""
        # 销毁所有订阅
        for topic_name in list(self.subscriptions.keys()):
            self.destroy_subscription(topic_name)
        
        # 关闭rclpy
        if self.rclpy_initialized and rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass


class TopicSubscription:
    """话题订阅封装类"""
    
    def __init__(self, topic_name: str, message_type: str, callback: Callable,
                 qos_depth: int = 10, manager: 'TopicManager' = None):
        self.topic_name = topic_name
        self.message_type = message_type
        self.callback = callback
        self.qos_depth = qos_depth
        self.manager = manager
        self.node = None
        self.subscription = None
        self.thread = None
        self.running = False
        
        self.setup_subscription()
    
    def setup_subscription(self):
        """设置订阅"""
        try:
            if not RCLPY_AVAILABLE:
                raise ImportError("rclpy不可用")
            
            # 创建临时节点进行订阅
            node_name = f'topic_monitor_{abs(hash(self.topic_name)) % 100000}'
            self.node = rclpy.create_node(node_name)
            
            # 动态导入消息类型
            msg_class = self.manager.import_message_type(self.message_type)
            if not msg_class:
                raise ImportError(f"无法导入消息类型: {self.message_type}")
            
            # 创建订阅
            self.subscription = self.node.create_subscription(
                msg_class,
                self.topic_name,
                self.message_callback,
                self.qos_depth
            )
            
            # 在单独线程中旋转节点
            self.running = True
            self.thread = threading.Thread(target=self.spin_node, daemon=True)
            self.thread.start()
            
        except Exception as e:
            print(f"创建订阅失败: {e}")
            raise
    
    def message_callback(self, msg):
        """消息回调函数"""
        try:
            # 将消息转换为字典
            message_dict = self.message_to_dict(msg)
            
            # 调用用户提供的回调
            self.callback({
                'topic': self.topic_name,
                'message': message_dict,
                'timestamp': time.strftime("%H:%M:%S.%f")[:-3]
            })
            
        except Exception as e:
            print(f"处理消息时出错: {e}")
    
    def message_to_dict(self, msg) -> Dict[str, Any]:
        """将消息对象转换为字典
        
        Args:
            msg: ROS2消息对象
            
        Returns:
            Dict: 消息字典
        """
        result = {}
        
        # 获取消息的所有字段
        if hasattr(msg, 'get_fields_and_field_types'):
            fields = msg.get_fields_and_field_types()
            for field_name in fields.keys():
                try:
                    value = getattr(msg, field_name)
                    # 递归处理嵌套消息
                    if hasattr(value, 'get_fields_and_field_types'):
                        result[field_name] = self.message_to_dict(value)
                    elif isinstance(value, (list, tuple)):
                        result[field_name] = [self.message_to_dict(v) if hasattr(v, 'get_fields_and_field_types') else v for v in value]
                    else:
                        result[field_name] = value
                except:
                    result[field_name] = None
        else:
            # 备用方法：使用 __slots__ 或 dir
            if hasattr(msg, '__slots__'):
                for field_name in msg.__slots__:
                    if not field_name.startswith('_'):
                        try:
                            result[field_name] = getattr(msg, field_name)
                        except:
                            pass
            else:
                for field_name in dir(msg):
                    if not field_name.startswith('_') and not callable(getattr(msg, field_name)):
                        try:
                            result[field_name] = getattr(msg, field_name)
                        except:
                            pass
        
        return result
    
    def spin_node(self):
        """旋转节点以接收消息"""
        try:
            while self.running and rclpy.ok() and self.subscription is not None:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"节点旋转出错: {e}")
    
    def destroy(self):
        """销毁订阅"""
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        
        if self.node:
            try:
                self.node.destroy_node()
            except:
                pass
            self.subscription = None
            self.node = None
