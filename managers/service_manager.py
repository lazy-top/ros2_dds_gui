import subprocess
import json
import yaml
import tempfile
import os
from typing import List, Dict, Optional, Any


class ServiceManager:
    """ROS2服务管理器 - 管理服务发现、调用和信息查询"""
    
    def __init__(self):
        self.services_cache = {}
        
    def get_services(self) -> List[str]:
        """获取所有可用的ROS2服务
        
        Returns:
            List[str]: 服务名称列表
        """
        try:
            result = subprocess.run(['ros2', 'service', 'list'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return [s.strip() for s in result.stdout.split('\n') if s.strip()]
        except Exception as e:
            print(f"获取服务列表失败: {e}")
        return []
    
    def get_service_type(self, service_name: str) -> Optional[str]:
        """获取服务的类型
        
        Args:
            service_name: 服务名称
            
        Returns:
            Optional[str]: 服务类型，如 'std_srvs/srv/SetBool'
        """
        try:
            result = subprocess.run(['ros2', 'service', 'type', service_name],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception as e:
            print(f"获取服务 {service_name} 类型失败: {e}")
        return None
    
    def get_service_info(self, service_name: str) -> Dict:
        """获取服务的详细信息
        
        Args:
            service_name: 服务名称
            
        Returns:
            Dict: 服务详细信息
        """
        info = {
            'name': service_name,
            'type': None,
            'definition': None,
            'request_fields': [],
            'response_fields': []
        }
        
        try:
            # 获取服务类型
            service_type = self.get_service_type(service_name)
            if service_type:
                info['type'] = service_type
                
                # 获取服务定义
                definition = self.get_service_definition(service_type)
                if definition:
                    info['definition'] = definition
                    # 解析请求和响应字段
                    parsed = self.parse_service_definition(definition)
                    info['request_fields'] = parsed['request_fields']
                    info['response_fields'] = parsed['response_fields']
                
                # 缓存服务信息
                self.services_cache[service_name] = info
        except Exception as e:
            print(f"获取服务 {service_name} 详细信息失败: {e}")
        
        return info
    
    def get_service_definition(self, service_type: str) -> Optional[str]:
        """获取服务的接口定义
        
        Args:
            service_type: 服务类型
            
        Returns:
            Optional[str]: 服务接口定义文本
        """
        try:
            result = subprocess.run(['ros2', 'interface', 'show', service_type],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout
        except Exception as e:
            print(f"获取服务类型 {service_type} 定义失败: {e}")
        return None
    
    def parse_service_definition(self, definition: str) -> Dict:
        """解析服务定义，提取请求和响应字段
        
        Args:
            definition: 服务定义文本
            
        Returns:
            Dict: 包含 request_fields 和 response_fields 的字典
        """
        result = {
            'request_fields': [],
            'response_fields': []
        }
        
        if not definition:
            return result
        
        lines = definition.split('\n')
        current_section = 'request'
        
        for line in lines:
            line = line.strip()
            
            # 分隔符表示响应部分开始
            if line.startswith('---'):
                current_section = 'response'
                continue
            
            # 跳过空行和注释
            if not line or line.startswith('#'):
                continue
            
            # 解析字段定义（格式：type field_name）
            parts = line.split()
            if len(parts) >= 2:
                field_info = {
                    'type': parts[0],
                    'name': parts[1],
                    'default': parts[2] if len(parts) > 2 else None
                }
                
                if current_section == 'request':
                    result['request_fields'].append(field_info)
                else:
                    result['response_fields'].append(field_info)
        
        return result
    
    def get_all_services_info(self) -> Dict[str, Dict]:
        """获取所有服务的信息
        
        Returns:
            Dict[str, Dict]: 服务名称到服务信息的映射
        """
        services = self.get_services()
        all_info = {}
        
        for service_name in services:
            info = self.get_service_info(service_name)
            all_info[service_name] = info
        
        return all_info
    
    def call_service(self, service_name: str, request_data: Any, 
                    format_type: str = 'yaml') -> Dict:
        """调用ROS2服务
        
        Args:
            service_name: 服务名称
            request_data: 请求数据（dict或str）
            format_type: 请求数据格式（'yaml' 或 'json'）
            
        Returns:
            Dict: 包含成功标志、响应数据和错误信息的字典
        """
        result = {
            'success': False,
            'response': None,
            'error': None,
            'stdout': '',
            'stderr': ''
        }
        
        try:
            # 获取服务类型
            service_type = self.get_service_type(service_name)
            if not service_type:
                result['error'] = "无法获取服务类型"
                return result
            
            # 准备请求数据
            if isinstance(request_data, dict):
                # 将字典转换为YAML格式字符串
                request_str = yaml.dump(request_data)
            else:
                request_str = str(request_data)
            
            # 使用临时文件存储请求数据
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', 
                                           delete=False) as f:
                f.write(request_str)
                temp_file = f.name
            
            try:
                # 调用 ros2 service call 命令
                # 格式: ros2 service call <service_name> <service_type> <request_yaml>
                cmd = [
                    'ros2', 'service', 'call',
                    service_name,
                    service_type,
                    request_str
                ]
                
                process_result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=10.0
                )
                
                result['stdout'] = process_result.stdout
                result['stderr'] = process_result.stderr
                
                if process_result.returncode == 0:
                    result['success'] = True
                    result['response'] = self.parse_service_response(process_result.stdout)
                else:
                    result['error'] = f"服务调用返回错误: {process_result.stderr}"
                    
            finally:
                # 清理临时文件
                if os.path.exists(temp_file):
                    try:
                        os.unlink(temp_file)
                    except:
                        pass
                        
        except subprocess.TimeoutExpired:
            result['error'] = "服务调用超时（10秒）"
        except Exception as e:
            result['error'] = f"服务调用异常: {str(e)}"
        
        return result
    
    def parse_service_response(self, response_text: str) -> Dict:
        """解析服务响应文本
        
        Args:
            response_text: 响应文本
            
        Returns:
            Dict: 解析后的响应数据
        """
        try:
            # 尝试从响应文本中提取YAML部分
            # ros2 service call的输出格式通常包含一些元信息
            lines = response_text.split('\n')
            yaml_lines = []
            capture = False
            
            for line in lines:
                if 'response:' in line.lower():
                    capture = True
                    continue
                if capture:
                    yaml_lines.append(line)
            
            if yaml_lines:
                yaml_text = '\n'.join(yaml_lines)
                return yaml.safe_load(yaml_text) or {}
            
            # 如果没有找到response部分，返回原始文本
            return {'raw_response': response_text}
            
        except Exception as e:
            print(f"解析服务响应失败: {e}")
            return {'raw_response': response_text}
    
    def find_service_by_type(self, service_type: str) -> List[str]:
        """根据服务类型查找服务
        
        Args:
            service_type: 服务类型
            
        Returns:
            List[str]: 匹配的服务名称列表
        """
        services = self.get_services()
        matching_services = []
        
        for service_name in services:
            svc_type = self.get_service_type(service_name)
            if svc_type == service_type:
                matching_services.append(service_name)
        
        return matching_services
    
    def service_exists(self, service_name: str) -> bool:
        """检查服务是否存在
        
        Args:
            service_name: 服务名称
            
        Returns:
            bool: 服务是否存在
        """
        services = self.get_services()
        return service_name in services
    
    def validate_request_data(self, service_name: str, request_data: Dict) -> Dict:
        """验证请求数据是否符合服务定义
        
        Args:
            service_name: 服务名称
            request_data: 请求数据
            
        Returns:
            Dict: 包含验证结果和错误信息的字典
        """
        result = {
            'valid': True,
            'errors': [],
            'warnings': []
        }
        
        try:
            # 获取服务信息
            service_info = self.get_service_info(service_name)
            request_fields = service_info.get('request_fields', [])
            
            if not request_fields:
                result['warnings'].append("无法获取服务请求字段定义")
                return result
            
            # 检查必需字段
            for field in request_fields:
                field_name = field['name']
                if field_name not in request_data:
                    result['warnings'].append(f"缺少字段: {field_name} ({field['type']})")
            
            # 检查额外字段
            expected_fields = {f['name'] for f in request_fields}
            for key in request_data.keys():
                if key not in expected_fields:
                    result['warnings'].append(f"未知字段: {key}")
                    
        except Exception as e:
            result['valid'] = False
            result['errors'].append(f"验证失败: {str(e)}")
        
        return result
    
    def format_service_info(self, service_name: str, format_type: str = 'text') -> str:
        """格式化服务信息为指定格式
        
        Args:
            service_name: 服务名称
            format_type: 格式类型 ('text', 'json', 'yaml')
            
        Returns:
            str: 格式化后的服务信息
        """
        info = self.get_service_info(service_name)
        
        if format_type == 'json':
            return json.dumps(info, indent=2, ensure_ascii=False)
        elif format_type == 'yaml':
            return yaml.dump(info, default_flow_style=False, allow_unicode=True)
        else:  # text
            text = f"服务名称: {info['name']}\n"
            text += "=" * 50 + "\n"
            text += f"服务类型: {info.get('type', '未知')}\n\n"
            
            if info.get('definition'):
                text += "服务定义:\n"
                text += info['definition'] + "\n"
            
            return text
    
    def get_service_statistics(self) -> Dict:
        """获取服务统计信息
        
        Returns:
            Dict: 统计信息
        """
        services = self.get_services()
        stats = {
            'total_services': len(services),
            'service_types': {},
            'services_by_type': {}
        }
        
        for service_name in services:
            service_type = self.get_service_type(service_name)
            if service_type:
                if service_type not in stats['service_types']:
                    stats['service_types'][service_type] = 0
                    stats['services_by_type'][service_type] = []
                
                stats['service_types'][service_type] += 1
                stats['services_by_type'][service_type].append(service_name)
        
        return stats
    
    def clear_cache(self):
        """清空服务信息缓存"""
        self.services_cache.clear()
    
    def refresh_cache(self, service_name: str = None):
        """刷新缓存
        
        Args:
            service_name: 要刷新的服务名称，如果为None则刷新所有
        """
        if service_name:
            if service_name in self.services_cache:
                del self.services_cache[service_name]
            self.get_service_info(service_name)
        else:
            self.clear_cache()
            self.get_all_services_info()
