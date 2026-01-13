# ROS2 DDS GUI

**轻量级 ROS2 图形化调试工具** - 一站式监控节点、话题、服务和计算图。

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)
![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

## 核心功能
![主界面](images/main_window.png)

| 模块 | 功能 |
|------|------|
| **节点浏览器** | 查看、过滤、终止 ROS2 节点 |
| **话题监控** | 实时订阅话题，支持 JSON/YAML 格式显示 |
| **服务调用** | 发现服务并发送请求 |
| **拓扑视图** | 可视化计算图，多种布局算法 |
| **DDS 配置** | 域 ID 管理，RMW 切换 |

## 系统要求

- **操作系统**: Ubuntu 20.04 / 22.04 (推荐)
- **Python**: 3.8+
- **ROS2**: Humble Hawksbill (或更高版本)
- **Qt**: PyQt5 5.15+

## 安装

### 1. 安装依赖

```bash
# 确保已安装 ROS2 Humble
source /opt/ros/humble/setup.bash

# 安装 Python 依赖
pip3 install PyQt5 networkx pyyaml
```

### 2. 克隆仓库

```bash
git clone https://github.com/yourusername/ros2_dds_gui.git
cd ros2_dds_gui
```

### 3. 运行应用

```bash
# 确保 ROS2 环境已加载
source /opt/ros/humble/setup.bash

# 启动应用
python3 main.py
```

## 项目结构

```
ros2_dds_gui/
├── main.py                 # 应用程序入口
├── main_window.py          # 主窗口实现
├── managers/               # 后端管理器模块
│   ├── config_manager.py   # 配置管理
│   ├── dds_manager.py      # DDS 服务管理
│   ├── graph_manager.py    # 计算图管理
│   ├── node_manager.py     # 节点管理
│   ├── ros2_manager.py     # ROS2 环境管理
│   ├── service_manager.py  # 服务管理
│   └── topic_manager.py    # 话题管理
├── widgets/                # 前端 UI 组件
│   ├── dds_selector.py     # DDS 选择器
│   ├── graph_view.py       # 拓扑图视图
│   ├── node_explorer.py    # 节点浏览器
│   ├── service_caller.py   # 服务调用器
│   ├── status_panel.py     # 状态面板
│   └── topic_monitor.py    # 话题监控器
├── LICENSE                 # Apache 2.0 许可证
└── README.md               # 项目说明
```

## 架构设计

本项目采用前后端分离架构：

- **Managers (后端)**: 封装所有 ROS2 CLI 命令调用和数据处理逻辑
- **Widgets (前端)**: 负责用户界面展示和交互

这种设计使得代码更易于维护、测试和扩展。

## 应用展示

### 主界面
![主界面](images/main_window.png)

<!-- ### 节点浏览器
查看和管理 ROS2 节点，支持实时刷新和详细信息展示。

![节点浏览器](images/node_explorer.png) -->

### 话题监控
订阅话题并实时查看消息内容，支持多种格式化显示。

![话题监控](images/topic_monitor.png)

### 服务调用
发现系统中的服务并发送请求。

![服务调用](images/service_caller.png)

### 拓扑视图
直观展示 ROS2 系统的计算图结构。

![拓扑视图](images/graph_view.png)

## 开发指南

### 添加新功能

1. 在 `managers/` 目录下创建对应的管理器类
2. 在 `widgets/` 目录下创建 UI 组件
3. 在 `main_window.py` 中集成新组件

### 代码规范

- 使用 Python 类型注解
- 遵循 PEP 8 编码规范
- 为公共方法添加文档字符串

## 常见问题

### Q: 运行时提示 GLIBCXX_3.4.30 not found

这是由于 Anaconda 环境与 ROS2 系统库冲突。解决方案：

```bash
# 方案1: 使用系统 Python
conda deactivate
python3 main.py

# 方案2: 更新 conda 库
conda install -c conda-forge libstdcxx-ng
```

### Q: 无法获取节点/话题列表

请确保：
1. ROS2 环境已正确加载 (`source /opt/ros/humble/setup.bash`)
2. ROS2 daemon 正在运行 (`ros2 daemon status`)
3. 有其他 ROS2 节点在运行

## 贡献指南

欢迎提交 Issue 和 Pull Request！

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 许可证

本项目基于 [Apache License 2.0](LICENSE) 开源。

您可以自由地：
- 使用、复制、修改和分发本软件
- 将本软件用于商业用途

但需要：
- 保留版权声明和许可证
- 标注对原始代码的修改
- 在衍生作品中包含许可证副本

## 致谢

- [ROS2](https://ros.org/) - 机器人操作系统
- [PyQt5](https://www.riverbankcomputing.com/software/pyqt/) - Python Qt 绑定
- [NetworkX](https://networkx.org/) - Python 图论库

## 联系方式

如有问题或建议，请通过以下方式联系：

- 提交 [GitHub Issue](https://github.com/yourusername/ros2_dds_gui/issues)
- 发送邮件至: your.email@example.com

---

**如果这个项目对您有帮助，请给个 Star 支持一下！**
