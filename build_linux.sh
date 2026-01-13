#!/bin/bash
# ROS2 DDS GUI 构建脚本 - Linux

set -e

echo "=========================================="
echo "  ROS2 DDS GUI 构建脚本 (Linux)"
echo "=========================================="

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到 python3"
    exit 1
fi

# 创建虚拟环境（可选）
if [ "$1" == "--venv" ]; then
    echo ">> 创建虚拟环境..."
    python3 -m venv build_env
    source build_env/bin/activate
fi

# 安装依赖
echo ">> 安装依赖..."
pip3 install -r requirements.txt
pip3 install pyinstaller

# 清理旧的构建
echo ">> 清理旧的构建文件..."
rm -rf build/ dist/

# 执行打包（使用 python3 -m 调用）
echo ">> 开始打包..."
python3 -m PyInstaller ros2_dds_gui.spec --clean

# 打包完成
echo ""
echo "=========================================="
echo "  构建完成!"
echo "  输出目录: dist/ROS2_DDS_GUI/"
echo "=========================================="

# 创建压缩包
if command -v zip &> /dev/null; then
    echo ">> 创建发布包..."
    cd dist
    zip -r ROS2_DDS_GUI_Linux_x64.zip ROS2_DDS_GUI/
    echo "  发布包: dist/ROS2_DDS_GUI_Linux_x64.zip"
fi
