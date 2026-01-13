@echo off
REM ROS2 DDS GUI 构建脚本 - Windows
chcp 65001 >nul

echo ==========================================
echo   ROS2 DDS GUI 构建脚本 (Windows)
echo ==========================================

REM 检查 Python
python --version >nul 2>&1
if errorlevel 1 (
    echo 错误: 未找到 Python
    pause
    exit /b 1
)

REM 安装依赖
echo ^>^> 安装依赖...
pip install -r requirements.txt
pip install pyinstaller

REM 清理旧的构建
echo ^>^> 清理旧的构建文件...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

REM 执行打包
echo ^>^> 开始打包...
pyinstaller ros2_dds_gui.spec --clean

echo.
echo ==========================================
echo   构建完成!
echo   输出目录: dist\ROS2_DDS_GUI\
echo ==========================================

REM 创建压缩包提示
echo.
echo 请手动将 dist\ROS2_DDS_GUI 文件夹压缩为发布包

pause
