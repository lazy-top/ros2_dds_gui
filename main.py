#!/usr/bin/env python3
import sys
import os
from PyQt5.QtWidgets import QApplication
from main_window import MainWindow
def main():
    # 创建Qt应用
    app = QApplication(sys.argv)
    app.setApplicationName("ROS2 DDS GUI")
    app.setApplicationVersion("1.0.0")
    # 创建主窗口
    main_window = MainWindow()
    main_window.show()
    
    # 启动事件循环
    try:
        return app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        main_window.shutdown()

if __name__ == '__main__':
    main()