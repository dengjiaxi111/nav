import sys
import rclpy
from PySide6.QtWidgets import QApplication
from .node import MyRefereeNode
from .gui import MyRefereeWindow

def main():
    rclpy.init()
    node = MyRefereeNode()

    app = QApplication(sys.argv)
    window = MyRefereeWindow(node)
    window.show()

    # ROS2 和 GUI 事件循环结合
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        app.processEvents()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
