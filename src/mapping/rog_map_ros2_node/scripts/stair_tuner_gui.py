#!/usr/bin/env python3
"""
台阶检测参数调整 GUI
使用滑块实时调整 StairDetector 参数
"""

import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QLabel, QSlider, QCheckBox, QGroupBox,
                                 QPushButton, QSpinBox, QDoubleSpinBox, QTabWidget)
    from PyQt5.QtCore import Qt, QTimer
except ImportError:
    print("Error: PyQt5 not installed. Install with:")
    print("  pip3 install PyQt5")
    sys.exit(1)


class StairTunerGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("台阶检测参数调整器")
        self.setGeometry(100, 100, 800, 900)
        
        # 从节点获取当前参数值
        self.current_params = self.load_current_parameters()
        
        # 创建主窗口
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # 创建标签页
        tabs = QTabWidget()
        main_layout.addWidget(tabs)
        
        # === Tab 1: 预筛选参数 ===
        prefilter_tab = QWidget()
        prefilter_layout = QVBoxLayout(prefilter_tab)
        
        prefilter_layout.addWidget(QLabel("<h2>🔍 预筛选参数</h2>"))
        
        self.add_double_slider(prefilter_layout, "网格大小 (m)", "cell_size_xy", 0.01, 0.20, 0.05, 100)
        self.add_int_slider(prefilter_layout, "网格最小点数", "min_cell_points", 1, 10, 3)
        self.add_double_slider(prefilter_layout, "网格最小高度 (m)", "min_cell_height", 0.00, 0.10, 0.03, 100)
        self.add_double_slider(prefilter_layout, "网格最大高度 (m)", "max_cell_height", 0.10, 1.00, 0.21, 100)  # 改为 0.21
        self.add_double_slider(prefilter_layout, "网格顶部最大Z (m)", "max_cell_top_z", 0.00, 2.00, 0.50, 100)
        
        prefilter_layout.addStretch()
        tabs.addTab(prefilter_tab, "预筛选")
        
        # === Tab 2: 几何约束 ===
        geometry_tab = QWidget()
        geometry_layout = QVBoxLayout(geometry_tab)
        
        geometry_layout.addWidget(QLabel("<h2>📏 几何约束参数</h2>"))
        
        self.add_double_slider(geometry_layout, "台阶标准高度 (m)", "single_stair_height", 0.10, 0.40, 0.20, 100)
        self.add_double_slider(geometry_layout, "高度容差 (m)", "height_tolerance", 0.02, 0.20, 0.10, 100)
        self.add_double_slider(geometry_layout, "最小宽度 (m)", "min_stair_width", 0.10, 0.50, 0.20, 100)
        self.add_double_slider(geometry_layout, "最小深度 (m)", "min_stair_depth", 0.01, 0.20, 0.02, 100)
        self.add_double_slider(geometry_layout, "最大厚度 (m)", "max_z_thickness", 0.10, 0.80, 0.40, 100)
        
        geometry_layout.addStretch()
        tabs.addTab(geometry_tab, "几何约束")
        
        # === Tab 3: 聚类参数 ===
        cluster_tab = QWidget()
        cluster_layout = QVBoxLayout(cluster_tab)
        
        cluster_layout.addWidget(QLabel("<h2>🔗 聚类参数</h2>"))
        
        self.add_double_slider(cluster_layout, "聚类搜索半径 (m)", "cluster_tolerance", 0.02, 0.20, 0.08, 100)
        self.add_int_slider(cluster_layout, "最小聚类点数", "min_cluster_size", 20, 500, 100)
        self.add_int_slider(cluster_layout, "最大聚类点数", "max_cluster_size", 1000, 30000, 15000)
        
        cluster_layout.addStretch()
        tabs.addTab(cluster_tab, "聚类")
        
        # === Tab 4: 法向量验证 ===
        normal_tab = QWidget()
        normal_layout = QVBoxLayout(normal_tab)
        
        normal_layout.addWidget(QLabel("<h2>📐 法向量验证参数</h2>"))
        
        self.add_checkbox(normal_layout, "启用法向量验证", "enable_normal_estimation", True)
        self.add_double_slider(normal_layout, "平面性阈值", "min_planarity", 0.30, 1.00, 0.60, 100)
        self.add_double_slider(normal_layout, "水平法向量 Z 分量下限", "horizontal_normal_z_min", 0.50, 1.00, 0.75, 100)
        self.add_double_slider(normal_layout, "水平点占比下限", "horizontal_points_ratio_min", 0.30, 1.00, 0.60, 100)
        self.add_int_slider(normal_layout, "法向量计算最小点数", "normal_min_points", 10, 200, 30)
        
        normal_layout.addStretch()
        tabs.addTab(normal_tab, "法向量")
        
        # === Tab 5: ROI 范围 ===
        roi_tab = QWidget()
        roi_layout = QVBoxLayout(roi_tab)
        
        roi_layout.addWidget(QLabel("<h2>📦 ROI 范围参数</h2>"))
        
        self.add_double_slider(roi_layout, "前方最小距离 (m)", "roi_x_min", -5.0, 5.0, 0.2, 100)
        self.add_double_slider(roi_layout, "前方最大距离 (m)", "roi_x_max", 0.0, 10.0, 3.0, 100)
        self.add_double_slider(roi_layout, "左侧范围 (m)", "roi_y_min", -5.0, 0.0, -2.0, 100)
        self.add_double_slider(roi_layout, "右侧范围 (m)", "roi_y_max", 0.0, 5.0, 2.0, 100)
        self.add_double_slider(roi_layout, "Z 最小值 (m)", "roi_z_min", -1.0, 0.5, -0.5, 100)
        self.add_double_slider(roi_layout, "Z 最大值 (m)", "roi_z_max", 0.0, 2.0, 0.5, 100)
        
        roi_layout.addStretch()
        tabs.addTab(roi_tab, "ROI 范围")
        
        # === 底部按钮 ===
        button_layout = QHBoxLayout()
        
        reset_btn = QPushButton("🔄 重置所有参数")
        reset_btn.clicked.connect(self.reset_all_parameters)
        button_layout.addWidget(reset_btn)
        
        info_label = QLabel("提示: 拖动滑块即时生效，无需重启节点")
        info_label.setStyleSheet("color: green; font-weight: bold;")
        button_layout.addWidget(info_label)
        
        main_layout.addLayout(button_layout)
        
        # ROS 定时器（用于处理回调）
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(100)  # 100ms
    
    def load_current_parameters(self):
        """从节点加载当前参数值"""
        param_names = [
            'cell_size_xy', 'min_cell_points', 'min_cell_height', 'max_cell_height', 'max_cell_top_z',
            'single_stair_height', 'double_stair_height', 'height_tolerance', 'min_stair_width', 'min_stair_depth',
            'cluster_tolerance', 'min_cluster_size', 'max_cluster_size',
            'min_planarity', 'horizontal_normal_z_min', 'horizontal_points_ratio_min', 'normal_min_points',
            'roi_x_min', 'roi_x_max', 'roi_y_min', 'roi_y_max', 'roi_z_min', 'roi_z_max'
        ]
        
        print("正在从节点读取参数...")
        future = self.node.call_get_parameters(param_names)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if not future.done():
            print("警告: 无法读取参数，使用默认值")
            return {}
        
        response = future.result()
        params = {}
        for name, value in zip(param_names, response.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                params[name] = value.double_value
            elif value.type == ParameterType.PARAMETER_INTEGER:
                params[name] = value.integer_value
            print(f"  {name} = {params.get(name, 'N/A')}")
        
        return params
    
    def get_param_value(self, param_name, default_val):
        """获取参数值，如果读取失败则使用默认值"""
        return self.current_params.get(param_name, default_val)
    
    def add_double_slider(self, layout, label, param_name, min_val, max_val, default_val, scale=10):
        """添加浮点数滑块"""
        # 使用从节点读取的值（如果有）
        current_val = self.get_param_value(param_name, default_val)
        
        group = QGroupBox(label)
        group_layout = QHBoxLayout()
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(min_val * scale))
        slider.setMaximum(int(max_val * scale))
        slider.setValue(int(current_val * scale))
        
        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(min_val)
        spinbox.setMaximum(max_val)
        spinbox.setValue(default_val)
        spinbox.setDecimals(3)
        spinbox.setSingleStep((max_val - min_val) / 100)
        
        # 同步滑块和数字框
        slider.valueChanged.connect(lambda v: spinbox.setValue(v / scale))
        spinbox.valueChanged.connect(lambda v: slider.setValue(int(v * scale)))
        
        # 参数变化时发送到 ROS
        spinbox.valueChanged.connect(lambda v: self.set_parameter(param_name, v))
        
        group_layout.addWidget(slider)
        group_layout.addWidget(spinbox)
        group.setLayout(group_layout)
        layout.addWidget(group)
    
    def add_int_slider(self, layout, label, param_name, min_val, max_val, default_val):
        """添加整数滑块"""
        # 使用从节点读取的值（如果有）
        current_val = int(self.get_param_value(param_name, default_val))
        
        group = QGroupBox(label)
        group_layout = QHBoxLayout()
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue(current_val)
        
        spinbox = QSpinBox()
        spinbox.setMinimum(min_val)
        spinbox.setMaximum(max_val)
        spinbox.setValue(current_val)
        
        # 同步
        slider.valueChanged.connect(spinbox.setValue)
        spinbox.valueChanged.connect(slider.setValue)
        
        # 发送到 ROS
        spinbox.valueChanged.connect(lambda v: self.set_parameter(param_name, v))
        
        group_layout.addWidget(slider)
        group_layout.addWidget(spinbox)
        group.setLayout(group_layout)
        layout.addWidget(group)
    
    def add_checkbox(self, layout, label, param_name, default_val):
        """添加复选框"""
        checkbox = QCheckBox(label)
        checkbox.setChecked(default_val)
        checkbox.stateChanged.connect(lambda state: self.set_parameter(param_name, state == Qt.Checked))
        layout.addWidget(checkbox)
    
    def set_parameter(self, name, value):
        """设置 ROS 参数"""
        try:
            # 创建参数
            param = Parameter()
            param.name = name
            
            if isinstance(value, bool):
                param.value.type = ParameterType.PARAMETER_BOOL
                param.value.bool_value = value
            elif isinstance(value, int):
                param.value.type = ParameterType.PARAMETER_INTEGER
                param.value.integer_value = value
            elif isinstance(value, float):
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value
            
            # 调用服务设置参数
            self.node.call_set_parameters([param])
            
        except Exception as e:
            print(f"Failed to set parameter {name}: {e}")
    
    def reset_all_parameters(self):
        """重置所有参数到默认值"""
        print("Resetting all parameters...")
        # TODO: 从配置文件重新加载
    
    def spin_ros(self):
        """处理 ROS 回调"""
        rclpy.spin_once(self.node, timeout_sec=0)


class ParameterClient(Node):
    def __init__(self):
        super().__init__('stair_tuner_client')
        # 修改：连接到 rog_map_integration 容器节点
        self.set_cli = self.create_client(SetParameters, '/rog_map_integration/set_parameters')
        self.get_cli = self.create_client(GetParameters, '/rog_map_integration/get_parameters')
        
        # 等待服务可用
        while not self.set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /rog_map_integration service...')
        
        self.get_logger().info('Connected to /rog_map_integration')
    
    def call_set_parameters(self, parameters):
        req = SetParameters.Request()
        req.parameters = parameters
        future = self.set_cli.call_async(req)
        return future
    
    def call_get_parameters(self, param_names):
        """获取参数当前值"""
        req = GetParameters.Request()
        req.names = param_names
        future = self.get_cli.call_async(req)
        return future


def main():
    rclpy.init()
    
    node = ParameterClient()
    
    app = QApplication(sys.argv)
    window = StairTunerGUI(node)
    window.show()
    
    exit_code = app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
