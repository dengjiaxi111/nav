import argparse
import numpy as np
import open3d as o3d
from open3d.visualization import gui, rendering


class RotatePointCloudApp:
    def __init__(self, input_path: str, output_path: str):
        self.input_path = input_path
        self.output_path = output_path

        self.pcd = o3d.io.read_point_cloud(input_path)
        if self.pcd.is_empty():
            raise RuntimeError("读取失败：点云为空，请检查输入文件路径或格式。")

        self.axis_size = self.estimate_axis_size(self.pcd)

        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=self.axis_size,
            origin=[0, 0, 0]
        )

        self.app = gui.Application.instance
        self.window = self.app.create_window("点云旋转工具", 1200, 800)

        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)

        self.pcd_material = rendering.MaterialRecord()
        self.pcd_material.shader = "defaultUnlit"
        self.pcd_material.point_size = 2.0

        self.axis_material = rendering.MaterialRecord()
        self.axis_material.shader = "defaultLit"

        self.panel = gui.Vert(10, gui.Margins(10, 10, 10, 10))

        title = gui.Label("点云旋转控制")
        self.panel.add_child(title)

        axis_info = gui.Label("坐标轴：红=X，绿=Y，蓝=Z")
        self.panel.add_child(axis_info)

        self.x_input = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.x_input.double_value = 0.0
        self.panel.add_child(gui.Label("绕 X 轴旋转角度，单位：度"))
        self.panel.add_child(self.x_input)

        self.y_input = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.y_input.double_value = 0.0
        self.panel.add_child(gui.Label("绕 Y 轴旋转角度，单位：度"))
        self.panel.add_child(self.y_input)

        self.z_input = gui.NumberEdit(gui.NumberEdit.DOUBLE)
        self.z_input.double_value = 0.0
        self.panel.add_child(gui.Label("绕 Z 轴旋转角度，单位：度"))
        self.panel.add_child(self.z_input)

        self.apply_button = gui.Button("应用旋转")
        self.apply_button.set_on_clicked(self.apply_rotation)
        self.panel.add_child(self.apply_button)

        self.reset_view_button = gui.Button("重置视角")
        self.reset_view_button.set_on_clicked(self.reset_camera)
        self.panel.add_child(self.reset_view_button)

        self.save_button = gui.Button("保存点云")
        self.save_button.set_on_clicked(self.save_point_cloud)
        self.panel.add_child(self.save_button)

        self.info_label = gui.Label("提示：点云会绕当前原点坐标系旋转")
        self.panel.add_child(self.info_label)

        self.window.add_child(self.scene_widget)
        self.window.add_child(self.panel)

        self.window.set_on_layout(self.on_layout)

        self.refresh_scene(reset_camera=True)

    def estimate_axis_size(self, pcd):
        bbox = pcd.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        max_len = np.max(extent)

        if max_len <= 0:
            return 1.0

        return max_len * 0.15

    def on_layout(self, layout_context):
        content_rect = self.window.content_rect

        panel_width = 280

        self.panel.frame = gui.Rect(
            content_rect.x,
            content_rect.y,
            panel_width,
            content_rect.height,
        )

        self.scene_widget.frame = gui.Rect(
            content_rect.x + panel_width,
            content_rect.y,
            content_rect.width - panel_width,
            content_rect.height,
        )

    def refresh_scene(self, reset_camera=False):
        self.scene_widget.scene.clear_geometry()

        self.scene_widget.scene.add_geometry(
            "point_cloud",
            self.pcd,
            self.pcd_material
        )

        self.scene_widget.scene.add_geometry(
            "origin_axis",
            self.axis,
            self.axis_material
        )

        if reset_camera:
            bounds = self.pcd.get_axis_aligned_bounding_box()
            axis_bounds = self.axis.get_axis_aligned_bounding_box()
            bounds += axis_bounds
            center = bounds.get_center()

            self.scene_widget.setup_camera(60.0, bounds, center)

        self.scene_widget.force_redraw()

    def reset_camera(self):
        self.refresh_scene(reset_camera=True)
        self.info_label.text = "已重置视角"

    def apply_rotation(self):
        rx = self.x_input.double_value
        ry = self.y_input.double_value
        rz = self.z_input.double_value

        rx_rad = np.deg2rad(rx)
        ry_rad = np.deg2rad(ry)
        rz_rad = np.deg2rad(rz)

        R = self.pcd.get_rotation_matrix_from_xyz(
            (rx_rad, ry_rad, rz_rad)
        )

        # 重点：点云绕原点坐标系旋转
        self.pcd.rotate(R, center=(0, 0, 0))

        # 坐标轴本身不旋转，始终表示世界坐标系/当前原点坐标系
        self.refresh_scene(reset_camera=False)

        self.info_label.text = f"已应用旋转：X={rx}°, Y={ry}°, Z={rz}°"

        # 清零，下一次输入的是新的增量角度
        self.x_input.double_value = 0.0
        self.y_input.double_value = 0.0
        self.z_input.double_value = 0.0

    def save_point_cloud(self):
        ok = o3d.io.write_point_cloud(
            self.output_path,
            self.pcd,
            write_ascii=False
        )

        if ok:
            self.info_label.text = f"保存成功：{self.output_path}"
            print(f"保存成功：{self.output_path}")
        else:
            self.info_label.text = "保存失败"
            print("保存失败")


def main():
    parser = argparse.ArgumentParser(description="点云旋转可视化工具")
    parser.add_argument("input", help="输入点云文件，例如 translated.pcd")
    parser.add_argument("output", help="输出点云文件，例如 final.pcd")

    args = parser.parse_args()

    gui.Application.instance.initialize()
    RotatePointCloudApp(args.input, args.output)
    gui.Application.instance.run()


if __name__ == "__main__":
    main()