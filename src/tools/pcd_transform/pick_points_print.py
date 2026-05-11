import argparse
import numpy as np
import open3d as o3d


def estimate_axis_size(pcd):
    bbox = pcd.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    max_len = np.max(extent)

    if max_len <= 0:
        return 1.0

    return max_len * 0.15


def pick_points(input_path: str):
    pcd = o3d.io.read_point_cloud(input_path)

    if pcd.is_empty():
        raise RuntimeError("读取失败：点云为空，请检查输入文件路径或格式。")

    axis_size = estimate_axis_size(pcd)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=axis_size,
        origin=[0, 0, 0]
    )

    print("操作说明：")
    print("1. Shift + 鼠标左键：选择点")
    print("2. 可以连续选择多个点")
    print("3. Q 或 ESC：退出窗口")
    print("4. 退出后，控制台会输出所有选中点的坐标")
    print()
    print("坐标轴说明：红色=X，绿色=Y，蓝色=Z")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="点云选点工具：Shift + 左键选择点")

    vis.add_geometry(pcd)
    vis.add_geometry(axis)

    vis.run()
    vis.destroy_window()

    picked_indices = vis.get_picked_points()
    points = np.asarray(pcd.points)

    if len(picked_indices) == 0:
        print("没有选择任何点。")
        return

    print()
    print("=" * 60)
    print(f"共选择了 {len(picked_indices)} 个点")
    print("=" * 60)

    for i, index in enumerate(picked_indices, start=1):
        point = points[index]
        x, y, z = point

        print(f"第 {i} 个点")
        print(f"点索引 index: {index}")
        print(f"坐标: x={x:.6f}, y={y:.6f}, z={z:.6f}")
        print(f"numpy格式: [{x:.6f}, {y:.6f}, {z:.6f}]")
        print("-" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="点云选点并输出坐标")
    parser.add_argument("input", help="输入点云文件，例如 input.pcd")

    args = parser.parse_args()

    pick_points(args.input)