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


def translate_origin_by_picked_point(input_path: str, output_path: str):
    pcd = o3d.io.read_point_cloud(input_path)

    if pcd.is_empty():
        raise RuntimeError("读取失败：点云为空，请检查输入文件路径或格式。")

    axis_size = estimate_axis_size(pcd)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=10.0,
        origin=[0, 0, 0]
    )

    print("操作说明：")
    print("1. 窗口中会显示当前点云坐标系")
    print("2. 红色轴 = X，绿色轴 = Y，蓝色轴 = Z")
    print("3. 按住 Shift + 鼠标左键，选择一个点")
    print("4. 选择完成后，按 Q 或 ESC 退出窗口")
    print("5. 程序会把你选中的点移动到坐标原点 (0, 0, 0)")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="选择新的点云原点")

    vis.add_geometry(pcd)
    vis.add_geometry(axis)

    vis.run()
    vis.destroy_window()

    picked_indices = vis.get_picked_points()

    if len(picked_indices) == 0:
        print("没有选择任何点，程序退出。")
        return

    index = picked_indices[0]
    points = np.asarray(pcd.points)
    picked_point = points[index].copy()

    print(f"选中的点索引: {index}")
    print(f"选中的点坐标: {picked_point}")

    points -= picked_point
    pcd.points = o3d.utility.Vector3dVector(points)

    new_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=axis_size,
        origin=[0, 0, 0]
    )

    print("平移后预览：")
    print("你选择的点现在已经被移动到坐标原点。")
    print("关闭窗口后会保存点云。")

    o3d.visualization.draw_geometries(
        [pcd, new_axis],
        window_name="平移后的点云预览，原点坐标系已显示"
    )

    ok = o3d.io.write_point_cloud(output_path, pcd, write_ascii=False)
    if not ok:
        raise RuntimeError("保存失败。")

    print("平移完成")
    print(f"新原点对应原坐标: {picked_point}")
    print(f"输出文件: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="点选一个点，并把该点平移到点云坐标原点")
    parser.add_argument("input", help="输入点云文件，例如 scaled.pcd")
    parser.add_argument("output", help="输出点云文件，例如 translated.pcd")

    args = parser.parse_args()

    translate_origin_by_picked_point(args.input, args.output)