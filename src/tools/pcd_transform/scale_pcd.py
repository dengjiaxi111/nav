import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def scale_point_cloud(input_path: str, output_path: str, scale: float):
    pcd = o3d.io.read_point_cloud(input_path)

    if pcd.is_empty():
        raise RuntimeError("读取失败：点云为空，请检查输入文件路径或格式。")

    points = np.asarray(pcd.points)
    points *= scale
    pcd.points = o3d.utility.Vector3dVector(points)

    ok = o3d.io.write_point_cloud(output_path, pcd, write_ascii=False)
    if not ok:
        raise RuntimeError("保存失败。")

    print("缩放完成")
    print(f"输入文件: {input_path}")
    print(f"缩放倍率: {scale}")
    print(f"输出文件: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="点云缩放工具")
    parser.add_argument("input", help="输入点云文件，例如 input.pcd")
    parser.add_argument("output", help="输出点云文件，例如 scaled.pcd")
    parser.add_argument("--scale", type=float, required=True, help="缩放倍率，例如 0.01 或 100")

    args = parser.parse_args()

    scale_point_cloud(args.input, args.output, args.scale)