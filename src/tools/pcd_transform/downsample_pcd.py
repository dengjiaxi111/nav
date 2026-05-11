import argparse
from pathlib import Path

import open3d as o3d


def format_file_size(size_bytes: int) -> str:
    if size_bytes < 1024:
        return f"{size_bytes} B"
    elif size_bytes < 1024 ** 2:
        return f"{size_bytes / 1024:.2f} KB"
    elif size_bytes < 1024 ** 3:
        return f"{size_bytes / 1024 ** 2:.2f} MB"
    else:
        return f"{size_bytes / 1024 ** 3:.2f} GB"


def downsample_point_cloud(
    input_path: str,
    output_path: str,
    voxel_size: float,
    write_ascii: bool = False
):
    input_file = Path(input_path)
    output_file = Path(output_path)

    if not input_file.exists():
        raise FileNotFoundError(f"输入文件不存在：{input_path}")

    input_size = input_file.stat().st_size

    pcd = o3d.io.read_point_cloud(str(input_file))

    if pcd.is_empty():
        raise RuntimeError("读取失败：点云为空，请检查输入文件路径或格式。")

    before_points = len(pcd.points)

    print("开始降采样...")
    print(f"输入文件: {input_path}")
    print(f"输出文件: {output_path}")
    print(f"体素大小 voxel_size: {voxel_size}")
    print()

    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    after_points = len(down_pcd.points)

    ok = o3d.io.write_point_cloud(
        str(output_file),
        down_pcd,
        write_ascii=write_ascii
    )

    if not ok:
        raise RuntimeError("保存失败。")

    output_size = output_file.stat().st_size

    point_ratio = after_points / before_points if before_points > 0 else 0
    size_ratio = output_size / input_size if input_size > 0 else 0

    print("=" * 60)
    print("降采样完成")
    print("=" * 60)

    print("点数统计：")
    print(f"降采样前点数: {before_points}")
    print(f"降采样后点数: {after_points}")
    print(f"点数保留比例: {point_ratio * 100:.2f}%")
    print(f"点数减少比例: {(1 - point_ratio) * 100:.2f}%")

    print()

    print("文件大小统计：")
    print(f"降采样前文件大小: {format_file_size(input_size)}")
    print(f"降采样后文件大小: {format_file_size(output_size)}")
    print(f"文件大小保留比例: {size_ratio * 100:.2f}%")
    print(f"文件大小减少比例: {(1 - size_ratio) * 100:.2f}%")

    print()

    print(f"已保存到: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="点云体素降采样工具")
    parser.add_argument("input", help="输入点云文件，例如 input.pcd")
    parser.add_argument("output", help="输出点云文件，例如 downsampled.pcd")
    parser.add_argument(
        "--voxel",
        type=float,
        required=True,
        help="体素大小，例如 0.01、0.05、0.1"
    )
    parser.add_argument(
        "--ascii",
        action="store_true",
        help="使用 ASCII 格式保存，默认使用二进制保存"
    )

    args = parser.parse_args()

    downsample_point_cloud(
        input_path=args.input,
        output_path=args.output,
        voxel_size=args.voxel,
        write_ascii=args.ascii
    )