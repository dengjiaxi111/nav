import open3d as o3d
import numpy as np
import copy

def crop_xy_skin(pcd_path, output_path, crop_ratio=0.02, crop_distance=None):
    """
    删除点云 XY 方向最外围的一层“皮”。
    
    参数:
        pcd_path (str): 输入路径
        output_path (str): 输出路径
        crop_ratio (float): 裁剪比例 (0.02 表示每一侧切掉 2% 的长度)。
                            如果设置了 crop_distance，则忽略此参数。
        crop_distance (float): 裁剪的具体距离（单位与点云单位一致，如米或毫米）。
                               例如 0.5 表示每一侧向内缩 0.5 单位。
                               如果为 None，则使用 crop_ratio。
    """
    print(f"[-] 正在读取点云: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    if pcd.is_empty():
        print("[!] 点云为空！")
        return

    # 1. 转换为 numpy 数组
    points = np.asarray(pcd.points)
    original_count = points.shape[0]

    # 2. 计算原始边界
    x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
    y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
    
    width_x = x_max - x_min
    width_y = y_max - y_min

    print(f"[-] 原始 X 范围: {width_x:.4f} (Min: {x_min:.4f}, Max: {x_max:.4f})")
    print(f"[-] 原始 Y 范围: {width_y:.4f} (Min: {y_min:.4f}, Max: {y_max:.4f})")

    # 3. 计算裁剪阈值
    if crop_distance is not None:
        # 按绝对距离裁剪
        delta_x = crop_distance
        delta_y = crop_distance
        print(f"[-] 模式: 绝对距离裁剪 (每侧切除 {crop_distance} 单位)")
    else:
        # 按比例裁剪
        delta_x = width_x * crop_ratio
        delta_y = width_y * crop_ratio
        print(f"[-] 模式: 比例裁剪 (每侧切除 {crop_ratio*100:.1f}%)")

    # 定义新的边界
    limit_x_min = x_min + delta_x
    limit_x_max = x_max - delta_x
    limit_y_min = y_min + delta_y
    limit_y_max = y_max - delta_y

    # 4. 创建筛选掩码 (Mask)
    # 保留内部的点 ( > min 且 < max )
    mask_x = (points[:, 0] > limit_x_min) & (points[:, 0] < limit_x_max)
    mask_y = (points[:, 1] > limit_y_min) & (points[:, 1] < limit_y_max)
    
    # 合并 X 和 Y 的条件 (Z轴不做处理，保留所有高度)
    final_mask = mask_x & mask_y

    # 5. 应用筛选
    cropped_points = points[final_mask]
    
    # 6. 处理颜色和法线 (防止数据丢失)
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(cropped_points)
    
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        new_pcd.colors = o3d.utility.Vector3dVector(colors[final_mask])
        
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        new_pcd.normals = o3d.utility.Vector3dVector(normals[final_mask])

    # 统计信息
    removed_count = original_count - cropped_points.shape[0]
    print(f"[-] 删除点数: {removed_count} ({(removed_count/original_count)*100:.2f}%)")
    print(f"[-] 剩余点数: {cropped_points.shape[0]}")

    # 7. 保存结果
    o3d.io.write_point_cloud(output_path, new_pcd)
    print(f"[+] 结果已保存: {output_path}")

    # --- 可视化部分 (可选) ---
    # 红色 = 被删除的边缘, 绿色 = 保留的中心
    visualize = True 
    if visualize:
        print("[-] 正在启动可视化对比...")
        # 取出被删除的点用于展示
        removed_mask = ~final_mask # 取反
        removed_points = points[removed_mask]
        
        pcd_removed = o3d.geometry.PointCloud()
        pcd_removed.points = o3d.utility.Vector3dVector(removed_points)
        
        # 涂色
        new_pcd.paint_uniform_color([0, 0.8, 0]) # 绿色：保留部分
        pcd_removed.paint_uniform_color([1, 0, 0])   # 红色：被切掉的皮
        
        o3d.visualization.draw_geometries([new_pcd, pcd_removed], 
                                          window_name="绿色:保留 | 红色:删除的边缘",
                                          width=800, height=600)

if __name__ == "__main__":
    # 输入和输出文件
    input_file = "2025RMUC.pcd"
    output_file = "2025RMUC_real.pcd"
    
    # === 参数设置 ===
    
    # 方式 A: 按百分比切 (比如切掉 5% 的边缘)
    # 推荐用于不知道具体尺寸的情况
    ratio = 0.01 
    
    # 方式 B: 按距离切 (比如切掉 0.5 米)
    # 如果你知道点云单位是米，想切掉具体的边缘，把下面改成 distance = 0.5
    distance = None 
    
    crop_xy_skin(input_file, output_file, crop_ratio=ratio, crop_distance=distance)