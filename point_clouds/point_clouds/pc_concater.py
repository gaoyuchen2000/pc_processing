import open3d as o3d

pcd1 = o3d.io.read_point_cloud("camera1_depth.pcd")
pcd2 = o3d.io.read_point_cloud("camera2_depth.pcd")
pcd3 = o3d.io.read_point_cloud("camera3_depth.pcd")
pcd4 = o3d.io.read_point_cloud("camera4_depth.pcd")

combined_pcd = pcd1 + pcd2 + pcd3 + pcd4

o3d.io.write_point_cloud("combined_point_cloud.pcd", combined_pcd)
print("Combined point cloud saved as combined_point_cloud.pcd")

o3d.visualization.draw_geometries([combined_pcd])

