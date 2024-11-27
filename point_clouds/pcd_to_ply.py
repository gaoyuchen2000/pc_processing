import open3d as o3d


pcd = o3d.io.read_point_cloud("1_point_cloud.pcd")

o3d.io.write_point_cloud("combined_point_cloud.ply", pcd)

