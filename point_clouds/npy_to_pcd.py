import numpy as np
import open3d as o3d
import os
import cv2
import re

def depth_to_point_cloud(depth_image, rgb_image, fx, fy, cx, cy, min_depth=-0.4, max_depth=1.2):

    height, width = depth_image.shape


    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u = u.flatten()
    v = v.flatten()
    Z = depth_image.flatten()

    valid = (Z > min_depth) & (Z < max_depth) & (~np.isnan(Z))
    u = u[valid]
    v = v[valid]
    Z = Z[valid]
    X = (cx - u) * Z / fx
    Y = -(v - cy) * Z / fy#############################why minus?
    points = np.stack((X, Y, Z), axis=-1)
    colors = rgb_image[v, u] / 255.0  # 归一化到 [0, 1]
    return points, colors

def transform_point_cloud(points, transformation_matrix):
 
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    points_transformed = np.dot(transformation_matrix, points_homogeneous.T).T
    return points_transformed[:, :3]

def save_point_cloud_with_color(points, colors, filename):
 
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Point cloud saved to {filename}")

def compute_full_transform(align_matrix, extrinsics):
     
    R_relative = extrinsics[:3, :3]
    t_cam_in_world = extrinsics[:3, 3]
    T_w2c = compute_w2c_transform(align_matrix, R_relative, t_cam_in_world)
    T_c2w = np.linalg.inv(T_w2c)
    return T_c2w

def compute_w2c_transform(align_matrix, R_relative, t_cam_in_world):
     
    R_w2c = R_relative @ align_matrix
    T_w2c = np.eye(4)
    T_w2c[:3, :3] = R_w2c
    T_w2c[:3, 3] = -R_w2c @ t_cam_in_world
    return T_w2c

def load_images_and_convert_to_point_clouds(depth_folder, rgb_folder, output_folder, camera_params, align_matrix):
    
    os.makedirs(output_folder, exist_ok=True)
    depth_files = [f for f in os.listdir(depth_folder) if f.endswith(".png")]
    depth_files.sort(key=lambda f: int(re.findall(r'\d+', f)[0]))

    for depth_file, params in zip(depth_files, camera_params):

        depth_image_path = os.path.join(depth_folder, depth_file)
        depth_image_uint16 = cv2.imread(depth_image_path, -1)
        if depth_image_uint16 is None:
            print(f"Failed to load depth image: {depth_file}")
            continue

        depth_scale = 1000.0  
        depth_image = depth_image_uint16.astype(np.float32) / depth_scale
        print(f"Loaded depth image: {depth_file}, shape: {depth_image.shape}")


        rgb_file = depth_file.replace("_depth.png", "_rgb.png")
        rgb_image_path = os.path.join(rgb_folder, rgb_file)
        if not os.path.exists(rgb_image_path):
            print(f"RGB image not found for {rgb_file}, skipping...")
            continue

        rgb_image = cv2.imread(rgb_image_path)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        #rgb_image=cv2.flip(rgb_image,1)##########################################zhuyizheli!!!!!!!!!!!!!!!!!!!!!!
        print(f"Loaded RGB image: {rgb_file}, shape: {rgb_image.shape}")
 
        fx, fy, cx, cy = params["intrinsics"]
        extrinsics = params["extrinsics"]
        full_transform = compute_full_transform(align_matrix, extrinsics)


        camera_points, colors = depth_to_point_cloud(depth_image, rgb_image, fx, fy, cx, cy)


        world_points = transform_point_cloud(camera_points, full_transform)


        output_filename = os.path.join(output_folder, depth_file.replace("_depth.png", ".pcd"))
        save_point_cloud_with_color(world_points, colors, output_filename)

if __name__ == "__main__":
    depth_folder = "depth_images"
    rgb_folder = "rgb_images"
    output_folder = "point_clouds"

    align_matrix = np.array([
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0],
    ])

    camera_params = [
        {
            "intrinsics": (554.256, 554.256, 320.0, 240.0),  # fx, fy, cx, cy
            "extrinsics": np.array([
                [1, 0, 0, -0.4],
                [0, 1, 0, 0.4],
                [0, 0, 1, 0.05],
                [0, 0, 0, 1]
            ])
        },
        {
            "intrinsics": (554.256, 554.256, 320.0, 240.0),
            "extrinsics": np.array([
                [-1, 0, 0, 1.2],
                [0, 1, 0, 0.4],
                [0, 0, -1, 0.05],
                [0, 0, 0, 1]
            ])
        },
        {
            "intrinsics": (554.256, 554.256, 320.0, 240.0),
            "extrinsics": np.array([
                [0, 0, -1, 0.4],
                [0, 1, 0, -0.4],
                [1, 0, 0, 0.05],
                [0, 0, 0, 1]
            ])
        },
        {
            "intrinsics": (554.256, 554.256, 320.0, 240.0),
            "extrinsics": np.array([
                [0, 0, 1, 0.4],
                [0, 1, 0, 1.2],
                [-1, 0, 0, 0.05],
                [0, 0, 0, 1]
            ])
        }
    ]

    load_images_and_convert_to_point_clouds(depth_folder, rgb_folder, output_folder, camera_params, align_matrix)
