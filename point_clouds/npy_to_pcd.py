import numpy as np
import open3d as o3d
import os

def depth_to_point_cloud(depth_image, fx, fy, cx, cy):

    height, width = depth_image.shape
    points = []

    for v in range(height):
        for u in range(width):
            Z = depth_image[v, u]
            if Z == 0 or np.isnan(Z):  # 跳过无效深度
                continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points.append([X, Y, Z])
    return np.array(points)  
    
def compute_w2c_transform(align_matrix, R_relative, t_cam_in_world):
    R_w2c = R_relative @ align_matrix
    T_w2c = np.eye(4)
    T_w2c[:3, :3] = R_w2c
    T_w2c[:3, 3] = -R_w2c @ t_cam_in_world

    return T_w2c
    
def compute_full_transform(align_matrix, extrinsics):
    R_relative = extrinsics[:3, :3]  
    t_cam_in_world = extrinsics[:3, 3]  
    T_w2c=compute_w2c_transform(align_matrix,R_relative,t_cam_in_world)
    T_c2w = np.linalg.inv(T_w2c)

    return T_c2w

def transform_point_cloud(points, transformation_matrix):
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    points_transformed = np.dot(transformation_matrix, points_homogeneous.T).T
    return points_transformed[:, :3] 
    
def save_point_cloud_as_pcd(points, filename):

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Point cloud saved to {filename}")

def load_depth_images_and_convert(input_folder, output_folder, camera_params, align_matrix):

    os.makedirs(output_folder, exist_ok=True)
    depth_files = [f for f in os.listdir(input_folder) if f.endswith(".npy")]

    for depth_file, params in zip(depth_files, camera_params):

        depth_image = np.load(os.path.join(input_folder, depth_file))
        print(f"Loaded depth image: {depth_file}, shape: {depth_image.shape}")


        fx, fy, cx, cy = params["intrinsics"]
        extrinsics = params["extrinsics"]


        full_transform = compute_full_transform(align_matrix, extrinsics)


        camera_points = depth_to_point_cloud(depth_image, fx, fy, cx, cy)


        world_points = transform_point_cloud(camera_points, full_transform)


        output_filename = os.path.join(output_folder, depth_file.replace(".npy", ".pcd"))
        save_point_cloud_as_pcd(world_points, output_filename)

if __name__ == "__main__":

    input_folder = "depth_images" 
    output_folder = "point_clouds"  


    align_matrix = np.array([
        [0,1,0],
        [0,0,1], 
        [1,0,0],  
  
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
                [-1,0,0, 1.2],
                [0, 1, 0, 0.4],
                [0, 0,-1, 0.05],
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
                [-1,0, 0, 0.05],
                [0, 0, 0, 1]
            ])
        }
    ]

##here the coordinates are, the relative rotation between the init position and the current position, 't' is the coordinates of location of cameras in world
    load_depth_images_and_convert(input_folder, output_folder, camera_params, align_matrix)

