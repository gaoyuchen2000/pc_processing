# Depth Image to Point Cloud Conversion and Alignment

## Overview

A simple converter to process depth images captured by multiple cameras, convert them into point clouds, and align these point clouds into a unified world coordinate system. The pipeline handles intrinsic and extrinsic camera parameters and transforms the point clouds based on camera positions and orientations.

---

## How It Works

### 1. Depth Image Processing
- Depth images are saved from ROS topics (`/cameraX/depth/image_raw`) using OpenCV (`cv_bridge`).
- These images are stored in `.npy` format for efficient numerical processing.

### 2. Point Cloud Generation
- Depth images are converted into 3D points in the **camera coordinate system** using the intrinsic parameters of each camera:

  \[
  X = \frac{(u - c_x) \cdot Z}{f_x}, \quad
  Y = \frac{(v - c_y) \cdot Z}{f_y}, \quad
  Z = \text{depth}(u, v)
  \]

- \(f_x, f_y, c_x, c_y\): Focal lengths and principal points from the camera's intrinsic parameters.
- \(u, v\): Pixel coordinates in the image.
- \(Z\): Depth value at the pixel.
### 3. World Coordinate Transformation
- Each camera's position and orientation in the world coordinate system is defined using **extrinsic parameters**:
- **Rotation Matrix (\(R_{\text{relative}}\))**: Defines the camera's orientation relative to the initial position.
- **Translation Vector (\(t_{\text{cam\_in\_world}}\))**: Defines the camera's position in the world coordinate system.
- Transformation matrices are constructed:
- From world to camera:
  ```
  R_w2c = R_relative @ align_matrix
  T_w2c = [[R_w2c, -R_w2c @ t_cam_in_world],
           [0, 0, 0, 1]]
  ```
- From camera to world:
  ```
  T_c2w = T_w2c⁻¹
  ```

### 4. Point Cloud Transformation
- The point cloud is transformed into the world coordinate system using \(T_{\text{c2w}}\).

### 5. Saving Results
- Transformed point clouds are saved as `.pcd` files for further processing or visualization.

---

## File Structure

- `depth_images/`: Contains the `.npy` depth images captured by each camera.
- `point_clouds/`: Outputs the `.pcd` files, one for each camera, and potentially a merged point cloud.

---
 
## Visualization

To visualize the generated point clouds, use the following script with Open3D:
```python
import open3d as o3d

pcd = o3d.io.read_point_cloud("point_clouds/merged_point_cloud.pcd")
o3d.visualization.draw_geometries([pcd])
