import open3d as o3d
import numpy as np

def create_axis_with_ticks_o3d(axis_length=1.0, step=0.2):
 
    geometries = []

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_length, origin=[0, 0, 0])
    geometries.append(axes)


    for axis, color in zip(["X", "Y", "Z"], [(1, 0, 0), (0, 1, 0), (0, 0, 1)]):
        for i in range(int(axis_length / step) + 1):
            value = i * step
            position = [0, 0, 0]
            if axis == "X":
                position = [value, 0, 0]
            elif axis == "Y":
                position = [0, value, 0]
            elif axis == "Z":
                position = [0, 0, value]

            tick = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            tick.translate(position)
            tick.paint_uniform_color(color)
            geometries.append(tick)


            text = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02, origin=position)
            geometries.append(text)

    return geometries

def draw_point_clouds_with_ticks_o3d(pcd_files, axis_length=1.0, step=0.2):
 
    geometries = []


    for idx, pcd_file in enumerate(pcd_files):
        pcd = o3d.io.read_point_cloud(pcd_file)
        pcd.paint_uniform_color(np.random.rand(3))  
        geometries.append(pcd)


    geometries.extend(create_axis_with_ticks_o3d(axis_length, step))


    o3d.visualization.draw_geometries(geometries)


pcd_files = [
    "camera1_depth.pcd",
    "camera2_depth.pcd",
    "camera3_depth.pcd",
    "camera4_depth.pcd"
]


draw_point_clouds_with_ticks_o3d(pcd_files, axis_length=1, step=0.2)

