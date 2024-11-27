import numpy as np

# 读取npy文件
depth_array = np.load('/home/g/catkin_ws/src/image_world/point_clouds/camera2/depth_image_2.npy')

# 打印数组信息
print("Array shape:", depth_array.shape)
print("Array dtype:", depth_array.dtype)
print("Array contents:\n", depth_array)
import matplotlib.pyplot as plt

# 显示深度图
plt.imshow(depth_array, cmap='gray')
plt.colorbar()
plt.title("Depth Image")
plt.show()
