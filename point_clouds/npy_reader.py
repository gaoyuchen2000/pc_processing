import numpy as np
depth_array = np.load('/home/g/catkin_ws/src/image_world/point_clouds/camera2/depth_image_2.npy')

print("Array shape:", depth_array.shape)
print("Array dtype:", depth_array.dtype)
print("Array contents:\n", depth_array)
import matplotlib.pyplot as plt

plt.imshow(depth_array, cmap='gray')
plt.colorbar()
plt.title("Depth Image")
plt.show()
