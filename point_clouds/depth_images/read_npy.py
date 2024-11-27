import numpy as np

 
depth_image = np.load("camera2_depth.npy")
 
print(f"Depth image shape: {depth_image.shape}")
print(depth_image)

 
import matplotlib.pyplot as plt
plt.imshow(depth_image, cmap="jet")   
plt.colorbar()
plt.show()

