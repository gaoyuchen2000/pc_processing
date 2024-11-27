There is a difference in image_saver.py and npy_to_pcd.py to calculate the W2C matrix, both are correct but the rotation matrix in front is different. In image_saver.py the relative rotation between cameras are descibed by the rotation along world axis, but the npy_to_pcd.py describes the rotation along camera's axis. 
# Rotation Calculation Methods in Camera Coordinate Transformations

This document explains two different ways to compute and describe camera rotations: **World-Axis Rotation** and **Camera-Axis Rotation**. Both methods are valid but serve different use cases and require distinct calculations.
# Rotation Calculation Methods in Camera Coordinate Transformations

---

## Camera-Axis Rotation

**Purpose**:  
Used to describe the camera’s local rotation around its own axes.

**Computation**:  
Directly calculate rotation matrices or quaternions in the camera’s local frame.  
For example, if a camera rotates 90° around its local Z-axis:
\[
R_{\text{local}} = 
\begin{bmatrix} 
\cos\theta & -\sin\theta & 0 \\ 
\sin\theta & \cos\theta & 0 \\ 
0 & 0 & 1 
\end{bmatrix}
\]
where \(\theta = 90^\circ\).

**Application**:  
- Used for pose estimation or describing the motion of cameras or robots.
- Does not rely on global alignment matrices like \( R_{\text{align}} \).

---

## Why Are They Not Interchangeable?

### **World-Axis Rotation**:
- Relies on a global reference frame to compute transformations.
- Example:
\[
R_{\text{w2c}} = R_{\text{align}} \cdot R_{\text{current}}^{-1}
\]
This ensures that all coordinate systems share the same global frame, which is critical for tasks like 3D point cloud alignment.

### **Camera-Axis Rotation**:
- Directly computes local rotations relative to the camera’s current orientation.
- Example: A 90° rotation around the camera’s local Z-axis:
\[
R_{\text{local}} = 
\begin{bmatrix} 
\cos\theta & -\sin\theta & 0 \\ 
\sin\theta & \cos\theta & 0 \\ 
0 & 0 & 1 
\end{bmatrix}
\]
Local rotations do not require the concept of a global alignment.

---

## Summary

### **World-Axis Rotations**:
- Used for aligning objects or 3D scenes in a **global world frame**.
- Requires both an alignment matrix (\(R_{\text{align}}\)) and a global rotation matrix (\(R_{\text{current}}^{-1}\)).

### **Camera-Axis Rotations**:
- Describes how the camera rotates around its **local axes**.
- Directly calculated in the camera’s local frame, useful for robotic motion or pose estimation.

---

## Choosing the Right Method

- For **global alignment** (e.g., point cloud fusion): Use **World-Axis Rotation**.
- For **local motion** (e.g., robot or camera movement): Use **Camera-Axis Rotation**.
