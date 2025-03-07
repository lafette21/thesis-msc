import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Set random seed for reproducibility
np.random.seed(42)

# Parameters
plane_size = 4  # Width/height of the square planes
plane_points = 800  # Number of points per plane
cylinder_radius = 0.35  # Radius of the cylinder
cylinder_height = 3  # Distance between planes (height of cylinder)
cylinder_points = 500  # Number of points on cylinder surface
z_lower = 0  # Z-level of the bottom plane
z_upper = z_lower + cylinder_height  # Z-level of the top plane

# --- Generate Points for Bottom Plane ---
# Random points in XY-plane at z = z_lower
x_bottom = np.random.uniform(-plane_size/2, plane_size/2, plane_points)
y_bottom = np.random.uniform(-plane_size/2, plane_size/2, plane_points)
z_bottom = np.full(plane_points, z_lower)  # Constant Z for bottom plane

# --- Generate Points for Top Plane ---
x_top = np.random.uniform(-plane_size/2, plane_size/2, plane_points)
y_top = np.random.uniform(-plane_size/2, plane_size/2, plane_points)
z_top = np.full(plane_points, z_upper)  # Constant Z for top plane

# --- Generate Points for Cylinder ---
# Parametric equations: x = r*cos(θ), y = r*sin(θ), z = varies linearly
theta = np.random.uniform(0, 2*np.pi, cylinder_points)  # Angular coordinate
z_cylinder = np.random.uniform(z_lower, z_upper, cylinder_points)  # Height along Z
x_cylinder = cylinder_radius * np.cos(theta)
y_cylinder = cylinder_radius * np.sin(theta)

# Create a 3D plot
fig = plt.figure(figsize=(8, 8))
ax1 = fig.add_subplot(111, projection='3d')

# Plot top plane points
ax1.scatter(x_top, y_top, z_top, color='#33AF84', s=10, label='Plane Point')

# Plot bottom plane points
ax1.scatter(x_bottom, y_bottom, z_bottom, color='#E63946', s=10, label='Plane Point')

# Plot cylinder points
ax1.scatter(x_cylinder, y_cylinder, z_cylinder, color='#031926', s=10, label='Cylinder Point')

# Set labels
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('Plane Filtering')

# Set axis limits
ax1.set_xlim([0, 10])
ax1.set_ylim([0, 10])
ax1.set_zlim([0, 10])
ax1.axis('equal')

# Remove grid, ticks, and background for a clean look
ax1.grid(False)
ax1.set_xticks([])
ax1.set_yticks([])
ax1.set_zticks([])
ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

ax1.legend()

# Adjust the view angle
#  ax1.view_init(elev=35.264, azim=45)
ax1.view_init(elev=20, azim=45)

plt.savefig('plane-filtering.pdf')
plt.show()
