import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(12, 6))

# --- First subplot: Side-view (XZ plane, looking along Y-axis) ---
ax1 = fig.add_subplot(121, projection='3d')

# Lidar
sensor_pos = np.array([0, 0, 1.5])
ax1.scatter([sensor_pos[0]], [sensor_pos[1]], [sensor_pos[2]], color='#031926', s=100, label='LiDAR Sensor')

# Ray
ray_direction = np.array([1.5, 1.5, 1.5]) - sensor_pos
ray_length = 3.5  # Extend the ray beyond the sphere
ray_end = sensor_pos + ray_length * ray_direction / np.linalg.norm(ray_direction)
ax1.plot([sensor_pos[0], ray_end[0]], [sensor_pos[1], ray_end[1]], [sensor_pos[2], ray_end[2]], color='#E63946', label='LiDAR Ray')

# Plane
xx, zz = np.meshgrid(np.linspace(0, 3, 10), np.linspace(0, 3, 10))
yy = np.ones_like(xx) * 1.5  # Centered at x=1.5
ax1.plot_surface(xx, yy, zz, color='#86C8E2', alpha=0.4, label='Plane')
#  ax1.plot_surface(x, y, z, color='#86C8E2', shade=False, alpha=0.4, label='Sphere')

# Set axis labels
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')

# Set axis limits
ax1.set_xlim([-0.5, 2])
ax1.set_ylim([-0.5, 2])
ax1.set_zlim([-0.5, 2])
ax1.axis('equal')

# Remove grid, ticks, and background for a clean look
ax1.grid(False)
ax1.set_xticks([])
ax1.set_yticks([])
ax1.set_zticks([])
ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Add a title for the first subplot
ax1.set_title('Side View')

# Add a legend
ax1.legend()

# Adjust view angle
#  ax1.view_init(elev=20, azim=45)
ax1.view_init(elev=0, azim=0)

# --- Second subplot: Different angle (3D perspective view) ---
ax2 = fig.add_subplot(122, projection='3d')  # 122 means 1 row, 2 columns, 2nd subplot

# Lidar
sensor_pos = np.array([0, 0, 1.5])
ax2.scatter([sensor_pos[0]], [sensor_pos[1]], [sensor_pos[2]], color='#031926', s=100, label='LiDAR Sensor')

# Ray
ray_direction = np.array([1.5, 1.5, 1.5]) - sensor_pos
ray_length = 5.5  # Extend the ray beyond the sphere
ray_end = sensor_pos + ray_length * ray_direction / np.linalg.norm(ray_direction)
ax2.plot([sensor_pos[0], ray_end[0]], [sensor_pos[1], ray_end[1]], [sensor_pos[2], ray_end[2]], color='#E63946', label='LiDAR Ray')

# Plane
xx, zz = np.meshgrid(np.linspace(0, 3, 10), np.linspace(0, 2.5, 10))
yy = np.ones_like(xx) * 1.5  # Centered at x=1.5
ax2.plot_surface(xx, yy, zz, color='#86C8E2', alpha=0.4, label='Plane')

# Set axis labels
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_zlabel('Z (m)')

# Set axis limits
ax2.set_xlim([-0.5, 2])
ax2.set_ylim([-0.5, 2])
ax2.set_zlim([-0.5, 2])
ax2.axis('equal')

# Remove grid, ticks, and background for a clean look
ax2.grid(False)
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_zticks([])
ax2.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax2.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax2.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Add a title for the first subplot
ax2.set_title('Isometric View')

# Add a legend
ax2.legend()

# Adjust view angle
ax2.view_init(elev=35.264, azim=45)

#  plt.show()
plt.savefig('plane-plot.pdf')
