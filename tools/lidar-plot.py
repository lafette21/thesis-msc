import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
SENSOR_POS = np.array([0, 0, 0])  # LiDAR sensor position (x, y, z)
RAY_LENGTH = 1.5  # Length of rays in meters
HORIZONTAL_FOV = 360  # Horizontal field of view in degrees
VERTICAL_FOV = 30  # Vertical field of view in degrees (-15 to 15)
NUM_VERTICAL_RAYS = 16  # Number of vertical rays
ANGULAR_RES_FACTOR = 1200 / 60 * 5.5296e-05  # Angular resolution factor

# Calculate angular resolutions
ang_res_hor = ANGULAR_RES_FACTOR * HORIZONTAL_FOV
num_horizontal_rays = int(HORIZONTAL_FOV / ang_res_hor) // 30
angles_hor = np.radians(np.linspace(0, HORIZONTAL_FOV, num_horizontal_rays))  # Convert to radians
angles_ver = np.radians(np.linspace(-VERTICAL_FOV / 2, VERTICAL_FOV / 2, NUM_VERTICAL_RAYS))

# Compute ray directions (vectorized)
cos_v, sin_v = np.cos(angles_ver), np.sin(angles_ver)
cos_h, sin_h = np.cos(angles_hor), np.sin(angles_hor)
ray_dirs = np.array([(cos_v[j] * sin_h[i], cos_v[j] * cos_h[i], sin_v[j])
                     for i in range(len(angles_hor)) for j in range(len(angles_ver))])

# Normalize and scale ray directions to endpoints
ray_ends = SENSOR_POS + RAY_LENGTH * ray_dirs / np.linalg.norm(ray_dirs, axis=1)[:, np.newaxis]

# Initialize figure and 3D axes
fig = plt.figure(figsize=(12, 6))
ax1 = fig.add_subplot(121, projection='3d')

# Plot sensor position
ax1.scatter(*SENSOR_POS, color='#031926', s=100, label='LiDAR Sensor')

# Plot rays efficiently (single call per ray bundle)
for i in range(len(ray_ends)):
    ax1.plot([SENSOR_POS[0], ray_ends[i, 0]],
            [SENSOR_POS[1], ray_ends[i, 1]],
            [SENSOR_POS[2], ray_ends[i, 2]],
            color='#E63946', linewidth=0.5, alpha=0.5, label='LiDAR Ray' if i == 0 else None)

# Set labels and limits
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_xlim([-2, 2])
ax1.set_ylim([-2, 2])
ax1.set_zlim([-2, 2])

# Clean up the plot
ax1.grid(False)
ax1.set_xticks([]), ax1.set_yticks([]), ax1.set_zticks([])
ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Title and legend
ax1.set_title('Side View')
ax1.legend()

# Adjust view angle
ax1.view_init(elev=0, azim=0)

ax2 = fig.add_subplot(122, projection='3d')

# Plot sensor position
ax2.scatter(*SENSOR_POS, color='#031926', s=100, label='LiDAR Sensor')

# Plot rays efficiently (single call per ray bundle)
for i in range(len(ray_ends)):
    ax2.plot([SENSOR_POS[0], ray_ends[i, 0]],
            [SENSOR_POS[1], ray_ends[i, 1]],
            [SENSOR_POS[2], ray_ends[i, 2]],
            color='#E63946', linewidth=0.5, alpha=0.5, label='LiDAR Ray' if i == 0 else None)

# Set labels and limits
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_zlabel('Z (m)')
ax2.set_xlim([-2, 2])
ax2.set_ylim([-2, 2])
ax2.set_zlim([-2, 2])

# Clean up the plot
ax2.grid(False)
ax2.set_xticks([]), ax2.set_yticks([]), ax2.set_zticks([])
ax2.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax2.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax2.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Title and legend
ax2.set_title('Isometric View')
ax2.legend()

# Adjust view angle
ax2.view_init(elev=35.264, azim=45)

# Save and display
plt.savefig('lidar-plot.pdf')
plt.show()
