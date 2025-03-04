import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a 3D voxel grid
grid_size = 3
voxelgrid = np.ones((grid_size, grid_size, grid_size), dtype=bool)

# Create a 3D plot
fig = plt.figure(figsize=(12, 6))
ax1 = fig.add_subplot(131, projection='3d')

# Plot the voxel grid
ax1.voxels(voxelgrid, facecolors='white', edgecolors='#363636', shade=False, alpha=0.8)

# Create a separate voxel grid for the top-right 3x3 subgrid at the top Z-layer
subgrid_size = 1
highlight_grid = np.zeros((grid_size, grid_size, grid_size), dtype=bool)

# Define the top-right 3x3 subgrid at the top Z-layer
# Top-right in XY-plane means high x, high y; top in Z means high z
for x in range(grid_size - subgrid_size, grid_size):  # x = 6 to 8
    for y in range(grid_size - subgrid_size, grid_size):  # y = 6 to 8
        for z in range(grid_size - subgrid_size, grid_size):  # z = 6 to 8
            highlight_grid[x, y, z] = True

# Overlay the highlighted subgrid with red edges (no fill to avoid obscuring the original grid)
ax1.voxels(highlight_grid, facecolors='white', edgecolors='#E63946', linewidth=1.5, shade=False, alpha=0.5)

# Set labels
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('Unfiltered Voxel Grid')

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

# Adjust the view angle
ax1.view_init(elev=35.264, azim=45)

# Create a 3D voxel grid
grid_size = 1
voxelgrid = np.ones((grid_size, grid_size, grid_size), dtype=bool)

ax2 = fig.add_subplot(132, projection='3d')

# Plot the voxel grid
ax2.voxels(voxelgrid, facecolors='white', edgecolors='#363636', shade=False, alpha=0.1)

points = np.array([[0.4, 0.3, 0.2], [0.6, 0.3, 0.7], [0.25, 0.6, 0.5], [0.4, 0.5, 0.8], [0.4, 0.7, 0.5]])

ax2.scatter(points[:,0], points[:,1], points[:,2], color='#18789F', s=30, label='Point')

# Set labels
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_zlabel('Z (m)')
ax2.set_title('Unfiltered Voxel')

# Set axis limits
ax2.set_xlim([0, 10])
ax2.set_ylim([0, 10])
ax2.set_zlim([0, 10])
ax2.axis('equal')

# Remove grid, ticks, and background for a clean look
ax2.grid(False)
ax2.set_xticks([])
ax2.set_yticks([])
ax2.set_zticks([])
ax2.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax2.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax2.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Add a legend
ax2.legend()

# Adjust the view angle
ax2.view_init(elev=35.264, azim=45)

# Create a 3D voxel grid
grid_size = 1
voxelgrid = np.ones((grid_size, grid_size, grid_size), dtype=bool)

ax3 = fig.add_subplot(133, projection='3d')

# Plot the voxel grid
ax3.voxels(voxelgrid, facecolors='white', edgecolors='#363636', shade=False, alpha=0.1)

ax3.scatter([0.41], [0.48], [0.54], color='#E63946', s=30, label='Point')

# Set labels
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')
ax3.set_zlabel('Z (m)')
ax3.set_title('Filtered Voxel')

# Set axis limits
ax3.set_xlim([0, 10])
ax3.set_ylim([0, 10])
ax3.set_zlim([0, 10])
ax3.axis('equal')

# Remove grid, ticks, and background for a clean look
ax3.grid(False)
ax3.set_xticks([])
ax3.set_yticks([])
ax3.set_zticks([])
ax3.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax3.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax3.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Add a legend
ax3.legend()

# Adjust the view angle
ax3.view_init(elev=35.264, azim=45)

# Show the plot
plt.savefig('voxel-grid.pdf')
plt.show()
