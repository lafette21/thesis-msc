import numpy as np
import matplotlib.pyplot as plt

# Generate sample data (noisy points around a HALF circle)
np.random.seed(42)

# Sample theta over [0, π] for a half circle (upper half)
theta = np.linspace(0, np.pi, 100)  # Reduced number of points for clarity
true_x0, true_y0 = 0, 0  # True center
true_r = 0.35  # True radius
x_true = true_x0 + true_r * np.cos(theta)
y_true = true_y0 + true_r * np.sin(theta)

# Add some noise
noise = np.random.normal(0, 0.025, size=theta.shape)
x_data = x_true + noise
y_data = y_true + noise

# Form the matrix A and vector b for the linear system
N = len(x_data)
A = np.zeros((N, 3))
b = np.zeros(N)

for i in range(N):
    A[i, 0] = 1  # Constant term
    A[i, 1] = -2 * x_data[i]  # Coefficient of x_0
    A[i, 2] = -2 * y_data[i]  # Coefficient of y_0
    b[i] = -(x_data[i]**2 + y_data[i]**2)  # Right-hand side

# Solve the linear system using the pseudoinverse (for over-determined system)
params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
a0, x0, y0 = params  # a0 = x_0^2 + y_0^2 - R^2, x0 = x_0, y0 = y_0

# Solve for radius R
# From a0 = x_0^2 + y_0^2 - R^2, R^2 = x_0^2 + y_0^2 - a0
R_squared = x0**2 + y0**2 - a0
R = np.sqrt(R_squared) if R_squared > 0 else np.sqrt(abs(R_squared))  # Take positive root

print(f"Fitted center: ({x0:.3f}, {y0:.3f}), Radius: {R:.3f}")

# Generate points for the fitted circle (full circle for visualization)
theta_fit = np.linspace(0, 2 * np.pi, 200)
x_fit = x0 + R * np.cos(theta_fit)
y_fit = y0 + R * np.sin(theta_fit)

# Create the plot
plt.figure(figsize=(8, 8))

plt.scatter(x_data, y_data, color='#031926', s=20, label='LiDAR Point')

plt.plot(x_fit, y_fit, '-', color='#E63946', label=f'Fitted Circle (r = {R:.3f})')

plt.plot(true_x0, true_y0, 'o', color='#33AF84', label='True Center')

plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Circle Fitting with Least-Squares')

plt.xlim([-1, 1])
plt.ylim([-1, 1])
plt.axis('equal')  # Equal aspect ratio to avoid circle distortion

plt.grid(False)

plt.legend()

plt.savefig('circle-fitting.pdf')
plt.show()
