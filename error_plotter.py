import numpy as np
import matplotlib.pyplot as plt

# Load data from files
x_data = np.loadtxt('x.txt')
x_hat_data = np.loadtxt('xhat.txt')
x_data_len = x_data.shape[0]
x_hat_data_len = x_hat_data.shape[0]
min_len = min(x_data_len, x_hat_data_len)
x_data = x_data[:min_len]
x_hat_data = x_hat_data[:min_len]
# Ensure both files have the same number of entries
if x_data.shape != x_hat_data.shape:
    raise ValueError("The files x.txt and x_hat.txt must have the same number of rows and columns.")

# Extract columns
time = x_data[:, 0]
bearing = x_data[:, 1]
x_pos = x_data[:, 2]
y_pos = x_data[:, 3]
left_wheel = x_data[:, 4]
right_wheel = x_data[:, 5]

bearing_hat = x_hat_data[:, 1]
x_pos_hat = x_hat_data[:, 2]
y_pos_hat = x_hat_data[:, 3]
left_wheel_hat = x_hat_data[:, 4]
right_wheel_hat = x_hat_data[:, 5]

bearing_error = np.abs(bearing - bearing_hat)
x_pos_error = np.abs(x_pos - x_pos_hat)
y_pos_error = np.abs(y_pos - y_pos_hat)
left_wheel_error = np.abs(left_wheel - left_wheel_hat)
right_wheel_error = np.abs(right_wheel - right_wheel_hat)

# Create a figure with subplots
fig, axs = plt.subplots(4, 1, figsize=(10, 15))

axs[0].scatter(time, x_pos_error, label='X Position Error (m)', color='orange')
axs[0].set_ylabel('X Position Error (m)')
axs[0].legend()

axs[1].scatter(time, y_pos_error, label='Y Position Error (m)', color='green')
axs[1].set_ylabel('Y Position Error (m)')
axs[1].legend()

axs[2].scatter(time, left_wheel_error, label='Left Wheel Error (rad)', color='red')
axs[2].set_ylabel('Left Wheel Error (rad)')
axs[2].legend()

axs[3].scatter(time, right_wheel_error, label='Right Wheel Error (rad)', color='purple')
axs[3].set_ylabel('Right Wheel Error (rad)')
axs[3].set_xlabel('Time (s)')
axs[3].legend()

# Adjust layout for better spacing
plt.tight_layout()

# Show the plot
plt.show()

# Calculate absolute errors
def rmse(true, pred):
    return np.sqrt(np.mean((true - pred) ** 2))

bearing_rmse = rmse(bearing, bearing_hat)
x_pos_rmse = rmse(x_pos, x_pos_hat)
y_pos_rmse = rmse(y_pos, y_pos_hat)
left_wheel_rmse = rmse(left_wheel, left_wheel_hat)
right_wheel_rmse = rmse(right_wheel, right_wheel_hat)

print(f"X Position RMSE: {x_pos_rmse}")
print(f"Y Position RMSE: {y_pos_rmse}")

print(f"X and Y RMSE: {np.linalg.norm([x_pos_rmse, y_pos_rmse])}")
print("Identity")