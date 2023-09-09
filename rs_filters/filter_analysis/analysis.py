import numpy as np
import matplotlib.pyplot as plt

# Generate fake ground truth and localisation estimates
np.random.seed(0)
n = 100  # Number of data points

# Fake ground truth (x, y) positions
ground_truth_x = np.linspace(0, 10, n)
ground_truth_y = np.linspace(0, 10, n)

# Fake localisation estimates with some random noise
localisation_x = ground_truth_x + np.random.normal(0, 0.5, n)
localisation_y = ground_truth_y + np.random.normal(0, 0.5, n)

# Function to calculate RMSE
def calculate_rmse(ground_truth, estimate):
    return np.sqrt(np.mean((ground_truth - estimate)**2))

# Calculate RMSE for both x and y coordinates
rmse_x = calculate_rmse(ground_truth_x, localisation_x)
rmse_y = calculate_rmse(ground_truth_y, localisation_y)
combined_rmse = np.sqrt(rmse_x**2 + rmse_y**2)

# Graphing
plt.figure(figsize=(12, 8))
plt.plot(ground_truth_x, ground_truth_y, label='Ground Truth', color='g', alpha=1.0, linestyle='-', linewidth=1)
plt.plot(localisation_x, localisation_y, label='Localisation Estimate', color='r', alpha=0.6, linestyle='-', linewidth=0.5)

# Adding annotations for RMSE
plt.text(1.1, 0.6, f'RMSE (X): {rmse_x:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.text(1.1, 0.5, f'RMSE (Y): {rmse_y:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.text(1.1, 0.4, f'RMSE (combined): {combined_rmse:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.tight_layout(rect=[0.1, 0.1, 0.9, 0.9])

plt.title('Ground Truth vs Localisation Estimates', fontsize=16)
plt.xlabel('X Position', fontsize=14)
plt.ylabel('Y Position', fontsize=14)
plt.legend(loc='upper left', fontsize=12)
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')

# Add axis limits for better visualization
plt.xlim([-1, 11])
plt.ylim([-1, 11])

# Show the plot
plt.show()
