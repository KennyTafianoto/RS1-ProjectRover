import numpy as np
import matplotlib.pyplot as plt


# Data files:
EKF_data_location = np.genfromtxt('data/EKF_data.csv', delimiter=',', skip_header=1)
AMCL_data_location = np.genfromtxt('data/AMCL_data.csv', delimiter=',', skip_header=1)

# Ground truth (x, y) positions
ground_truth_x = EKF_data_location[:,0]
ground_truth_y = EKF_data_location[:,1]

# Localisation estimates
EKF_localisation_x = EKF_data_location[:,6]
EKF_localisation_y = EKF_data_location[:,7]

AMCL_localisation_x = AMCL_data_location[:,6]
AMCL_localisation_y = AMCL_data_location[:,7]

# Function to calculate RMSE
def calculate_rmse(ground_truth, estimate):
    return np.sqrt(np.mean((ground_truth - estimate)**2))

# Calculate RMSE for both x and y coordinates
rmse_x = calculate_rmse(ground_truth_x, EKF_localisation_x)
rmse_y = calculate_rmse(ground_truth_y, EKF_localisation_y)
combined_rmse = np.sqrt(rmse_x**2 + rmse_y**2)

# Graphing
plt.figure(figsize=(12, 8))
plt.plot(ground_truth_x, ground_truth_y, label='Ground Truth', color='black', alpha=1.0, linestyle='-', linewidth=1)
plt.plot(EKF_localisation_x, EKF_localisation_y, label='EKF Estimate', color='r', alpha=0.6, linestyle='-', linewidth=1)

plt.plot(AMCL_localisation_x, AMCL_localisation_y, label='AMCL Estimate', color='g', alpha=0.6, linestyle='-', linewidth=1)


# Adding annotations for RMSE
plt.text(1.1, 0.6, f'RMSE (X): {rmse_x:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.text(1.1, 0.5, f'RMSE (Y): {rmse_y:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.text(1.1, 0.4, f'RMSE (combined): {combined_rmse:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.tight_layout(rect=[0.1, 0.1, 0.9, 0.9])

plt.title('Ground Truth vs Localisation Estimates', fontsize=16)
plt.xlabel('X Position (metres)', fontsize=14)
plt.ylabel('Y Position (metres)', fontsize=14)
plt.legend(loc='upper left', fontsize=12)
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')

# Add axis limits for better visualization
plt.xlim([0, (max(ground_truth_x)+0.1)])
plt.ylim([0, (max(ground_truth_y)+0.1)])

# Show the plot
plt.show()
