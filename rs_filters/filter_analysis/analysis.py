import numpy as np
import matplotlib.pyplot as plt


# Data files:
EKF_data_location = np.genfromtxt('data/EKF_data.csv', delimiter=',', skip_header=1)
AMCL_data_location = np.genfromtxt('data/AMCL_data.csv', delimiter=',', skip_header=1)

# Ground truth (x, y) positions
ground_truth_x_EKF = EKF_data_location[:,0]
ground_truth_y_EKF = EKF_data_location[:,1]

# Just testing that they're starting at the same time
ground_truth_x_AMCL = AMCL_data_location[:,0]
ground_truth_y_AMCL = AMCL_data_location[:,1]

# Localisation estimates
EKF_localisation_x = EKF_data_location[:,6]
EKF_localisation_y = EKF_data_location[:,7]

AMCL_localisation_x = AMCL_data_location[:,6]
AMCL_localisation_y = AMCL_data_location[:,7]

# Function to calculate RMSE
def calculate_rmse(ground_truth, estimate):
    return np.sqrt(np.mean((ground_truth - estimate)**2))

# Calculate RMSE for EKF
EKF_rmse_x = calculate_rmse(ground_truth_x_EKF, EKF_localisation_x)
EKF_rmse_y = calculate_rmse(ground_truth_y_EKF, EKF_localisation_y)
EKF_combined_rmse = np.sqrt(EKF_rmse_x**2 + EKF_rmse_x**2)

# Calculate RMSE for AMCL
AMCL_rmse_x = calculate_rmse(ground_truth_x_AMCL, AMCL_localisation_x)
AMCL_rmse_y = calculate_rmse(ground_truth_y_AMCL, AMCL_localisation_y)
AMCL_combined_rmse = np.sqrt(AMCL_rmse_x**2 + AMCL_rmse_y**2)

# Graphing
plt.figure(figsize=(12, 8))
#plt.plot(ground_truth_x_2, ground_truth_y_2, label='Ground Truth 2', color='r', alpha=1.0, linestyle='-', linewidth=1, marker='x')
plt.plot(ground_truth_x_EKF, ground_truth_y_EKF, label='Ground Truth', color='black', alpha=1, linestyle='-', linewidth=1)
plt.plot(EKF_localisation_x, EKF_localisation_y, label='EKF Estimate', color='r', alpha=0.6, linestyle='-', linewidth=1)
plt.plot(AMCL_localisation_x, AMCL_localisation_y, label='AMCL Estimate', color='g', alpha=0.6, linestyle='-', linewidth=1)

# Adding annotations for RMSE
plt.text(1.1, 0.52, f'RMSE EKF (combined): {EKF_combined_rmse:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.text(1.1, 0.48, f'RMSE AMCL (combined): {AMCL_combined_rmse:.3f}', transform=plt.gca().transAxes, fontsize=12, ha='left', va='center', color='black')
plt.tight_layout(rect=[0.1, 0.1, 0.9, 0.9])

plt.title('Ground Truth vs Localisation Estimates', fontsize=16)
plt.xlabel('X Position (metres)', fontsize=14)
plt.ylabel('Y Position (metres)', fontsize=14)
plt.legend(loc='upper left', fontsize=12)
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')

# Add axis limits for better visualization
plt.xlim([0, (max(ground_truth_x_EKF)+0.1)])
plt.ylim([0, (max(ground_truth_y_EKF)+0.1)])

# Show the plot
plt.show()
