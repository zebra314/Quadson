import numpy as np
import matplotlib.pyplot as plt

def analyze_stability(observations):
  # Convert lists to numpy arrays
  pos = np.array(observations['pos'])
  euler_ori = np.array(observations['euler_ori'])
  linear_vel = np.array(observations['linear_vel'])

  # Compute statistics for reduced factors
  metrics = {
    'roll_std': np.std(euler_ori[:, 0]),
    'pitch_std': np.std(euler_ori[:, 1]),
    'height_std': np.std(pos[:, 2]),
    'linear_vel_x_std': np.std(linear_vel[:, 0]),  # Forward velocity
    'linear_vel_z_std': np.std(linear_vel[:, 2]),  # Vertical velocity
  }

  # Print stability summary
  print("Stability Analysis (Standard Deviation over 3 seconds):")
  print(f"Roll Std: {metrics['roll_std']:.4f} rad")
  print(f"Pitch Std: {metrics['pitch_std']:.4f} rad")
  print(f"Height Std: {metrics['height_std']:.4f} m")
  print(f"Linear Velocity X Std: {metrics['linear_vel_x_std']:.4f} m/s")
  print(f"Linear Velocity Z Std: {metrics['linear_vel_z_std']:.4f} m/s")

  return metrics

def plot_stability(times, observations, metrics):
  pos = np.array(observations['pos'])
  euler_ori = np.array(observations['euler_ori'])
  linear_vel = np.array(observations['linear_vel'])


  # Plot 1: Orientation (Roll and Pitch only)
  plt.figure(figsize=(10, 4))
  plt.plot(times, euler_ori[:, 0], label=f"Roll (std={metrics['roll_std']:.4f})")
  plt.plot(times, euler_ori[:, 1], label=f"Pitch (std={metrics['pitch_std']:.4f})")
  plt.xlabel("Time (s)")
  plt.ylabel("Orientation (rad)")
  plt.title("Orientation Over Time")
  plt.legend()
  plt.grid(True)

  plt.tight_layout()
  plt.show()

  # Plot 2: Height (Z Position)
  # plt.plot(times, pos[:, 2], label=f"Height (std={metrics['height_std']:.4f})")
  # plt.xlabel("Time (s)")
  # plt.ylabel("Height (m)")
  # plt.title("Height Over Time")
  # plt.legend()
  # plt.grid(True)

  # plt.tight_layout()
  # plt.show()

  # Plot 3: Linear Velocity (X and Z only)
  plt.figure(figsize=(10, 4))
  plt.plot(times, linear_vel[:, 0], label=f"Vx (std={metrics['linear_vel_x_std']:.4f})")
  plt.plot(times, linear_vel[:, 2], label=f"Vz (std={metrics['linear_vel_z_std']:.4f})")
  plt.xlabel("Time (s)")
  plt.ylabel("Linear Velocity (m/s)")
  plt.title("Linear Velocity Over Time")
  plt.legend()
  plt.grid(True)

  plt.tight_layout()
  plt.show()