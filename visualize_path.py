import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

def load_tum_file(filename):
    """
    Parses a TUM format file.
    Format: timestamp x y z q_x q_y q_z q_w
    Returns: numpy arrays for x, y, and yaw
    """
    timestamps = []
    xs = []
    ys = []
    quats = [] # [x, y, z, w]

    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        return None, None, None

    with open(filename, 'r') as f:
        for line in f:
            # Skip comments
            if line.startswith('#') or not line.strip():
                continue
            
            parts = line.split()
            if len(parts) < 8:
                continue

            # Extract data
            # TUM: ts x y z qx qy qz qw
            ts = float(parts[0])
            tx = float(parts[1])
            ty = float(parts[2])
            tz = float(parts[3])
            qx = float(parts[4])
            qy = float(parts[5])
            qz = float(parts[6])
            qw = float(parts[7])

            timestamps.append(ts)
            xs.append(tx)
            ys.append(ty)
            quats.append([qx, qy, qz, qw])

    # Convert lists to numpy arrays
    xs = np.array(xs)
    ys = np.array(ys)
    quats = np.array(quats)

    # Convert quaternions to Yaw (Rotation around Z)
    # Scipy expects quaternions in scalar-last format (x, y, z, w), which matches TUM
    r = R.from_quat(quats)
    euler = r.as_euler('xyz', degrees=False) # Returns [roll, pitch, yaw]
    yaws = euler[:, 2] # Extract yaw (index 2)

    return xs, ys, yaws

def visualize_path(x, y, yaw, filename):
    plt.figure(figsize=(10, 6))
    
    # 1. Plot the continuous path
    plt.plot(x, y, label='Path', color='blue', linewidth=1.5, alpha=0.7)

    # 2. Add Orientation Arrows (Quiver)
    # We downsample the arrows to prevent cluttering the plot
    arrow_density = max(1, len(x) // 30)  # Aim for roughly 30 arrows total
    
    # U and V components for arrows
    u = np.cos(yaw)
    v = np.sin(yaw)
    
    plt.quiver(x[::arrow_density], y[::arrow_density], 
               u[::arrow_density], v[::arrow_density], 
               color='red', scale=20, scale_units='inches', 
               width=0.005, headwidth=4, label='Orientation')

    # 3. Mark Start and End
    plt.scatter(x[0], y[0], color='green', marker='o', s=100, label='Start', zorder=5)
    plt.scatter(x[-1], y[-1], color='red', marker='x', s=100, label='End', zorder=5)

    # Formatting
    plt.title(f"2D Path Visualization: {os.path.basename(filename)}")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.axis('equal') # Important to keep aspect ratio correct for paths
    plt.grid(True, which='both', linestyle='--')
    plt.legend()
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize a 2D path from a TUM format file.")
    parser.add_argument("filename", type=str, help="Path to the TUM format file (.txt)")
    
    args = parser.parse_args()
    
    x, y, yaw = load_tum_file(args.filename)
    
    if x is not None:
        print(f"Loaded {len(x)} poses. Plotting...")
        visualize_path(x, y, yaw, args.filename)