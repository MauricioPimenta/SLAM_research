import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    return data

def extract_positions(data, key='messages'):
    """Extract time and position (assumed to be a list of 3 values) from a YAML file."""
    times = []
    positions = []
    for msg in data.get(key, []):
        if "Time" in msg and "Position" in msg:
            times.append(msg["Time"])
            positions.append(msg["Position"])
    return np.array(times), np.array(positions)

def extract_velocities(data, key='messages'):
    """Extract linear and angular velocities from a YAML file."""
    linear = []
    angular = []
    for msg in data.get(key, []):
        if "Linear" in msg and "Angular" in msg:
            linear.append(msg["Linear"])
            angular.append(msg["Angular"])
    return np.array(linear), np.array(angular)

def compute_error(measured_times, measured_positions, goal_times, goal_positions):
    """
    Interpolate the goal positions at the measured times and compute the Euclidean error.
    """
    interp_coords = []
    for i in range(3):
        interp = np.interp(measured_times, goal_times, goal_positions[:, i])
        interp_coords.append(interp)
    goal_interp = np.vstack(interp_coords).T
    error = np.linalg.norm(measured_positions - goal_interp, axis=1)
    return error

def plot_positions(ax, times, positions, label):
    ax.plot(times, positions[:, 0], label=f'{label} X')
    ax.plot(times, positions[:, 1], label=f'{label} Y')
    ax.plot(times, positions[:, 2], label=f'{label} Z')
    ax.set_xlabel("Time")
    ax.set_ylabel("Position")
    ax.set_title(f"Position vs Time: {label}")
    ax.legend()
    ax.grid(True)

def plot_velocities(ax, linear, angular, label):
    ax.plot(linear, label=f'{label} Linear')
    ax.plot(angular, label=f'{label} Angular')
    ax.set_xlabel("Message Index")
    ax.set_ylabel("Velocity")
    ax.set_title(f"Velocities: {label}")
    ax.legend()
    ax.grid(True)

def plot_error(ax, times, error, label):
    ax.plot(times, error, label=f'Error: {label}')
    ax.set_xlabel("Time")
    ax.set_ylabel("Position Error")
    ax.set_title(f"Position Error vs Time: {label}")
    ax.legend()
    ax.grid(True)

def main():
    # Folder path
    folder = 'messages/2025-03-14_08-19-00_SLAM_HOUSE/'
    
    # File paths (ensure these files are in your working directory)
    goal_file = folder + '_goal_messages.yaml'
    slam_file = folder + '_slam_messages.yaml'
    vrpn_file = folder + '_vrpn_messages.yaml'
    slam_cmd_vel_file = folder + '_slam_cmd_vel_messages.yaml'
    vrpn_cmd_vel_file = folder + '_vrpn_cmd_vel_messages.yaml'
    
    # Load YAML data from files
    goal_data = load_yaml(goal_file)
    slam_data = load_yaml(slam_file)
    vrpn_data = load_yaml(vrpn_file)
    slam_cmd_vel_data = load_yaml(slam_cmd_vel_file)
    vrpn_cmd_vel_data = load_yaml(vrpn_cmd_vel_file)
    
    # Extract positions and timestamps
    goal_times, goal_positions = extract_positions(goal_data)
    slam_times, slam_positions = extract_positions(slam_data)
    vrpn_times, vrpn_positions = extract_positions(vrpn_data)
    
    # Extract velocities (from cmd_vel messages)
    slam_linear, slam_angular = extract_velocities(slam_cmd_vel_data)
    vrpn_linear, vrpn_angular = extract_velocities(vrpn_cmd_vel_data)
    
    # Compute position errors using the goal as reference
    slam_error = compute_error(slam_times, slam_positions, goal_times, goal_positions)
    vrpn_error = compute_error(vrpn_times, vrpn_positions, goal_times, goal_positions)
    
    # Create subplots: we'll use a 4x2 grid (8 subplots) and leave one empty.
    fig, axs = plt.subplots(4, 2, figsize=(15, 20))
    
    # Row 1: Positions
    plot_positions(axs[0, 0], goal_times, goal_positions, 'Goal')
    plot_positions(axs[0, 1], slam_times, slam_positions, 'SLAM')
    
    # Row 2: More Positions and Velocities
    plot_positions(axs[1, 0], vrpn_times, vrpn_positions, 'VRPN')
    plot_velocities(axs[1, 1], slam_linear, slam_angular, 'SLAM Cmd Vel')
    
    # Row 3: Velocities and Error
    plot_velocities(axs[2, 0], vrpn_linear, vrpn_angular, 'VRPN Cmd Vel')
    plot_error(axs[2, 1], slam_times, slam_error, 'SLAM vs Goal')
    
    # Row 4: Error and empty subplot
    plot_error(axs[3, 0], vrpn_times, vrpn_error, 'VRPN vs Goal')
    axs[3, 1].axis('off')  # Hide the unused subplot
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
