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
        # Check if both Time and Position keys exist in the message.
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
    For each measured time, interpolate the goal position (for each coordinate)
    and then compute the Euclidean error.
    """
    # Interpolate each coordinate of the goal position at the measured times.
    interp_coords = []
    for i in range(3):
        interp = np.interp(measured_times, goal_times, goal_positions[:, i])
        interp_coords.append(interp)
    goal_interp = np.vstack(interp_coords).T  # shape (N, 3)
    error = np.linalg.norm(measured_positions - goal_interp, axis=1)
    return error

def plot_positions(times, positions, label):
    plt.figure()
    plt.plot(times, positions[:, 0], label=f'{label} X')
    plt.plot(times, positions[:, 1], label=f'{label} Y')
    plt.plot(times, positions[:, 2], label=f'{label} Z')
    plt.xlabel("Time")
    plt.ylabel("Position")
    plt.title(f"Position vs Time: {label}")
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_velocities(linear, angular, label):
    plt.figure()
    plt.plot(linear, label=f'{label} Linear')
    plt.plot(angular, label=f'{label} Angular')
    plt.xlabel("Message Index")
    plt.ylabel("Velocity")
    plt.title(f"Velocities: {label}")
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_error(times, error, label):
    plt.figure()
    plt.plot(times, error, label=f'Error: {label}')
    plt.xlabel("Time")
    plt.ylabel("Position Error")
    plt.title(f"Position Error vs Time: {label}")
    plt.legend()
    plt.grid(True)
    plt.show()

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
    
    # Plot positions for goal, SLAM, and VRPN data
    plot_positions(goal_times, goal_positions, 'Goal')
    plot_positions(slam_times, slam_positions, 'SLAM')
    plot_positions(vrpn_times, vrpn_positions, 'VRPN')
    
    # Plot velocities for SLAM and VRPN
    plot_velocities(slam_linear, slam_angular, 'SLAM Cmd Vel')
    plot_velocities(vrpn_linear, vrpn_angular, 'VRPN Cmd Vel')
    
    # Compute position errors using the goal as reference.
    # For SLAM:
    slam_error = compute_error(slam_times, slam_positions, goal_times, goal_positions)
    # For VRPN:
    vrpn_error = compute_error(vrpn_times, vrpn_positions, goal_times, goal_positions)
    
    # Plot the computed errors
    plot_error(slam_times, slam_error, 'SLAM vs Goal')
    plot_error(vrpn_times, vrpn_error, 'VRPN vs Goal')

if __name__ == "__main__":
    main()
