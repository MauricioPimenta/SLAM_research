import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_yaml(file_path):
    print(f"Loading {file_path} ...")
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    print(f"Loaded {file_path}.")
    return data


def extract_positions(data, key='messages'):
    """Extract time and 3D position data from a YAML file."""
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
    print("Computing errors between measured positions and goal positions ...")
    interp_coords = []
    for i in range(3):
        interp = np.interp(measured_times, goal_times, goal_positions[:, i])
        interp_coords.append(interp)
    goal_interp = np.vstack(interp_coords).T
    error = np.linalg.norm(measured_positions - goal_interp, axis=1)
    print("Errors computed.")
    return error

def main():
    # Folder path
    folder = 'messages/2025-03-18_17-17-48/'

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
    print("Extracting position data ...")
    goal_times, goal_positions = extract_positions(goal_data)
    slam_times, slam_positions = extract_positions(slam_data)
    vrpn_times, vrpn_positions = extract_positions(vrpn_data)

    # Extract velocities (from cmd_vel messages)
    print("Extracting velocity data ...")
    slam_linear, slam_angular = extract_velocities(slam_cmd_vel_data)
    vrpn_linear, vrpn_angular = extract_velocities(vrpn_cmd_vel_data)

    # Compute position errors using the goal as reference.
    slam_error = compute_error(slam_times, slam_positions, goal_times, goal_positions)
    vrpn_error = compute_error(vrpn_times, vrpn_positions, goal_times, goal_positions)

    print("Preparing plots ...")

    # ----- Figure 1: Position Plots -----
    fig1, axs1 = plt.subplots(3, 1, figsize=(10, 15))

    # Goal positions
    axs1[0].plot(goal_times, goal_positions[:, 0], label='X')
    axs1[0].plot(goal_times, goal_positions[:, 1], label='Y')
    axs1[0].set_title("Goal Positions vs Time")
    axs1[0].set_xlabel("Time")
    axs1[0].set_ylabel("Position")
    axs1[0].legend()
    axs1[0].grid(True)

    # SLAM positions
    axs1[1].plot(slam_times, slam_positions[:, 0], label='X')
    axs1[1].plot(slam_times, slam_positions[:, 1], label='Y')
    axs1[1].set_title("SLAM Positions vs Time")
    axs1[1].set_xlabel("Time")
    axs1[1].set_ylabel("Position")
    axs1[1].legend()
    axs1[1].grid(True)

    # VRPN positions
    axs1[2].plot(vrpn_times, vrpn_positions[:, 0], label='X')
    axs1[2].plot(vrpn_times, vrpn_positions[:, 1], label='Y')
    axs1[2].set_title("VRPN Positions vs Time")
    axs1[2].set_xlabel("Time")
    axs1[2].set_ylabel("Position")
    axs1[2].legend()
    axs1[2].grid(True)

    fig1.tight_layout()

    # ----- Figure 2: Velocity Plots -----
    fig2, axs2 = plt.subplots(2, 1, figsize=(10, 10))

    # SLAM command velocities
    axs2[0].plot(slam_linear, label='Linear')
    axs2[0].plot(slam_angular, label='Angular')
    axs2[0].set_title("SLAM Command Velocities")
    axs2[0].set_xlabel("Message Index")
    axs2[0].set_ylabel("Velocity")
    axs2[0].legend()
    axs2[0].grid(True)

    # VRPN command velocities
    axs2[1].plot(vrpn_linear, label='Linear')
    axs2[1].plot(vrpn_angular, label='Angular')
    axs2[1].set_title("VRPN Command Velocities")
    axs2[1].set_xlabel("Message Index")
    axs2[1].set_ylabel("Velocity")
    axs2[1].legend()
    axs2[1].grid(True)

    fig2.tight_layout()

    # ----- Figure 3: Error Plots -----
    fig3, axs3 = plt.subplots(2, 1, figsize=(10, 10))

    # SLAM vs Goal error
    axs3[0].plot(slam_times, slam_error, label='SLAM vs Goal Error')
    axs3[0].set_title("SLAM vs Goal Position Error")
    axs3[0].set_xlabel("Time")
    axs3[0].set_ylabel("Error")
    axs3[0].legend()
    axs3[0].grid(True)

    # VRPN vs Goal error
    axs3[1].plot(vrpn_times, vrpn_error, label='VRPN vs Goal Error')
    axs3[1].set_title("VRPN vs Goal Position Error")
    axs3[1].set_xlabel("Time")
    axs3[1].set_ylabel("Error")
    axs3[1].legend()
    axs3[1].grid(True)

    fig3.tight_layout()

    print("Preparing comparison plots ...")
    # ----- Figure for Positions Comparison -----
    # Create a figure with three subplots: one for X, one for Y, and one for Z.
    coord_labels = ['X', 'Y']
    fig_pos, axs_pos = plt.subplots(3, 1, figsize=(10, 15))
    for i in range(2):
        axs_pos[i].plot(goal_times, goal_positions[:, i], label='Goal')
        axs_pos[i].plot(slam_times, slam_positions[:, i], label='SLAM')
        axs_pos[i].plot(vrpn_times, vrpn_positions[:, i], label='VRPN')
        axs_pos[i].set_title(f"{coord_labels[i]} Position vs Time")
        axs_pos[i].set_xlabel("Time")
        axs_pos[i].set_ylabel(f"{coord_labels[i]} Position")
        axs_pos[i].legend()
        axs_pos[i].grid(True)
    fig_pos.tight_layout()

    # ----- Figure for Velocities Comparison -----
    # Create a figure with two subplots: one for linear velocities and one for angular velocities.
    fig_vel, axs_vel = plt.subplots(2, 1, figsize=(10, 10))

    # Use the message index as the x-axis.
    index_slam = np.arange(len(slam_linear))
    index_vrpn = np.arange(len(vrpn_linear))

    axs_vel[0].plot(index_slam, slam_linear, label='SLAM Linear')
    axs_vel[0].plot(index_vrpn, vrpn_linear, label='VRPN Linear')
    axs_vel[0].set_title("Linear Velocities Comparison")
    axs_vel[0].set_xlabel("Message Index")
    axs_vel[0].set_ylabel("Linear Velocity")
    axs_vel[0].legend()
    axs_vel[0].grid(True)

    axs_vel[1].plot(index_slam, slam_angular, label='SLAM Angular')
    axs_vel[1].plot(index_vrpn, vrpn_angular, label='VRPN Angular')
    axs_vel[1].set_title("Angular Velocities Comparison")
    axs_vel[1].set_xlabel("Message Index")
    axs_vel[1].set_ylabel("Angular Velocity")
    axs_vel[1].legend()
    axs_vel[1].grid(True)

    fig_vel.tight_layout()

    # Display all figures (each will appear in its own window)
    plt.show()

if __name__ == "__main__":
    main()
