import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_yaml(file_path):
    print(f"Loading {file_path} ...")

    # try loading the file
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        print(f"Loaded {file_path}.")
    # if not possible, print an error message and return None
    except FileNotFoundError:
        print(f"File {file_path} not found.")
        data = None
    return data


def extract_positions(data, key='messages'):
    """Extract time and 3D position data from a YAML file."""
    times = []
    positions = []
    try:
        for msg in data.get(key, []):
            if "Time" in msg and "Position" in msg:
                times.append(msg["Time"])
                positions.append(msg["Position"])
    except AttributeError:
        print("Error: Unable to extract position data.")
        return np.array([]), np.array([])

    return np.array(times), np.array(positions)

def extract_velocities(data, key='messages'):
    """Extract linear and angular velocities from a YAML file."""
    linear = []
    angular = []
    try:
        for msg in data.get(key, []):
            if "Linear" in msg and "Angular" in msg:
                linear.append(msg["Linear"])
                angular.append(msg["Angular"])
    except AttributeError:
        print("Error: Unable to extract velocity data.")
        return np.array([]), np.array([])

    return np.array(linear), np.array(angular)

def compute_error(measured_times, measured_positions, goal_times, goal_positions):
    """
    Interpolate the goal positions at the measured times and compute the Euclidean error.
    """
    print("Computing errors between measured positions and goal positions ...")
    interp_coords = []
    try:
        for i in range(3):
            interp = np.interp(measured_times, goal_times, goal_positions[:, i])
            interp_coords.append(interp)
        goal_interp = np.vstack(interp_coords).T
        error = np.linalg.norm(measured_positions - goal_interp, axis=1)
        print("Errors computed.")
    except ValueError:
        print("Error: Unable to compute errors.")
        error = np.array([])
    return error

def main():
    # Folder path
    folder = 'messages/2025-03-18_17-17-48/'

    # File paths (ensure these files are in your working directory)
    goal_file = folder + '_goal_messages.yaml'
    pose_file = folder + '_pose_messages.yaml'

    cmd_file = folder + '_cmd_vel_messages.yaml'

    # Load YAML data from files
    goal_data = load_yaml(goal_file)
    pose_data = load_yaml(pose_file)

    cmd_data = load_yaml(cmd_file)

    
    # Extract positions and timestamps
    print("Extracting position data ...")
    goal_times, goal_positions = extract_positions(goal_data)
    pose_times, pose_positions = extract_positions(pose_data)

    # Extract velocities (from cmd_vel messages)
    print("Extracting velocity data ...")
    cmd_linear, cmd_angular = extract_velocities(cmd_data)

    # Compute position errors using the goal as reference.
    pose_error = compute_error(pose_times, pose_positions, goal_times, goal_positions)

    print("Preparing plots ...")

    # ----- Figure 1: Position Plots -----
    fig1, axs1 = plt.subplots(2, 1, figsize=(10, 15))

    # Goal positions
    axs1[0].plot(goal_times, goal_positions[:, 0], label='X')
    axs1[0].plot(goal_times, goal_positions[:, 1], label='Y')
    axs1[0].set_title("Goal Positions vs Time")
    axs1[0].set_xlabel("Time")
    axs1[0].set_ylabel("Position")
    axs1[0].legend()
    axs1[0].grid(True)

    # Pose positions
    axs1[1].plot(pose_times, pose_positions[:, 0], label='X')
    axs1[1].plot(pose_times, pose_positions[:, 1], label='Y')
    axs1[1].set_title("Pose Positions vs Time")
    axs1[1].set_xlabel("Time")
    axs1[1].set_ylabel("Position")
    axs1[1].legend()
    axs1[1].grid(True)

    fig1.tight_layout()

    # ----- Figure 2: Velocity Plots -----
    fig2, axs2 = plt.subplots(1, 1, figsize=(10, 10))

    # Command velocities
    axs2.plot(cmd_linear, label='Linear')
    axs2.plot(cmd_angular, label='Angular')
    axs2.set_title("Command Velocities")
    axs2.set_xlabel("Message Index")
    axs2.set_ylabel("Velocity")
    axs2.legend()
    axs2.grid(True)

    fig2.tight_layout()



    # ----- Figure 3: Error Plots -----
    fig3, axs3 = plt.subplots(1, 1, figsize=(10, 10))

    # Pose vs Goal error
    axs3.plot(pose_times, pose_error, label='Pose vs Goal Error')
    axs3.set_title("Pose vs Goal Position Error")
    axs3.set_xlabel("Time")
    axs3.set_ylabel("Error")
    axs3.legend()
    axs3.grid(True)

    fig3.tight_layout()




    print("Preparing comparison plots ...")
    # ----- Figure for Positions Comparison -----
    # Create a figure with three subplots: one for X, one for Y, and one for Z.
    coord_labels = ['X', 'Y']
    fig_pos, axs_pos = plt.subplots(2, 1, figsize=(10, 15))
    for i in range(2):
        axs_pos[i].plot(goal_times, goal_positions[:, i], label='Goal')
        axs_pos[i].plot(pose_times, pose_positions[:, i], label='Pose')
        axs_pos[i].set_title(f"{coord_labels[i]} Position vs Time")
        axs_pos[i].set_xlabel("Time")
        axs_pos[i].set_ylabel(f"{coord_labels[i]} Position")
        axs_pos[i].legend()
        axs_pos[i].grid(True)
    fig_pos.tight_layout()
    
    
    # ------ Figure with position and command velocity comparison -----
    fig_pos_vel, axs_pos_vel = plt.subplots(2, 1, figsize=(10, 15))
    for i in range(2):
        axs_pos_vel[i].plot(goal_times, goal_positions[:, i], label='Goal')
        axs_pos_vel[i].plot(pose_times, pose_positions[:, i], label='Pose')
        axs_pos_vel[i].plot(cmd_linear, label='Command Linear')
        axs_pos_vel[i].plot(cmd_angular, label='Command Angular')
        axs_pos_vel[i].set_title(f"{coord_labels[i]} Position vs Time")
        axs_pos_vel[i].set_xlabel("Time")
        axs_pos_vel[i].set_ylabel(f"{coord_labels[i]} Position")
        axs_pos_vel[i].legend()
        axs_pos_vel[i].grid(True)
    fig_pos_vel.tight_layout()



    # ----- Figure for Velocities Comparison -----
    # Create a figure with two subplots: one for linear velocities and one for angular velocities.
    fig_vel, axs_vel = plt.subplots(2, 1, figsize=(10, 10))

    # Use the message index as the x-axis.
    index_cmd = np.arange(len(cmd_linear))

    axs_vel[0].plot(index_cmd, cmd_linear, label='Command Linear')
    axs_vel[0].set_title("Linear Velocities Comparison")
    axs_vel[0].set_xlabel("Message Index")
    axs_vel[0].set_ylabel("Linear Velocity")
    axs_vel[0].legend()
    axs_vel[0].grid(True)


    axs_vel[1].plot(index_cmd, cmd_angular, label='Command Angular')
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
