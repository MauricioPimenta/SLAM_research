#!/usr/bin/env python3

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
    times = []
    for msg in data.get(key, []):
        if "Time" in msg and "Linear" in msg and "Angular" in msg:
            times.append(msg["Time"])
            linear.append(msg["Linear"])
            angular.append(msg["Angular"])
    return np.array(times), np.array(linear), np.array(angular)

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
    
    # configura o tamanho da fonte utilizada na matplotlib
    plt.rcParams.update({'font.size': 12})
    
    # Folder path
    folder = 'messages/2025-03-21_07-50-02/'


    controller = 'slam'
    watcher = 'vrpn'

    # diff_controller_vrpn
    ctrl_files = folder + 'diff_controller' + '_' + controller
    ctrl_goal_file = ctrl_files + '_goal_messages.yaml'
    ctrl_pose_file = ctrl_files + '_pose_messages.yaml'
    ctrl_cmd_file = ctrl_files + '_cmd_vel_messages.yaml'

    watch_files = folder + 'watcher' + '_' + watcher
    watcher_goal_file = watch_files + '_goal_messages.yaml'
    watcher_pose_file = watch_files + '_pose_messages.yaml'
    watcher_cmd_file = watch_files + '_cmd_vel_messages.yaml'

    # Load YAML data from ctrl files
    ctrl_goal_data = load_yaml(ctrl_goal_file)
    ctrl_pose_data = load_yaml(ctrl_pose_file)
    ctrl_cmd_data = load_yaml(ctrl_cmd_file)

    # Load YAML data from watcher files
    watcher_goal_data = load_yaml(watcher_goal_file)
    watcher_pose_data = load_yaml(watcher_pose_file)
    watcher_cmd_data = load_yaml(watcher_cmd_file)

    # Extract positions and timestamps from the ctrl data
    print("Extracting position data ...")
    ctrl_goal_times, ctrl_goal_positions = extract_positions(ctrl_goal_data)
    ctrl_pose_times, ctrl_pose_positions = extract_positions(ctrl_pose_data)

    # Extract velocities from the ctrl data (from cmd_vel messages)
    print("Extracting velocity data ...")
    ctrl_cmd_times, ctrl_cmd_linear, ctrl_cmd_angular = extract_velocities(ctrl_cmd_data)

    # Extract positions and timestamps from the watcher data
    print("Extracting position data ...")
    watcher_goal_times, watcher_goal_positions = extract_positions(watcher_goal_data)
    watcher_pose_times, watcher_pose_positions = extract_positions(watcher_pose_data)

    # Extract velocities from the watcher data (from cmd_vel messages)
    print("Extracting velocity data ...")
    watcher_cmd_times, watcher_cmd_linear, watcher_cmd_angular = extract_velocities(watcher_cmd_data)

    # if controller is the vrpn, we should subtract the initial position from all the positions of the controller
    if controller == 'vrpn':
        ctrl_pose_positions = ctrl_pose_positions - ctrl_pose_positions[0]
    if watcher == 'vrpn':
        watcher_pose_positions = watcher_pose_positions - watcher_pose_positions[0]
        ctrl_pose_positions = ctrl_pose_positions - ctrl_pose_positions[0]


    # Compute position errors using the goal as reference.
    # Compute position errors using the goal as reference.
    ctrl_pose_error = compute_error(ctrl_pose_times, ctrl_pose_positions, ctrl_goal_times, ctrl_goal_positions)
    watcher_pose_error = compute_error(watcher_pose_times, watcher_pose_positions, watcher_goal_times, watcher_goal_positions)




    print("Preparing plots ...")

    # ----- Figure 1: Position Plots -----
    fig1, axs1 = plt.subplots(3, 1, figsize=(10, 15))

    # Goal positions
    axs1[0].plot(ctrl_goal_times, ctrl_goal_positions[:, 0], label='X')
    axs1[0].plot(ctrl_goal_times, ctrl_goal_positions[:, 1], label='Y')
    axs1[0].set_title("Goal Positions vs Time")
    axs1[0].set_xlabel("Time")
    axs1[0].set_ylabel("Position")
    axs1[0].legend()
    axs1[0].grid(True)

    # controller positions
    axs1[1].plot(ctrl_pose_times, ctrl_pose_positions[:, 0], label='X')
    axs1[1].plot(ctrl_pose_times, ctrl_pose_positions[:, 1], label='Y')
    axs1[1].set_title(controller + " Positions vs Time")
    axs1[1].set_xlabel("Time")
    axs1[1].set_ylabel("Position")
    axs1[1].legend()
    axs1[1].grid(True)

    # watcher positions
    axs1[2].plot(watcher_pose_times, watcher_pose_positions[:, 0], label='X')
    axs1[2].plot(watcher_pose_times, watcher_pose_positions[:, 1], label='Y')
    axs1[2].set_title(watcher + " Positions vs Time")
    axs1[2].set_xlabel("Time")
    axs1[2].set_ylabel("Position")
    axs1[2].legend()
    axs1[2].grid(True)

    fig1.tight_layout()

    # ----- Figure 2: Velocity Plots -----
    fig2, axs2 = plt.subplots(2, 1, figsize=(10, 10))

    # SLAM command velocities
    axs2[0].plot(ctrl_cmd_times, ctrl_cmd_linear, label='Linear')
    axs2[0].plot(ctrl_cmd_times, ctrl_cmd_angular, label='Angular')
    axs2[0].set_title(controller + " Command Velocities")
    axs2[0].set_xlabel("Message Index")
    axs2[0].set_ylabel("Velocity")
    axs2[0].legend()
    axs2[0].grid(True)

    # VRPN command velocities
    axs2[1].plot(watcher_cmd_times, watcher_cmd_linear, label='Linear')
    axs2[1].plot(watcher_cmd_times, watcher_cmd_angular, label='Angular')
    axs2[1].set_title(watcher + " Command Velocities")
    axs2[1].set_xlabel("Message Index")
    axs2[1].set_ylabel("Velocity")
    axs2[1].legend()
    axs2[1].grid(True)

    fig2.tight_layout()



    # ----- Figure 3: Error Plots -----
    fig3, axs3 = plt.subplots(3, 1, figsize=(10, 10))

    # SLAM vs Goal error
    axs3[0].plot(ctrl_pose_times, ctrl_pose_error, label= controller + ' vs Goal Error')
    axs3[0].set_title(controller + " vs Goal Position Error")
    axs3[0].set_xlabel("Time")
    axs3[0].set_ylabel("Error")
    axs3[0].legend()
    axs3[0].grid(True)

    # VRPN vs Goal error
    axs3[1].plot(watcher_pose_times, watcher_pose_error, label=watcher + ' vs Goal Error')
    axs3[1].set_title(watcher + " vs Goal Position Error")
    axs3[1].set_xlabel("Time")
    axs3[1].set_ylabel("Error")
    axs3[1].legend()
    axs3[1].grid(True)

    ctrl_vs_watcher_error = compute_error(ctrl_pose_times, ctrl_pose_positions, watcher_pose_times, watcher_pose_positions)

    # SLAM vs VRPN error
    axs3[2].plot(ctrl_pose_times, ctrl_vs_watcher_error, label= controller + ' vs ' + watcher + ' Error')
    axs3[2].set_title(controller + " vs " + watcher + " Position Error")
    axs3[2].set_xlabel("Time")
    axs3[2].set_ylabel("Error")
    axs3[2].legend()
    axs3[2].grid(True)

    fig3.tight_layout()



    print("Preparing comparison plots ...")
    # ----- Figure for Positions Comparison -----
    # Create a figure with three subplots: one for X, one for Y, and one for Z.
    coord_labels = ['X', 'Y']
    fig_pos, axs_pos = plt.subplots(2, 1, figsize=(10, 15))
    for i in range(2):
        axs_pos[i].plot(ctrl_goal_times, ctrl_goal_positions[:, i], label='Goal')
        axs_pos[i].plot(ctrl_pose_times, ctrl_pose_positions[:, i], label=controller)
        axs_pos[i].plot(watcher_pose_times, watcher_pose_positions[:, i], label=watcher)
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
    index_ctrl = np.arange(len(ctrl_cmd_linear))
    index_watcher = np.arange(len(watcher_cmd_linear))

    axs_vel[0].plot(ctrl_cmd_times, ctrl_cmd_linear, label=controller + ' Linear')
    axs_vel[0].plot(watcher_cmd_times, watcher_cmd_linear, label=watcher + ' Linear')
    axs_vel[0].set_title("Linear Velocities Comparison")
    axs_vel[0].set_xlabel("Message Index")
    axs_vel[0].set_ylabel("Linear Velocity")
    axs_vel[0].legend()
    axs_vel[0].grid(True)

    axs_vel[1].plot(ctrl_cmd_times, ctrl_cmd_angular, label=controller + ' Angular')
    axs_vel[1].plot(watcher_cmd_times, watcher_cmd_angular, label=watcher + ' Angular')
    axs_vel[1].set_title("Angular Velocities Comparison")
    axs_vel[1].set_xlabel("Message Index")
    axs_vel[1].set_ylabel("Angular Velocity")
    axs_vel[1].legend()
    axs_vel[1].grid(True)

    fig_vel.tight_layout()


    # ----- Figure for showing the trajectories X vs Y -----
    fig_traj, axs_traj = plt.subplots(1, 1, figsize=(10, 10))
    axs_traj.plot(ctrl_goal_positions[:, 0], ctrl_goal_positions[:, 1], label=controller+'Goal')
    axs_traj.plot(watcher_goal_positions[:, 0], watcher_goal_positions[:, 1], label=watcher+'Goal')
    axs_traj.plot(ctrl_pose_positions[:, 0], ctrl_pose_positions[:, 1], label=controller)
    axs_traj.plot(watcher_pose_positions[:, 0], watcher_pose_positions[:, 1], label=watcher)
    axs_traj.set_title("Trajectories X vs Y")
    axs_traj.set_xlabel("X Position")
    axs_traj.set_ylabel("Y Position")
    axs_traj.legend()
    axs_traj.grid(True)
    fig_traj.tight_layout()

    # X vs Y

    # Display all figures (each will appear in its own window)
    plt.show()
    

if __name__ == "__main__":
    main()
