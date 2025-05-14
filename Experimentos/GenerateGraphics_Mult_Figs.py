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
    orientations = []
    for msg in data.get(key, []):
        if "Time" in msg and "Position" in msg and "Orientation" in msg:
            times.append(msg["Time"])
            positions.append(msg["Position"])
            orientations.append(msg["Orientation"])
    return np.array(times), np.array(positions), np.array(orientations)

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

# Set xlim for all figures
def set_xlim_all(figures, xlim):
    for fig in figures:
        for ax in fig.get_axes():
            ax.set_xlim(xlim)


def main():

    # Folder path
    folder = 'messages/2025-03-27_15-28-13/'

    # Enum containing the Exp numbers
    Exp = 2

    if Exp == 1 or Exp == 2:
        controller = 'OptiTrack'
        watcher = 'SLAM Toolbox'
    elif Exp == 3 or Exp == 4:
        controller = 'SLAM Toolbox'
        watcher = 'OptiTrack'

    if controller == 'SLAM Toolbox':
        controller_file = 'slam'
        watcher_file = 'vrpn'
    elif controller == 'OptiTrack':
        controller_file = 'vrpn'
        watcher_file = 'slam'

    # diff_controller_vrpn
    ctrl_files = folder + 'diff_controller' + '_' + controller_file
    ctrl_goal_file = ctrl_files + '_goal_messages.yaml'
    ctrl_pose_file = ctrl_files + '_pose_messages.yaml'
    ctrl_cmd_file = ctrl_files + '_cmd_vel_messages.yaml'

    watch_files = folder + 'watcher' + '_' + watcher_file
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
    ctrl_goal_times_abs, ctrl_goal_positions, ctrl_goal_orientations = extract_positions(ctrl_goal_data)
    ctrl_pose_times_abs, ctrl_pose_positions, ctrl_pose_orientations = extract_positions(ctrl_pose_data)

    # Extract velocities from the ctrl data (from cmd_vel messages)
    print("Extracting velocity data ...")
    ctrl_cmd_times_abs, ctrl_cmd_linear, ctrl_cmd_angular = extract_velocities(ctrl_cmd_data)

    # Extract positions and timestamps from the watcher data
    print("Extracting position data ...")
    watcher_goal_times_abs, watcher_goal_positions, watcher_goal_orientations = extract_positions(watcher_goal_data)
    watcher_pose_times_abs, watcher_pose_positions, watcher_pose_orientations = extract_positions(watcher_pose_data)

    # Extract velocities from the watcher data (from cmd_vel messages)
    print("Extracting velocity data ...")
    watcher_cmd_times_abs, watcher_cmd_linear, watcher_cmd_angular = extract_velocities(watcher_cmd_data)




    # Turn the times into relative times
    watcher_goal_times = watcher_goal_times_abs - watcher_pose_times_abs[0]
    watcher_pose_times = watcher_pose_times_abs - watcher_pose_times_abs[0]
    watcher_cmd_times = watcher_cmd_times_abs - watcher_pose_times_abs[0]

    ctrl_goal_times = ctrl_goal_times_abs - watcher_pose_times_abs[0]
    ctrl_cmd_times = ctrl_cmd_times_abs - watcher_pose_times_abs[0]
    ctrl_pose_times = ctrl_pose_times_abs - watcher_pose_times_abs[0]


    print("watcher time 0: " + str(watcher_pose_times[0]))
    print("watcher time last: " + str(watcher_pose_times[-1]))
    print("ctrl time 0: " + str(ctrl_pose_times[0]))
    print("ctrl time last: " + str(ctrl_pose_times[-1]))

    # put the watcher, controller and goal in the same reference frame (vrpn)
    if controller_file == "vrpn":
        watcher_pose_positions = watcher_pose_positions - watcher_pose_positions[0]
        watcher_pose_positions = watcher_pose_positions + ctrl_pose_positions[0]
        
        goal_first = ctrl_goal_positions[0]
        ctrl_goal_positions = ctrl_goal_positions - goal_first
        ctrl_goal_positions = ctrl_goal_positions + ctrl_pose_positions[0]
        ctrl_goal_positions = ctrl_goal_positions + goal_first
        
        goal_first = watcher_goal_positions[0]
        watcher_goal_positions = watcher_goal_positions - goal_first
        watcher_goal_positions = watcher_goal_positions + ctrl_pose_positions[0]
        watcher_goal_positions = watcher_goal_positions + goal_first
    if watcher_file == "vrpn":
        ctrl_pose_positions = ctrl_pose_positions - ctrl_pose_positions[0]
        ctrl_pose_positions = ctrl_pose_positions + watcher_pose_positions[0]
        
        goal_first = ctrl_goal_positions[0]
        ctrl_goal_positions = ctrl_goal_positions - goal_first
        ctrl_goal_positions = ctrl_goal_positions + watcher_pose_positions[0]
        ctrl_goal_positions = ctrl_goal_positions + goal_first
        
        goal_first = watcher_goal_positions[0]
        watcher_goal_positions = watcher_goal_positions - goal_first
        watcher_goal_positions = watcher_goal_positions + watcher_pose_positions[0]
        watcher_goal_positions = watcher_goal_positions + goal_first
        
    # ctrl_pose_positions = ctrl_pose_positions - ctrl_pose_positions[0]
    # ctrl_pose_positions = ctrl_pose_positions + watcher_pose_positions[0]
    # watcher_pose_positions = watcher_pose_positions + (0.25, 0, 0)


    # Compute position errors using the goal as reference.
    # Compute position errors using the goal as reference.
    ctrl_pose_error = compute_error(ctrl_pose_times, ctrl_pose_positions, ctrl_goal_times, ctrl_goal_positions)
    watcher_pose_error = compute_error(watcher_pose_times, watcher_pose_positions, watcher_goal_times, watcher_goal_positions)



    # ----- Figures Configurations ------ #

    print("Preparing plots ...")

    # Define the xlim range
    xmin = min(ctrl_goal_times[0], ctrl_pose_times[0], watcher_pose_times[0], 0)
    if xmin < 0:
        xmin = 0
    xmax = max(ctrl_goal_times[-1], ctrl_pose_times[-1], watcher_pose_times[-1])
    xlim_range = (xmin, xmax)
    # xlim_range = (1742552750, 1742552865)

    # List of all figures that X is the time axis
    all_figures = []
    
    pos_label = "Posição (m)"
    Time_Label = "Tempo (s)"
    Vel_Label = "Velocidade (m/s)"
    Lin_Vel_Label = "Velocidade Linear (m/s)"
    Ang_Vel_Label = "Velocidade Angular (rad/s)"
    error_Label = "Erro (m)"
    

    figsize = (8, 8)
    plt.rcParams['font.size'] = 12
    # plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['axes.labelsize'] = 16
    plt.rcParams['axes.titlesize'] = 16
    plt.rcParams['legend.fontsize'] = 10
    plt.rcParams['legend.framealpha'] = 0.8
    plt.rcParams['legend.loc'] = 'best'

    # ----- Figure 1: Position Plots -----
    fig1, axs1 = plt.subplots(3, 1, figsize=figsize)

    # Goal positions
    axs1[0].plot(ctrl_goal_times, ctrl_goal_positions[:, 0], label='X')
    axs1[0].plot(ctrl_goal_times, ctrl_goal_positions[:, 1], label='Y')
    axs1[0].set_title("Posição Desejada x Tempo")
    axs1[0].set_xlabel(Time_Label)
    axs1[0].set_ylabel(pos_label)
    axs1[0].legend()
    axs1[0].grid(True)

    # controller positions
    axs1[1].plot(ctrl_pose_times, ctrl_pose_positions[:, 0], label='X')
    axs1[1].plot(ctrl_pose_times, ctrl_pose_positions[:, 1], label='Y')
    axs1[1].set_title("Posição " + controller + " x Tempo")
    axs1[1].set_xlabel(Time_Label)
    axs1[1].set_ylabel(pos_label)
    axs1[1].legend()
    axs1[1].grid(True)

    # watcher positions
    axs1[2].plot(watcher_pose_times, watcher_pose_positions[:, 0], label='X')
    axs1[2].plot(watcher_pose_times, watcher_pose_positions[:, 1], label='Y')
    axs1[2].set_title("Posição " + watcher + " x Tempo")
    axs1[2].set_xlabel(Time_Label)
    axs1[2].set_ylabel(pos_label)
    axs1[2].legend()
    axs1[2].grid(True)

    fig1.tight_layout()
    
    all_figures.append(fig1)

    # ----- Figure 2: Velocity Plots -----
    fig2, axs2 = plt.subplots(2, 1, figsize=figsize)

    # SLAM command velocities
    axs2[0].plot(ctrl_cmd_times, ctrl_cmd_linear, label='Linear')
    axs2[0].plot(ctrl_cmd_times, ctrl_cmd_angular, label='Angular')
    axs2[0].set_title("Comandos de Velocidade " + controller)
    axs2[0].set_xlabel(Time_Label)
    axs2[0].set_ylabel(Vel_Label)
    axs2[0].legend()
    axs2[0].grid(True)

    # VRPN command velocities
    axs2[1].plot(watcher_cmd_times, watcher_cmd_linear, label='Linear')
    axs2[1].plot(watcher_cmd_times, watcher_cmd_angular, label='Angular')
    axs2[1].set_title("Comandos de Velocidade " + watcher)
    axs2[1].set_xlabel(Time_Label)
    axs2[1].set_ylabel(Vel_Label)
    axs2[1].legend()
    axs2[1].grid(True)

    fig2.tight_layout()

    all_figures.append(fig2)


    # ----- Figure 3: Error Plots -----
    fig3, axs3 = plt.subplots(3, 1, figsize=figsize)

    # SLAM vs Goal error
    axs3[0].plot(ctrl_pose_times, ctrl_pose_error, label= controller + ' vs Goal Error')
    axs3[0].set_title("Erro de posição " + controller + " vs Objetivo")
    axs3[0].set_xlabel(Time_Label)
    axs3[0].set_ylabel("Erro (m)")
    axs3[0].legend()
    axs3[0].grid(True)

    # VRPN vs Goal error
    axs3[1].plot(watcher_pose_times, watcher_pose_error, label="Erro " + watcher + ' vs Objetivo')
    axs3[1].set_title(watcher + " vs Goal Position Error")
    axs3[1].set_xlabel(Time_Label)
    axs3[1].set_ylabel("Erro (m)")
    axs3[1].legend()
    axs3[1].grid(True)

    ctrl_vs_watcher_error = compute_error(ctrl_pose_times, ctrl_pose_positions, watcher_pose_times, watcher_pose_positions)

    # SLAM vs VRPN error
    axs3[2].plot(ctrl_pose_times, ctrl_vs_watcher_error, label= 'Erro ' + controller + ' vs ' + watcher)
    axs3[2].set_title("Erro de Posição " + controller + " vs " + watcher)
    axs3[2].set_xlabel(Time_Label)
    axs3[2].set_ylabel("Erro (m)")
    axs3[2].legend()
    axs3[2].grid(True)

    fig3.tight_layout()
    
    all_figures.append(fig3)


    # ----- Figure 4: Plots For Errors in X and Y -----
    # Controller and watcher errors n X and Y compared to the goal
    fig4, axs4 = plt.subplots(2, 1, figsize=figsize)
    # Compute errors in X and Y separately
    ctrl_goal_interp_x = np.interp(ctrl_pose_times, ctrl_goal_times, ctrl_goal_positions[:, 0])
    ctrl_goal_interp_y = np.interp(ctrl_pose_times, ctrl_goal_times, ctrl_goal_positions[:, 1])
    watcher_goal_interp_x = np.interp(watcher_pose_times, watcher_goal_times, watcher_goal_positions[:, 0])
    watcher_goal_interp_y = np.interp(watcher_pose_times, watcher_goal_times, watcher_goal_positions[:, 1])

    ctrl_error_x = ctrl_pose_positions[:, 0] - ctrl_goal_interp_x
    ctrl_error_y = ctrl_pose_positions[:, 1] - ctrl_goal_interp_y
    watcher_error_x = watcher_pose_positions[:, 0] - watcher_goal_interp_x
    watcher_error_y = watcher_pose_positions[:, 1] - watcher_goal_interp_y

    # Plot errors in X
    axs4[0].plot(ctrl_pose_times, ctrl_error_x, label='Erro X do ' + controller)
    axs4[0].plot(watcher_pose_times, watcher_error_x, label='Erro X do ' + watcher)
    axs4[0].set_title("Evolução do erro de posição X")
    axs4[0].set_xlabel(Time_Label)
    axs4[0].set_ylabel("Erro X (m)")
    axs4[0].legend()
    axs4[0].grid(True)

    # Plot errors in Y
    axs4[1].plot(ctrl_pose_times, ctrl_error_y, label='Erro Y do ' + controller)
    axs4[1].plot(watcher_pose_times, watcher_error_y, label='Erro Y do ' + watcher)
    axs4[1].set_title("Evolução do erro de posição Y")
    axs4[1].set_xlabel(Time_Label)
    axs4[1].set_ylabel("Erro Y (m)")
    axs4[1].legend()
    axs4[1].grid(True)

    fig4.tight_layout()
    all_figures.append(fig4)
    
    
    # ----- Figure 5: Plots For Errors in X and Y -----
    # Plot the Errors in X and Y of the Controller compared to the watcher
    fig5, axs5 = plt.subplots(2, 1, figsize=figsize)
    
    # Compute errors in X and Y separately
    # if the controller is the vrpn, we use the ctrl_pose_times as the interpolation times
    if controller_file == "vrpn":
        interp_times = ctrl_pose_times
    # if the watcher is the vrpn, we use the watcher_pose_times as the interpolation times
    if watcher_file == "vrpn":
        interp_times = watcher_pose_times

    ctrl_pose_interp_x = np.interp(interp_times, ctrl_pose_times, ctrl_pose_positions[:, 0])
    ctrl_pose_interp_y = np.interp(interp_times, ctrl_pose_times, ctrl_pose_positions[:, 1])
    watcher_pose_interp_x = np.interp(interp_times, watcher_pose_times, watcher_pose_positions[:, 0])
    watcher_pose_interp_y = np.interp(interp_times, watcher_pose_times, watcher_pose_positions[:, 1])

    ctrl_v_watcher_error_x = ctrl_pose_interp_x - watcher_pose_interp_x
    ctrl_v_watcher_error_y = ctrl_pose_interp_y - watcher_pose_interp_y

    # Plot errors in X
    axs5[0].plot(interp_times, ctrl_v_watcher_error_x, label=controller + " vs " + watcher + ' X Error')
    axs5[0].set_title("X Position Error vs Time")
    axs5[0].set_xlabel("Time (s)")
    axs5[0].set_ylabel("X Error")
    axs5[0].legend()
    axs5[0].grid(True)

    # Plot errors in Y
    axs5[1].plot(interp_times, ctrl_v_watcher_error_y, label=controller + " vs " + watcher + ' Y Error')
    axs5[1].set_title("Y Position Error vs Time")
    axs5[1].set_xlabel("Time (s)")
    axs5[1].set_ylabel("Y Error")
    axs5[1].legend()
    axs5[1].grid(True)

    fig5.tight_layout()
    all_figures.append(fig5)
    


    print("Preparing comparison plots ...")
    # ----- Figure for Positions Comparison -----
    # Create a figure with three subplots: one for X, one for Y, and one for Z.
    coord_labels = ['X', 'Y']
    fig_pos, axs_pos = plt.subplots(2, 1, figsize=figsize)
    
    for i in range(2):
        axs_pos[i].plot(ctrl_goal_times, ctrl_goal_positions[:, i], label='Desejado')
        axs_pos[i].plot(ctrl_pose_times, ctrl_pose_positions[:, i], label=controller)
        axs_pos[i].plot(watcher_pose_times, watcher_pose_positions[:, i], label=watcher)
        axs_pos[i].set_title(f"Evolução da Posição {coord_labels[i]}")
        axs_pos[i].set_xlabel(Time_Label)
        axs_pos[i].set_ylabel(f"Posição em {coord_labels[i]} (m)")
        axs_pos[i].legend()
        axs_pos[i].grid(True)
    fig_pos.tight_layout()
    all_figures.append(fig_pos)



    # ----- Figure for Velocities Comparison -----
    # Create a figure with two subplots: one for linear velocities and one for angular velocities.
    fig_vel, axs_vel = plt.subplots(2, 1, figsize=figsize)

    axs_vel[0].plot(ctrl_cmd_times, ctrl_cmd_linear, label=controller + ' Linear')
    axs_vel[0].plot(watcher_cmd_times, watcher_cmd_linear, label=watcher + ' Linear')
    axs_vel[0].set_title("Linear Velocities Comparison")
    axs_vel[0].set_xlabel("Time (s)")
    axs_vel[0].set_ylabel("Linear Velocity")
    axs_vel[0].legend()
    axs_vel[0].grid(True)

    axs_vel[1].plot(ctrl_cmd_times, ctrl_cmd_angular, label=controller + ' Angular')
    axs_vel[1].plot(watcher_cmd_times, watcher_cmd_angular, label=watcher + ' Angular')
    axs_vel[1].set_title("Angular Velocities Comparison")
    axs_vel[1].set_xlabel("Times (s)")
    axs_vel[1].set_ylabel("Angular Velocity")
    axs_vel[1].legend()
    axs_vel[1].grid(True)

    fig_vel.tight_layout()
    all_figures.append(fig_vel)


    # ----- Figure for showing the trajectories X vs Y -----
    fig_traj, axs_traj = plt.subplots(1, 1, figsize=figsize)
    axs_traj.plot(ctrl_goal_positions[:, 0], ctrl_goal_positions[:, 1], linestyle='dashed', label='Desejada')
    axs_traj.plot(ctrl_pose_positions[:, 0], ctrl_pose_positions[:, 1], label=controller)
    axs_traj.plot(watcher_pose_positions[:, 0], watcher_pose_positions[:, 1], label=watcher)
    axs_traj.set_title("Trajetória Realizada e Desejada")
    axs_traj.set_xlabel("X Position")
    axs_traj.set_ylabel("Y Position")
    # axs_traj.tick_params(axis='both', which='major', labelsize=10)
    axs_traj.legend()
    axs_traj.grid(True, 'both', 'both')
    
    fig_traj.tight_layout()

    # Apply xlim to all figures
    set_xlim_all(all_figures, xlim_range)

    # Display all figures (each will appear in its own window)
    plt.show()



if __name__ == "__main__":
    main()
