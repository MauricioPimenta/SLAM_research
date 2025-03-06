/**---------------------------------------------------------------------------------------
 *                           diff_control.cpp
 * @brief     Main file for the diff_control package. This node subscribes to
 *            the vrpn twist topic from the vrpn_client_ros package and the
 *            pose topic from the slam_toolbox package. It calculates the
 *            velocity commands to move the robot to the desired position.
 *
 *
 *
 *-----------------------------------------------------------------------------------------**/
#include <iostream>
#include <ros/ros.h>

#include <csignal> // to catch the SIGINT signal - ctrl+c

#include "diff_controller.h"
#include "path_planner.h"

bool signal_SIGINT = false;

/*
 * Function to Handle the use of Ctrl+c to terminate the program
 */
void signalHandler(int signum)
{
    std::cout << "\n\n\nSignal " << signum << " received. Shutting down the Differential Controller Node...\n"
              << std::endl;

    signal_SIGINT = true;
}

int main(int argc, char *argv[])
{
    // register SIGINT signal and signal handler
    signal(SIGINT, signalHandler);

    // Initialize the ROS node without the ctrl+c handler so we can use our own
    ros::init(argc, argv, "diff_control", ros::init_options::NoSigintHandler);
    DiffController *controller = new DiffController();

    // PathPlanner path_planner_generator = PathPlanner();

    while (ros::ok())
    {
        ros::spinOnce();

        // if ctrl+c is pressed, delete the controller object and break the loop
        if (signal_SIGINT)
        {
            std::cout << "Deleting the controller object..." << std::endl;
            delete controller;
            // delete &path_planner_generator;
            // ros::shutdown();
            break;
        }
    }

    std::cout << "diff_control node has been shut down." << std::endl;
    return 0;
}
