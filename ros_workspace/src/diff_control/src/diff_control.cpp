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

#include <csignal>  // to catch the SIGINT signal - ctrl+c

#include "diff_controller.h"

bool signal_SIGINT = false;

/*
 * Function to Handle the use of Ctrl+c to terminate the program 
 */
void signalHandler(int signum){
        std::cout << "\n\n\nSignal " << signum << " received. Shutting down the Differential Controller Node...\n" << std::endl;

        signal_SIGINT = true;
    }

int main(int argc, char * argv[])
{
    // register SIGINT signal and signal handler
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "diff_control", ros::init_options::NoSigintHandler);
    diff_controller controller = diff_controller();

    while (ros::ok())
    {
        ros::spinOnce();

        if (signal_SIGINT)
        {
            std::cout << "Deleting the controller object..." << std::endl;
            delete &controller;
            break;
        }
    }

    std::cout << "diff_control node has been shut down." << std::endl;
    return 0;
}
