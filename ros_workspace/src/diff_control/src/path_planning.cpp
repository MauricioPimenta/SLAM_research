#include <iostream>
#include <ros/ros.h>

#include <csignal>  // to catch the SIGINT signal - ctrl+c

#include "path_planner.h"

bool g_signal_SIGINT = false;

/*
 * Function to Handle the use of Ctrl+c to terminate the program
 */
void signalHandler(int signum){
        std::cout << "\n\nSignal " << signum << " received. Shutting down the Path Planner Node...\n" << std::endl;

        g_signal_SIGINT = true;
    }

int main(int argc, char * argv[])
{
    // register SIGINT signal and signal handler
    signal(SIGINT, signalHandler);

    // Initialize the ROS node without the ctrl+c handler so we can use our own
    ros::init(argc, argv, "path_planner", ros::init_options::NoSigintHandler);

    PathPlanner path_planner_generator = PathPlanner();

    while (ros::ok())
    {
        ros::spinOnce();

        // if ctrl+c is pressed, delete the controller object and break the loop
        if (g_signal_SIGINT)
        {
            std::cout << "Deleting the PathPlanner object..." << std::endl;
            delete &path_planner_generator;
            ros::shutdown();
            break;
        }
    }

    std::cout << "path_planner node has been shut down." << std::endl;
    return 0;
}