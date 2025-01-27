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

#include "diff_controller.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "diff_control");
    diff_controller controller = diff_controller();

    ros::spin();

    delete &controller;
    // controller.~diff_controller();

    ros::shutdown();

    std::cout << "diff_control node has been shut down." << std::endl;
    return 0;
}
