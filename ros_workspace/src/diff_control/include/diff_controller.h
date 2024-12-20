/**------------------------------------------------------------------------------------------------
 * *                                           diff_controller.h
 *
 * This file implements a controller for a differential drive robot.
 *
 * It receives the desired position and the actual position of the robot and sends velocity commands
 * to a 'cmd_vel' ROS topic to make the robot move to the desired position.
 *
 *
 *
 *------------------------------------------------------------------------------------------------**/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class diff_controller
{
public:
    diff_controller() : nh_(""), priv_nh_("~")
    {

    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

};
