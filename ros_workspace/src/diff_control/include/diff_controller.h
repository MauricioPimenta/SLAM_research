/**------------------------------------------------------------------------------------------------
 * *                                           diff_controller.h
 *
 * This file implements a controller for a differential drive robot.
 *
 * It receives the desired position and the actual position of the robot and sends velocity commands
 * to a 'cmd_vel' ROS topic to make the robot move to the desired position.
 *
 *
 ** Subscribes to:
 *      * vrpn twist topic from the vrpn_client_ros package (name defined by the launch file)
 *      * pose topic from the slam_toolbox: pose of the base_frame along with the covariance
 **       calculated from the scan match
 *
 *
 ** Publishes to: cmd_vel topic
 *
 *------------------------------------------------------------------------------------------------**/

/* System Includes */
#include <iostream>
#include <vector>

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>



class diff_controller
{
public:
    /*
     * Public Atributes and Properties
     */

    /*
     * Public Methods
     */

    // Constructor
    diff_controller() : nh_("controller"), priv_nh_("~")
    {
        /*
         * Subscribers
         */
        // subscribe to the vrpn twist topic
        vrpn_sub_ = nh_.subscribe("vrpn_client_node/twist", 1, &diff_controller::vrpnCallback, this);
        // subscribe to the pose topic from the slam_toolbox
        pose_sub_ = nh_.subscribe("slam_toolbox/pose", 1, &diff_controller::poseCallback, this);

        /*
        * Publishers *
        */
        // publish to the cmd_vel topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    /*
     * vrpn Twist Message Callback
     * @param msg: vrpn twist message
     *
     * This function is called whenever a new vrpn twist message is received.
     * Gets information of the robot position from the optitrack system.
     * Used as reference to compare with the results from the slam_toolbox.
     */
    void vrpnCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        // save the message
        vrpn_twist_.linear.x = msg->linear.x;
        vrpn_twist_.linear.y = msg->linear.y;
        vrpn_twist_.linear.z = msg->linear.z;
        vrpn_twist_.angular.x = msg->angular.x;
        vrpn_twist_.angular.y = msg->angular.y;
        vrpn_twist_.angular.z = msg->angular.z;

        // add the message to an array of all messages
        vrpn_twist_list_.push_back(vrpn_twist_);
    }

    /*
     * slam_toolbox Pose Message Callback
     * @param msg: pose message
     *
     * This function is called whenever a new pose message is received.
     * Gets information of the robot position from the slam_toolbox.
     * Used to calculate the velocity commands to move the robot to the desired position.
     */
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // save the message
        slam_pose_.header.seq = msg->header.seq;
        slam_pose_.header.stamp = msg->header.stamp;
        slam_pose_.header.frame_id = msg->header.frame_id;
        slam_pose_.pose.pose.position.x = msg->pose.pose.position.x;
        slam_pose_.pose.pose.position.y = msg->pose.pose.position.y;
        slam_pose_.pose.pose.position.z = msg->pose.pose.position.z;
        slam_pose_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        slam_pose_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        slam_pose_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        slam_pose_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        slam_pose_.pose.covariance = msg->pose.covariance;

        // add the message to an array of all messages
        slam_pose_list_.push_back(slam_pose_);

        // calculate the velocity commands
        calculateVelocities();

    }

    /**----------------------------------------------
     **              calculateVelocities
     *?  Calculates the velocity commands to move the robot to the desired position.
     * @param last_position type  
     * @param desired_position type  
     * @return cmd_vel type
     *---------------------------------------------**/
    geometry_msgs::Twist calculateVelocities()
    {

    }

private:
    /*
     * Private Atributes and Properties
     */
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Subscriber vrpn_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher cmd_vel_pub_;

    geometry_msgs::Twist vrpn_twist_;
    geometry_msgs::PoseWithCovarianceStamped slam_pose_;
    geometry_msgs::Twist cmd_vel_;

    // Arrays and Lists
    std::vector<geometry_msgs::Twist> vrpn_twist_list_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> slam_pose_list_;
    std::vector<geometry_msgs::Twist> cmd_vel_list_;

    /*
     * Private Methods
     */

};
