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
#include <math.h>
#include <string>

/* ROS */
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

/* tf is needed to convert quaternions to Euler and vice versa */
#include <tf.h>


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
        // Get and Set the Parameters from launch file
        priv_nh_.param<std::string>("vrpn_topic", vrpn_topic_, "vrpn_client_node/twist");
        priv_nh_.param<std::string>("pose_topic", pose_topic_, "slam_toolbox/pose");
        priv_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
        priv_nh_.param<double>("a", a_, 0.1);
        priv_nh_.param<double>("Kx", Kx_, 1.0);
        priv_nh_.param<double>("Ky", Ky_, 1.0);



        /*
         * Subscribers
         */
        // subscribe to the vrpn twist topic
        vrpn_sub_ = nh_.subscribe(vrpn_topic_, 1, &diff_controller::vrpnCallback, this);
        // subscribe to the pose topic from the slam_toolbox
        pose_sub_ = nh_.subscribe(pose_topic_, 1, &diff_controller::slam_poseCallback, this);

        /*
        * Publishers
        */
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    }

    /****************************************************************************
    ** vrpn Twist Message Callback *
    *  @param msg: vrpn twist message
    *
    *  This function is called whenever a new vrpn twist message is received.
    *  Gets information of the robot position from the optitrack system.
    *  Used as reference to compare with the results from the slam_toolbox.
     ****************************************************************************/
    void vrpnCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        ROS_INFO("\nGetting Twist Message from vrpn_client_ros...");

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

    /****************************************************************************************
    ** slam_toolbox Pose Message Callback
    *  @param msg: pose message
    *
    *  This function is called whenever a new pose message is received.
    *  Gets information of the robot position from the slam_toolbox.
    *  Used to calculate the velocity commands to move the robot to the desired position.
    */
    void slam_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        ROS_INFO("\nGetting Pose Message from slam_toolbox...");

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
    void calculateVelocities()
    {
        
    }

    /**----------------------------------------------
     **              calculateControlSignals
     *?  Calculates the control signals to move the robot to the desired position.
     * @param last_position geometry_msgs::Pose
     * @param desired_position geometry_msgs::Pose
     * @return void
     *---------------------------------------------**/
    void calculateControlSignals(geometry_msgs::Pose last_position, geometry_msgs::Pose desired_position)
    {
        double x = last_position.position.x;
        double y = last_position.position.y;
        double theta = last_position.orientation.z;

        double x_d = desired_position.position.x;
        double y_d = desired_position.position.y;
        double theta_d = desired_position.orientation.z;

        // Erro de posicao
        double x_error = x_d - x;
        double y_error = y_d - y;

        // Velocidades desejadas
        double Vxd = 1.0;
        double Vyd = 1.0;

        // Calculo dos sinais de controle - velocidade linear e angular no eixo do robo
        double u = cos(theta) * (Vxd + this->Kx_*x_error) + sin(theta) *(Vyd + this->Ky_*y_error);
        double w = (1/this->a_)*(-sin(theta)*(Vxd + this->Kx_*x_error) + cos(theta)*(Vyd + this->Ky_*y_error));

        cmd_vel_.linear.x = u;
        cmd_vel_.angular.z = w;

        // add the message to an array of all messages
        cmd_vel_list_.push_back(cmd_vel_);

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

    /*
     * Parameters
     */
        std::string vrpn_topic_;
        std::string pose_topic_;
        std::string cmd_vel_topic_;

        // Controller Parameters
        double a_;
        double Kx_;
        double Ky_;


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
