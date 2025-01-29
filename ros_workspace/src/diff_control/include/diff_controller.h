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
 *      * vrpn Pose topic from the vrpn_client_ros package (name defined by the launch file)
 *      * pose topic from the slam_toolbox: pose of the base_frame along with the covariance
 **       calculated from the scan match
 *
 *
 ** Publishes to: cmd_vel topic
 *
 *------------------------------------------------------------------------------------------------**/

/* System Includes */
#include <iostream>
#include <fstream>  // to use files
#include <vector>
#include <math.h>
#include <string>


/* ROS */
#include <ros/ros.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <turtlesim/Pose.h>

/* tf is needed to convert quaternions to Euler and vice versa */
#include <tf/tf.h>
#include <tf2/convert.h>


class diff_controller
{
    private:
    /*
     * Private Atributes and Properties
     */
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Subscriber vrpn_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_vel_pub_;

    ros::Subscriber turtle_sub_;

    /*
     * Parameters
     */
        std::string vrpn_topic_;
        std::string pose_topic_;
        std::string cmd_vel_topic_;
        std::string goal_topic_;

        std::string turtle_topic_;

        // Controller Parameters
        double a_;
        double Kx_;
        double Ky_;


    geometry_msgs::PoseStamped vrpn_twist_;
    geometry_msgs::PoseWithCovarianceStamped slam_pose_;
    geometry_msgs::Twist cmd_vel_;
    geometry_msgs::Pose goal_;

    turtlesim::Pose turtle_pose_;

    // Arrays and Lists
    std::vector<geometry_msgs::PoseStamped> vrpn_twist_list_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> slam_pose_list_;
    std::vector<geometry_msgs::Twist> cmd_vel_list_;
    std::vector<geometry_msgs::Pose> goal_list_;

    std::vector<turtlesim::Pose> turtle_pose_list_;

    /*
     * Private Methods
     */
    



public:
    /*
     * Public Atributes and Properties
     */

    /*
     * Public Methods
     */

    // Constructor
    diff_controller() : nh_(""), priv_nh_("~")
    {
        

        // Get the Parameters from the launch file and set default values
        priv_nh_.param<std::string>("vrpn_topic", vrpn_topic_, "/vrpn_client_node/L1/pose");
        priv_nh_.param<std::string>("pose_topic", pose_topic_, "/slam_toolbox/pose");
        priv_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        priv_nh_.param<std::string>("goal_topic", goal_topic_, "/goal");
        priv_nh_.param<double>("a", a_, 0.1);
        priv_nh_.param<double>("Kx", Kx_, 1.0);
        priv_nh_.param<double>("Ky", Ky_, 1.0);

        // priv_nh_.param<std::string>("turtle_topic", turtle_topic_, "/turtle1/pose");



        /*
         * Subscribers
         */
        // subscribe to the vrpn twist topic
        vrpn_sub_ = nh_.subscribe(vrpn_topic_, 1, &diff_controller::vrpnCallback, this);
        // subscribe to the pose topic from the slam_toolbox
        pose_sub_ = nh_.subscribe(pose_topic_, 1, &diff_controller::slam_poseCallback, this);
        // subscribe to the goal topic
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &diff_controller::goalCallback, this);

        // turtle_sub_ = nh_.subscribe(turtle_topic_, 1, &diff_controller::turtleCallback, this);

        /*
        * Publishers
        */
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    }

    

    // void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
    // {
    //     ROS_DEBUG("\nGetting Turtle Pose Message from %s...", turtle_topic_.c_str());

    //     // save the message
    //     turtle_pose_.x = msg->x;
    //     turtle_pose_.y = msg->y;
    //     turtle_pose_.theta = msg->theta;
    //     turtle_pose_.linear_velocity = msg->linear_velocity;
    //     turtle_pose_.angular_velocity = msg->angular_velocity;

    //     // add the message to an array of all messages
    //     turtle_pose_list_.push_back(turtle_pose_);
    // }

    /****************************************************************************
    ** vrpn Twist Message Callback *
    *  @param msg: vrpn twist message
    *
    *  This function is called whenever a new vrpn twist message is received.
    *  Gets information of the robot position from the optitrack system.
    *  Used as reference to compare with the results from the slam_toolbox.
     ****************************************************************************/
    void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Twist Message from %s...", vrpn_topic_.c_str());

        // save the message
        vrpn_twist_.pose.position.x = msg->pose.position.x;
        vrpn_twist_.pose.position.y = msg->pose.position.y;
        vrpn_twist_.pose.position.z = msg->pose.position.z;
        vrpn_twist_.pose.orientation.x = msg->pose.orientation.x;
        vrpn_twist_.pose.orientation.y = msg->pose.orientation.y;
        vrpn_twist_.pose.orientation.z = msg->pose.orientation.z;
        vrpn_twist_.pose.orientation.w = msg->pose.orientation.w;

        // add the message to an array of all messages
        vrpn_twist_list_.push_back(vrpn_twist_);

        this->calculateControlSignals(vrpn_twist_.pose, goal_);
    }

    /****************************************************************************************
    ** slam_toolbox Pose Message Callback
    *  @param msg: pose message
    *
    *?  This function is called whenever a new pose message is received.
    *?  Gets information of the robot position from the slam_toolbox.
    *?  Used to calculate the velocity commands to move the robot to the desired position.
    */
    void slam_poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Pose Message from %s...", pose_topic_.c_str());

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
        this->calculateControlSignals(slam_pose_.pose.pose, goal_);

    }

    /****************************************************************************************
     ** Goal Message Callback
     * @param msg: goal message
     * 
     *? This function is called whenever a new goal message is received.
     *? Gets the desired position of the robot.
     */
    void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Goal Message from %s...", goal_topic_.c_str());

        // save the message
        goal_.position.x = msg->position.x;
        goal_.position.y = msg->position.y;
        goal_.position.z = msg->position.z;
        goal_.orientation.x = msg->orientation.x;
        goal_.orientation.y = msg->orientation.y;
        goal_.orientation.z = msg->orientation.z;
        goal_.orientation.w = msg->orientation.w;

        // add the message to an array of all messages
        goal_list_.push_back(goal_);

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

        // Se a posicao desejada for alcancada
        if (x_error < 0.01 && y_error < 0.01)
        {
            // zera os comandos de velocidade
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel_);
            return;
        }

        // Velocidades desejadas
        double Vxd = 1.0;
        double Vyd = 1.0;

        // Calculo dos sinais de controle - velocidade linear e angular no eixo do robo
        double u = cos(theta) * (Vxd + this->Kx_*x_error) + sin(theta) *(Vyd + this->Ky_*y_error);
        double w = (1/this->a_)*(-sin(theta)*(Vxd + this->Kx_*x_error) + cos(theta)*(Vyd + this->Ky_*y_error));

        cmd_vel_.linear.x = u;
        cmd_vel_.angular.z = w;

        // publish the velocity commands
        cmd_vel_pub_.publish(cmd_vel_);

        // add the message to an array of all messages
        cmd_vel_list_.push_back(cmd_vel_);

    }

    // Destructor
    ~diff_controller()
    {
        std::cout << "\n\n\nShutting down the Differential Controller Node...\n\n\n" << std::endl;

        std::cout << "Stopping Robot...\n\n" << std::endl;
        // send a message to stop the robot
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel_);

        // save a file with all messages received and published and their timestamps
        this->saveMessagesToFile();
    }


    void saveMessagesToFile(){

        std::cout << "\n\n\nSaving Messages to a File...\n\n\n" << std::endl;
        // save the messages to a file
        std::string file_name = "../Documents/SLAM_research/messages.txt";
        std::ofstream file(file_name);

        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << file_name << std::endl;
            return;
        }

        // Header
        file << "Messages Received and Published by the Differential Controller\n\n";
        file << "===============================================\n\n";

        // save the vrpn messages
        file << "VRPN Messages: \n";
        for (int i = 0; i < vrpn_twist_list_.size(); i++)
        {
            file << "Message " << i << ": \n";
            file << "Position: (" << vrpn_twist_list_[i].pose.position.x << ", " << vrpn_twist_list_[i].pose.position.y << ", " << vrpn_twist_list_[i].pose.position.z << ")\n";
            file << "Orientation: (" << vrpn_twist_list_[i].pose.orientation.x << ", " << vrpn_twist_list_[i].pose.orientation.y << ", " << vrpn_twist_list_[i].pose.orientation.z << ", " << vrpn_twist_list_[i].pose.orientation.w << ")\n";
        }

        file << "\n\n===============================================\n\n";

        // save the slam messages
        file << "SLAM Messages: \n";
        for (int i = 0; i < slam_pose_list_.size(); i++)
        {
            file << "Message " << i << ": \n";
            file << "Position: (" << slam_pose_list_[i].pose.pose.position.x << ", " << slam_pose_list_[i].pose.pose.position.y << ", " << slam_pose_list_[i].pose.pose.position.z << ")\n";
            file << "Orientation: (" << slam_pose_list_[i].pose.pose.orientation.x << ", " << slam_pose_list_[i].pose.pose.orientation.y << ", " << slam_pose_list_[i].pose.pose.orientation.z << ", " << slam_pose_list_[i].pose.pose.orientation.w << ")\n";
        }

        file << "\n\n===============================================\n\n";

        // save the goal messages
        file << "Goal Messages: \n";
        for (int i = 0; i < goal_list_.size(); i++)
        {
            file << "Message " << i << ": \n";
            file << "Position: (" << goal_list_[i].position.x << ", " << goal_list_[i].position.y << ", " << goal_list_[i].position.z << ")\n";
            file << "Orientation: (" << goal_list_[i].orientation.x << ", " << goal_list_[i].orientation.y << ", " << goal_list_[i].orientation.z << ", " << goal_list_[i].orientation.w << ")\n";
        }

        file << "\n\n===============================================\n\n";

        // save the cmd_vel messages
        file << "CMD_VEL Messages: \n";
        for (int i = 0; i < cmd_vel_list_.size(); i++)
        {
            file << "Message " << i << ": \n";
            file << "Linear Velocity: " << cmd_vel_list_[i].linear.x << "\n";
            file << "Angular Velocity: " << cmd_vel_list_[i].angular.z << "\n";
        }

        // close and save file
        file.close();

        std::cout << "\n\n\nMessages Saved to File: " << file_name << "\n\n\n" << std::endl;

    }

};
