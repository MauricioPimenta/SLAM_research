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
#include <cmath>
#include <string>

#include <filesystem> // to use filesystem

#include <ctime> // to use time

#include <optional> // to use optional type

/* ROS */
#include <ros/ros.h>


#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/ModelStates.h>

#include <turtlesim/Pose.h>

/* tf is needed to convert quaternions to Euler and vice versa */
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>   // tf2 library for Matrix3x3
#include <tf2/LinearMath/Quaternion.h>  // tf2 library for Quaternions
#include <tf2/LinearMath/Transform.h>   // tf2 library for Transforms

#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // to convert between tf2 and geometry_msgs


class DiffController
{
    /*
     * Private Atributes and Properties
     */
private:

    /*
     * Node Parameters
     */

    std::string vrpn_topic_;        // topic name for the vrpn twist messages
    std::string pose_topic_;        // topic name for the pose messages from the slam_toolbox

    std::string cmd_vel_topic_;     // topic name for the velocity commands
    std::string vrpn_cmd_vel_topic_;    // topic name for the velocity commands for the vrpn
    std::string slam_cmd_vel_topic_;    // topic name for the velocity commands for the slam_toolbox

    std::string goal_topic_;        // topic name for the desired position
    std::string gazebo_topic_;      // topic name for the gazebo model states

    bool use_turtle_sim_;           // flag to use the turtle simulator
    std::string turtle_topic_;      // topic name for the turtle pose messages

    /*
     * Controller Parameters
     */

    // offset from the robot center that defines the point for control
    //   If a_ = 0, then the control point is the center of the robot
    double a_;

    // gains for the controller
    double Kx_;     // gain applied to the x_error
    double Ky_;    // gain applied to the y_error

    // Desired Velocities
    // For positioning control, these should be zero.
    // For velocity control, these should be the desired velocities of the trajectory
    double x_desired_velocity_;
    double y_desired_velocity_;

    double max_linear_velocity_;    // The desired maximum linear velocity the robot should perform
    double max_angular_velocity_;   // The desired maximum angular velocity the robot shold perform

    // The frequency to publish the control signals.
    double control_frequency_;

    /*
     * ROS Node Handlers, Publishers and Subscribers
     */
    ros::NodeHandle nh_;        // public node handler
    ros::NodeHandle priv_nh_;   // private node handler
    ros::Subscriber vrpn_sub_;  // subscriber to the vrpn twist topic
    ros::Subscriber pose_sub_;  // subscriber to the pose topic from the slam_toolbox
    ros::Subscriber goal_sub_;  // subscriber to the goal topic
    ros::Subscriber gazebo_sub_;    // subscriber to the gazebo model states

    ros::Publisher cmd_vel_pub_;    // publisher for the velocity commands

    ros::Publisher vrpn_cmd_vel_pub_;   // publisher for the velocity commands for the vrpn
    ros::Publisher slam_cmd_vel_pub_;   // publisher for the velocity commands for the slam_toolbox

    ros::Subscriber turtle_sub_;        // subscriber to the turtle pose topic

    ros::Timer slam_control_loop_timer_;  // timer for the control loop for the slam_toolbox cmd_vel
    ros::Timer vrpn_control_loop_timer_;  // timer for the control loop that publishes the vrpn_cmd_vel
    ros::Timer control_loop_timer_;       // timer for the control loop that publish in cmd_vel

    /*
     * ROS Messages
     */
    geometry_msgs::PoseStamped vrpn_twist_;     // vrpn twist message
    geometry_msgs::TwistStamped vrpn_cmd_vel_;         // vrpn velocity command

    geometry_msgs::PoseWithCovarianceStamped slam_pose_;    // pose message from the slam_toolbox
    geometry_msgs::TwistStamped slam_cmd_vel_;                     // slam_toolbox velocity command

    geometry_msgs::TwistStamped cmd_vel_;    // message use for the velocity command published in cmd_vel
    std::optional<geometry_msgs::PoseStamped> goal_;    // goal message type - this is optional because the goal may not be received

    turtlesim::Pose turtle_pose_;   // turtle pose message

    // Gazebo Messages
    gazebo_msgs::ModelStates gazebo_Models_msg_;    // gazebo ModelStates message - to get the positions from the models inside the gazebo
    geometry_msgs::Pose limo_gazebo_pose_;          // Pose message to save the robot position from gazebo


    /*
     * Arrays and Lists
     */
    std::vector<geometry_msgs::PoseStamped> vrpn_twist_list_;       // list to record all messages received from the vrpn topic
    std::vector<geometry_msgs::TwistStamped> vrpn_cmd_vel_list_;           // list to record all velocity commands generated using the vrpn poses to calculate the control

    std::vector<geometry_msgs::PoseWithCovarianceStamped> slam_pose_list_;  // list to record all messages received from the slam_toolbox topic
    std::vector<geometry_msgs::TwistStamped> slam_cmd_vel_list_;                   // list to record all velocity commands generated using the slam_toolbox poses to calculate the control

    std::vector<geometry_msgs::TwistStamped> cmd_vel_list_;        // list to record all velocity commands published in the cmd_vel topic
    std::vector<geometry_msgs::PoseStamped> goal_list_;     // list to record all goal messages received

    std::vector<turtlesim::Pose> turtle_pose_list_;         // list to record all turtle pose messages received

    std::vector<geometry_msgs::Pose> limo_gazebo_poses_list_;   // list to record all robot positions received from gazebo

    /*
     * Private Methods
     */
    // nothing here

    /*
     * Synchronization
     */
    std::mutex mutex_; // Mutex for thread safety

public:
    /*
     * Public Atributes and Properties
     */

    /*
     * Public Methods
     */

    // Constructor
    DiffController() : nh_(""), priv_nh_("~")
    {
        // Get the Parameters from the launch file and set default values
        priv_nh_.param<std::string>("vrpn_topic", vrpn_topic_, "vrpn_client_node/L1/pose");
        priv_nh_.param<std::string>("pose_topic", pose_topic_, "slam_toolbox/pose");
        priv_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
        priv_nh_.param<std::string>("vrpn_cmd_vel_topic", vrpn_cmd_vel_topic_, "vrpn_cmd_vel");
        priv_nh_.param<std::string>("slam_cmd_vel_topic", slam_cmd_vel_topic_, "slam_cmd_vel");
        priv_nh_.param<std::string>("goal_topic", goal_topic_, "goal");
        // gazebo topic param
        priv_nh_.param<std::string>("gazebo_topic", gazebo_topic_, "gazebo/model_states");

        priv_nh_.param<double>("a", a_, 0.1);
        priv_nh_.param<double>("Kx", Kx_, 1.0);
        priv_nh_.param<double>("Ky", Ky_, 1.0);
        priv_nh_.param<double>("control_frequency", control_frequency_, 10.0);
        priv_nh_.param<double>("max_linear_velocity", max_linear_velocity_, 1.0);
        priv_nh_.param<double>("max_angular_velocity", max_angular_velocity_, 1.5);
        priv_nh_.param<double>("x_desired_velocity", x_desired_velocity_, 0.0);
        priv_nh_.param<double>("y_desired_velocity", y_desired_velocity_, 0.0);

        priv_nh_.param<bool>("use_turtle_sim", use_turtle_sim_, false);
        priv_nh_.param<std::string>("turtle_topic", turtle_topic_, "turtle1/pose");

        /*
         * Subscribers
         */
        // subscribe to the vrpn twist topic
        vrpn_sub_ = nh_.subscribe(vrpn_topic_, 1, &DiffController::vrpnCallback, this);
        // subscribe to the pose topic from the slam_toolbox
        pose_sub_ = nh_.subscribe(pose_topic_, 1, &DiffController::slam_poseCallback, this);
        // subscribe to the goal topic
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &DiffController::goalCallback, this);

        gazebo_sub_ = nh_.subscribe(gazebo_topic_, 1, &DiffController::gazeboCallback, this);

        if (use_turtle_sim_)
        {
            // create subscriber to turtlesim pose topic
            turtle_sub_ = nh_.subscribe(turtle_topic_, 1, &DiffController::turtleCallback, this);
        }

        /*
        * Publishers
        */
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        vrpn_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vrpn_cmd_vel_topic_, 1);
        slam_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(slam_cmd_vel_topic_, 1);

        // Start the timer that call the control loop
        // the callback in this timer should publish the control signal (cmd_vel) for vrpn and slam_toolbox
        ros::Duration control_interval = ros::Duration(1.0 / control_frequency_);
        vrpn_control_loop_timer_ = nh_.createTimer(ros::Duration(control_interval), &DiffController::vrpn_controlLoop, this);
        slam_control_loop_timer_ = nh_.createTimer(ros::Duration(control_interval), &DiffController::slam_controlLoop, this);
        control_loop_timer_ = nh_.createTimer(ros::Duration(control_interval), &DiffController::publishControlSignals, this);
    }


    /****************************************************************************************
     * gazebo ModelStates Message Callback
     * @param msg: gazebo ModelStates message
     * 
     * This function is called whenever a new gazebo ModelStates message is received.
     * Gets information of the robot position from the gazebo.
     * Used to simulate the robot behaviour using the gazebo.
     */
    void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Gazebo ModelStates Message from %s...", gazebo_topic_.c_str());

        // save the message
        gazebo_Models_msg_ = *msg;

        limo_gazebo_pose_ = gazebo_Models_msg_.pose[2];

        // add the message to an array of all messages
        // gazebo_Models_poses_list_.push_back(gazebo_Models_poses_);

        // check if the desired position was received
        if (!goal_)
        {
            ROS_INFO("\nDesired Position not received yet...\n");
            return;
        }

        // calculate the velocity commands
        this->calculateControlSignals(limo_gazebo_pose_, goal_.value().pose, &cmd_vel_);

        ROS_INFO("\n\n****** Gazebo Control Signals: ******\n");
        ROS_INFO("Linear Velocity: %.4f\n", cmd_vel_.twist.linear.x);
        ROS_INFO("Angular Velocity: %.4f\n", cmd_vel_.twist.angular.z);
    }


    /****************************************************************************
    ** turtlesim Pose Message Callback *
    *  @param msg: turtlesim::Pose message
    *
    *  This function is called whenever a new turtlesim::Pose message is received.
    *  Gets information of the robot position from the turtlesim.
    *  Used to simulate the robot behaviour using the turtlesim.
    */
    void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Turtle Pose Message from %s...", turtle_topic_.c_str());

        // save the message
        turtle_pose_.x = msg->x;
        turtle_pose_.y = msg->y;
        turtle_pose_.theta = msg->theta;
        turtle_pose_.linear_velocity = msg->linear_velocity;
        turtle_pose_.angular_velocity = msg->angular_velocity;

        // add the message to an array of all messages
        turtle_pose_list_.push_back(turtle_pose_);
    }

    /****************************************************************************
    ** vrpn Twist Message Callback *
    *  @param msg: vrpn twist message
    *
    *  This function is called whenever a new vrpn twist message is received.
    *  Gets information of the robot position from the optitrack system.
    *  Used as reference to compare with the results from the slam_toolbox.
    */
    void vrpnCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Twist Message from %s...", vrpn_topic_.c_str());

        // save the message
        vrpn_twist_.header.seq = msg->header.seq;
        vrpn_twist_.header.stamp = msg->header.stamp;
        vrpn_twist_.header.frame_id = msg->header.frame_id;

        vrpn_twist_.pose.position.x = msg->pose.position.x;
        vrpn_twist_.pose.position.y = msg->pose.position.y;
        vrpn_twist_.pose.position.z = msg->pose.position.z;

        vrpn_twist_.pose.orientation.x = msg->pose.orientation.x;
        vrpn_twist_.pose.orientation.y = msg->pose.orientation.y;
        vrpn_twist_.pose.orientation.z = msg->pose.orientation.z;
        vrpn_twist_.pose.orientation.w = msg->pose.orientation.w;

        // add the message to an array of all messages
        vrpn_twist_list_.push_back(vrpn_twist_);

        
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


    }

    /****************************************************************************************
     ** Goal Message Callback
     * @param msg: goal message
     * 
     *? This function is called whenever a new goal message is received.
     *? Gets the desired position of the robot.
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_DEBUG("\nGetting Goal Message from %s...", goal_topic_.c_str());

        // save the message
        goal_ = *msg;
        // goal_.position.x = msg->position.x;
        // goal_.position.y = msg->position.y;
        // goal_.position.z = msg->position.z;
        // goal_.orientation.x = msg->orientation.x;
        // goal_.orientation.y = msg->orientation.y;
        // goal_.orientation.z = msg->orientation.z;
        // goal_.orientation.w = msg->orientation.w;

        // add the message to an array of all messages
        goal_list_.push_back(goal_.value());

    }

    /**----------------------------------------------
     **              calculateControlSignals
     *?  Calculates the control signals to move the robot to the desired position.
     * @param last_position geometry_msgs::Pose
     * @param desired_position geometry_msgs::Pose
     * @param cmd_vel geometry_msgs::Twist::Ptr - cmd_vel message to save the control signals
     * @return void
     *---------------------------------------------**/
    void calculateControlSignals(geometry_msgs::Pose last_position, geometry_msgs::Pose desired_position, geometry_msgs::TwistStamped *cmd_vel)
    {

        // Convert the orientation from quaternion to Euler angles using tf2
        tf2::Quaternion last_pos_quat;
        tf2::fromMsg(last_position.orientation, last_pos_quat);
        tf2::Matrix3x3 last_position_rotation_matrix(last_pos_quat);

        // Get the Euler angles
        double pose_roll, pose_pitch, pose_yaw;
        last_position_rotation_matrix.getRPY(pose_roll, pose_pitch, pose_yaw);

        // Convert the desired orientation from quaternion to Euler angles
        tf2::Quaternion des_pos_quat;
        tf2::fromMsg(desired_position.orientation, des_pos_quat);
        tf2::Matrix3x3 desired_position_rotation_matrix(des_pos_quat);

        // Get the Euler angles
        double des_roll, des_pitch, des_yaw;
        desired_position_rotation_matrix.getRPY(des_roll, des_pitch, des_yaw);


        // Print the rotation matrices
        ROS_INFO("\n\nMatrix last_position_rotation_matrix: \n");
        ROS_INFO("| %.4f  %.4f  %.4f | \n", last_position_rotation_matrix[0][0], last_position_rotation_matrix[0][1], last_position_rotation_matrix[0][2]);
        ROS_INFO("| %.4f  %.4f  %.4f | \n", last_position_rotation_matrix[1][0], last_position_rotation_matrix[1][1], last_position_rotation_matrix[1][2]);
        ROS_INFO("| %.4f  %.4f  %.4f | \n", last_position_rotation_matrix[2][0], last_position_rotation_matrix[2][1], last_position_rotation_matrix[2][2]);
        ROS_INFO("\npose_roll: %.4f, pose_pitch: %.4f, pose_yaw: %.4f\n", pose_roll, pose_pitch, pose_yaw);

        ROS_INFO("\n\nMatrix desired_position_rotation_matrix: \n");
        ROS_INFO("| %.4f  %.4f  %.4f | \n", desired_position_rotation_matrix[0][0], desired_position_rotation_matrix[0][1], desired_position_rotation_matrix[0][2]);
        ROS_INFO("| %.4f  %.4f  %.4f | \n", desired_position_rotation_matrix[1][0], desired_position_rotation_matrix[1][1], desired_position_rotation_matrix[1][2]);
        ROS_INFO("| %.4f  %.4f  %.4f | \n", desired_position_rotation_matrix[2][0], desired_position_rotation_matrix[2][1], desired_position_rotation_matrix[2][2]);
        ROS_INFO("\ndes_roll: %.4f, des_pitch: %.4f, des_yaw: %.4f\n", des_roll, des_pitch, des_yaw);


        double x = last_position.position.x;
        double y = last_position.position.y;
        double theta = pose_yaw;

        double x_d = desired_position.position.x;
        double y_d = desired_position.position.y;
        double theta_d = des_yaw;

        // Erro de posicao
        double x_error = x_d - x;
        double y_error = y_d - y;

        ROS_WARN("\n\n x: %.4f, y: %.4f, theta: %.4f\n", x, y, theta);
        ROS_WARN("x_d: %.4f, y_d: %.4f, theta_d: %.4f\n", x_d, y_d, theta_d);
        ROS_WARN("x_error: %.4f, y_error: %.4f\n", x_error, y_error);

        // // Se a posicao desejada for alcancada
        // if (abs(x_error) < 0.1 && abs(y_error) < 0.1)
        // {
        //     // zera os comandos de velocidade
        //     cmd_vel->linear.x = 0.0;
        //     cmd_vel->angular.z = 0.0;
        //     return;
        // }

        // Velocidades desejadas - should be zero for positioning
        double Vxd = x_desired_velocity_;
        double Vyd = y_desired_velocity_;

        // Velocidades máximas
        double vmax = max_linear_velocity_;
        double wmax = max_angular_velocity_;

        // Calculo dos sinais de controle - velocidade linear e angular no eixo do robo
        double u = cos(theta) * (Vxd + this->Kx_*x_error) + sin(theta) *(Vyd + this->Ky_*y_error);
        double w = (1/this->a_)*(-sin(theta)*(Vxd + this->Kx_*x_error) + cos(theta)*(Vyd + this->Ky_*y_error));

        ROS_WARN("\n\n\nControl Signals Calculated...\n");
        ROS_WARN("Linear Velocity: %.4f\n", u);
        ROS_WARN("Angular Velocity: %.4f\n", w);

        // Set the velocity commands
        cmd_vel->twist.linear.x = u;
        cmd_vel->twist.angular.z = w;

        // Uses tanh to limit the values of the velocities
        cmd_vel->twist.linear.x = vmax*tanh(u);
        cmd_vel->twist.angular.z = wmax*tanh(w);


        ROS_ERROR("\n\n\nControl Signals Saved...\n");
        ROS_ERROR("Linear Velocity: %.4f\n", cmd_vel->twist.linear.x);
        ROS_ERROR("Angular Velocity: %.4f\n", cmd_vel->twist.angular.z);


    }

    void publishControlSignals(const ros::TimerEvent&)
    {
        ROS_INFO("\nControl Loop Timer Callback...\n");

        // publish the velocity commands
        cmd_vel_pub_.publish(cmd_vel_);

        // add the message to an array of all messages
        cmd_vel_list_.push_back(cmd_vel_);
    }

    void vrpn_controlLoop(const ros::TimerEvent&)
    {
        ROS_INFO("\n VRPN Control Loop Timer Callback...\n");

        // check if the desired position was received
        if (!goal_)
        {
            ROS_INFO("\nDesired Position not received yet...\n");
            return;
        }

        // calculate the velocity commands
        this->calculateControlSignals(vrpn_twist_.pose, goal_.value().pose, &vrpn_cmd_vel_);

        ROS_INFO("\n\n****** VRPN Control Signals: ******\n");
        ROS_INFO("Linear Velocity: %.4f\n", vrpn_cmd_vel_.twist.linear.x);
        ROS_INFO("Angular Velocity: %.4f\n", vrpn_cmd_vel_.twist.angular.z);

        // publish the velocity commands
        vrpn_cmd_vel_pub_.publish(vrpn_cmd_vel_);

        // add the message to an array of all messages
        vrpn_cmd_vel_list_.push_back(vrpn_cmd_vel_);
    }

    void slam_controlLoop(const ros::TimerEvent&)
    {

        ROS_INFO("\nSLAM Control Loop Timer Callback...\n");

        // check if the desired position was received
        if (!goal_)
        {
            ROS_INFO("\nDesired Position not received yet...\n");
            return;
        }

        // calculate the velocity commands
        this->calculateControlSignals(slam_pose_.pose.pose, goal_.value().pose, &slam_cmd_vel_);

        ROS_INFO("\n\n====== SLAM_TOOLBOX Control Signals: ======\n");
        ROS_INFO("Linear Velocity: %.4f\n", slam_cmd_vel_.twist.linear.x);
        ROS_INFO("Angular Velocity: %.4f\n", slam_cmd_vel_.twist.angular.z);

        slam_cmd_vel_.header.stamp = ros::Time::now();
        slam_cmd_vel_.header.frame_id = "base_link";

        // publish the velocity commands
        slam_cmd_vel_pub_.publish(slam_cmd_vel_);

        // add the message to an array of all messages
        slam_cmd_vel_list_.push_back(slam_cmd_vel_);
    }

    // Destructor
    ~DiffController()
    {
        std::cout << "\n\n\nShutting down the Differential Controller Node...\n\n\n" << std::endl;

        std::cout << "Stopping Robot...\n\n" << std::endl;
        // send a message to stop the robot
        cmd_vel_.header.stamp = ros::Time::now();
        cmd_vel_.header.frame_id = "base_link";
        cmd_vel_.twist.linear = geometry_msgs::Vector3();   // set the linear velocity to zero
        cmd_vel_.twist.angular = geometry_msgs::Vector3();  // set the angular velocity to zero
        cmd_vel_pub_.publish(cmd_vel_);

        // save a file with all messages received and published and their timestamps
        this->saveMessagesToFile();

        // shutdown the nodehandlers
        // nh_.shutdown();
        // priv_nh_.shutdown();
    }


    void saveMessagesToFile(){

        std::cout << "\n\n\nSaving Messages to a File...\n\n\n" << std::endl;

        // save the messages to a file
        std::string file_date;
        // get the current date and time
        std::time_t now = time(0);
        std::tm *localtm = localtime(&now);

        std::string t_mes = ((localtm->tm_mon +1 < 10) ? ("0" + std::to_string(localtm->tm_mon + 1)) : std::to_string(localtm->tm_mon + 1));
        std::string t_mday = ((localtm->tm_mday < 10) ? ("0" + std::to_string(localtm->tm_mday)) : std::to_string(localtm->tm_mday));
        std::string t_hour = ((localtm->tm_hour < 10) ? ("0" + std::to_string(localtm->tm_hour)) : std::to_string(localtm->tm_hour));
        std::string t_min = ((localtm->tm_min < 10) ? ("0" + std::to_string(localtm->tm_min)) : std::to_string(localtm->tm_min));
        std::string t_sec = ((localtm->tm_sec < 10) ? ("0" + std::to_string(localtm->tm_sec)) : std::to_string(localtm->tm_sec));

        file_date = std::to_string(localtm->tm_year + 1900)
                     + "-"
                     + t_mes
                     + "-"
                     + t_mday
                     + "_"
                     + t_hour
                     + "-"
                     + t_min
                     + "-"
                     + t_sec;

        // file path and name
        std::string file_path = "diff_control/messages/" + file_date + "/";
        std::string file_name = "messages";
        std::string file_extension = ".yaml";

        // create the directory
        std::filesystem::create_directories(file_path);

        /*
         * Create one file for each message type
         */

        // VRPN Messages
        std::string vrpn_filename = file_path + "_vrpn_" + file_name + file_extension;
        this->savePoseMessageToFile(vrpn_filename, "vrpn_poses", vrpn_twist_list_);

        // SLAM Messages
        std::string slam_filename = file_path + "_slam_" + file_name + file_extension;
        this->savePoseWithCovarianceStampedMessageToFile(slam_filename, "slam_poses", slam_pose_list_);

        // Goal Messages
        std::string goal_filename = file_path + "_goal_" + file_name + file_extension;
        this->savePoseMessageToFile(goal_filename, "goal_poses", goal_list_);

        // VRPN_CMD_VEL Messages
        std::string vrpn_cmd_vel_filename = file_path + "_vrpn_cmd_vel_" + file_name + file_extension;
        this->saveTwistMessageToFile(vrpn_cmd_vel_filename, "vrpn_cmd_vel", vrpn_cmd_vel_list_);

        // SLAM_CMD_VEL Messages
        std::string slam_cmd_vel_filename = file_path + "_slam_cmd_vel_" + file_name + file_extension;
        this->saveTwistMessageToFile(slam_cmd_vel_filename, "slam_cmd_vel", slam_cmd_vel_list_);


        std::cout << "\n\n\nMessages Saved to : " << file_path << "\n\n\n" << std::endl;

    }

    void savePoseMessageToFile(std::string filename, std::string message_name, std::vector<geometry_msgs::PoseStamped> message_list)
    {
        // Try to open file
        std::ofstream file(filename);

        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        // Header
        file << "# ===============================================\n\n";
        file << "# Messages Received and Published by the Differential Controller\n";
        file << "# File: " << filename << "\n\n";
        file << "# ===============================================\n\n";

        // save the vrpn messages
        file << "Message_name: " << message_name << "\n";
        file << "number_of_messages: " << message_list.size() << "\n";
        file << "messages: \n";
        for (int i = 0; i < message_list.size(); i++)
        {
            file << "    - {id: " << i << ", ";
            file << "Time: " << message_list[i].header.stamp << ", ";
            file << "Position: [" << message_list[i].pose.position.x << ", " << message_list[i].pose.position.y << ", " << message_list[i].pose.position.z << "] , ";
            file << "Orientation: [" << message_list[i].pose.orientation.x << ", " << message_list[i].pose.orientation.y << ", " << message_list[i].pose.orientation.z << ", " << message_list[i].pose.orientation.w << "] ";
            file << "}\n";
        }

        // close and save file
        file.close();
    }

    void savePoseWithCovarianceStampedMessageToFile(std::string filename, std::string message_name, std::vector<geometry_msgs::PoseWithCovarianceStamped> message_list)
    {
        // Try to open file
        std::ofstream file(filename);

        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        // Header
        file << "# ===============================================\n\n";
        file << "# Messages Received and Published by the Differential Controller\n";
        file << "# File: " << filename << "\n\n";
        file << "# ===============================================\n\n";

        // save the vrpn messages
        file << "Message_name: " << message_name << "\n";
        file << "number_of_messages: " << message_list.size() << "\n";
        file << "messages: \n";
        for (int i = 0; i < message_list.size(); i++)
        {
            file << "    - {id: " << i << ", ";
            file << "Time: " << message_list[i].header.stamp << ", ";
            file << "Position: [" << message_list[i].pose.pose.position.x << ", " << message_list[i].pose.pose.position.y << ", " << message_list[i].pose.pose.position.z << "] , ";
            file << "Orientation: [" << message_list[i].pose.pose.orientation.x << ", " << message_list[i].pose.pose.orientation.y << ", " << message_list[i].pose.pose.orientation.z << ", " << message_list[i].pose.pose.orientation.w << "] ";
            file << "}\n";
        }

        // close and save file
        file.close();
    }

    void saveTwistMessageToFile(std::string filename, std::string message_name, std::vector<geometry_msgs::TwistStamped> message_list)
    {
        // Try to open file
        std::ofstream file(filename);

        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        // Header
        file << "# ===============================================\n\n";
        file << "# Messages Received and Published by the Differential Controller\n";
        file << "# File: " << filename << "\n\n";
        file << "# ===============================================\n\n";

        // save the vrpn messages
        file << "Message_name: " << message_name << "\n";
        file << "number_of_messages: " << message_list.size() << "\n";
        file << "messages: \n";
        for (int i = 0; i < message_list.size(); i++)
        {
            file << "    - {id: " << i << ", ";
            file << "Time: " << message_list[i].header.stamp << ", ";
            file << "Linear: " << message_list[i].twist.linear.x << ", ";
            file << "Angular: " << message_list[i].twist.angular.z << " ";
            file << "}\n";
        }

        // close and save file
        file.close();
    }

};
