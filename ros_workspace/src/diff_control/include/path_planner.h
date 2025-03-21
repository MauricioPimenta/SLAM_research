/**--------------------------------------------------------------------------------------------------------
 * ?                                ABOUT
 * @author         :  Mauricio Bittencourt Pimenta
 * @email          :  mauricio.pimenta@edu.ufes.br
 * @repo           :  --
 * @createdOn      :  --
 * @description    :  This file contains the definitions for a class that implements
 *                    a path planner. The path planner is responsible for generating
 *                    a path for the robot to follow. The path is generated as a trajectory
 *                    for the robot to follow, even if the desired path is given as a finite
 *                    set of points.
 * There are two main ways to generate the path, one by defining the function that describes
 * the path and other is to get the points for positioning and the corresponding connections
 * between the points.
 *
 * I created this file in order to turn the positioning task into a trajectory tracking task.
 * The path planner will be responsible for generating the trajectory that the robot will follow
 * from the points defined in a .yaml file.
 *
 * For the positioning:
 *      Given the points from a Yaml file, define the straight line trajectories between the points and
 *      calculate the velocities for the robot in each point of the trajectory.
 *
*--------------------------------------------------------------------------------------------------------**/

#ifndef diff_control__path_planner_H_
#define diff_control__path_planner_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

/* ROS */
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2 library for Quaternions
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++ Libraries for handling .yaml files
#include <yaml-cpp/yaml.h>
#include <xmlrpcpp/XmlRpcValue.h>   // to deal with nested parameters inside the .yaml file

// ROS Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


class PathPlanner
{

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::Timer goal_pub_timer_;

    // Messages
    geometry_msgs::PoseStamped goal_msg_;
    nav_msgs::Path goal_path_msg_;

    // Publishers
    ros::Publisher goal_pub_;   // Publish the goal point
    ros::Publisher goal_path_pub_;  // Publish the list of points in the path

    /*
     * Parameters
     */

    // Map Parameters
    std::string name_;       // Name of the path
    std::string path_type_; // Type of the path
    std::string frame_id_;  // Frame ID to publish the goal points
    int num_points_;        // Number of points in the path
    std::vector<geometry_msgs::Point> points_;

    double desired_velocity_;
    double path_resolution_  {0.001} /* meters */;
    double time_to_reach_goal_;

    double frequency_to_publish_goal_; // Hz

    std::string goal_topic_name_;


    // Lemniscata Parameters
    double Rx_;
    double Ry_;
    double w_;
    double time_of_experiment_;


    // Path type will determine how the goal points will be generated
    enum path_type
    {
        LINEAR,     // Linear paths - straight lines - x(t) = (1-t)*P + t*Q ; P and Q are the points
        BEZIER,     // Bezier paths - x(t) = (1-t)^3*P + 3(1-t)^2*t*Q + 3(1-t)*t^2*R + t^3*S ; P, Q, R and S are the points
        CUBIC,      // Cubic paths - x(t) = (1-t)^3*P + 3(1-t)^2*t*Q + 3(1-t)*t^2*R + t^3*S ; P, Q, R and S are the points
        QUINTIC,     // Quintic paths - x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 ; a0, a1, a2, a3, a4 and a5 are the coefficients
        CIRCULAR,
        SPIRAL,
        LEMNISCATA
    };

    // This struct define the points in the path.
    // Each point has a position and a desired velocity tangent to the trajectory
    typedef struct path_point
    {
        geometry_msgs::Pose pose;   // position of the point
        double desired_velocity;        // desired velocity tangent to the trajectory
    } path_point;

    // The Path is a vector of path points
    typedef struct path
    {
        unsigned int num_points;
        path_point start_point;
        path_point end_point;
        std::vector<path_point> points;
        enum path_type path_type;
    }Path;

    // Path path_;
    std::vector<Path> paths_;
    int number_of_paths_;


public:
    PathPlanner() : nh_(""), priv_nh_("~")
    {
        /*
         * get the parameters from the parameter server
        */
        loadCommonParameters();

        // generate trajectory from loaded points
        if (path_type_ == "LINEAR")
        {
            // Load the parameters from the .yaml file and create the path from the points
            loadPathFromYaml();
            createLinearPath();

        }
        else if (path_type_ == "LEMNISCATA")
        {
            loadLemniscataParameters();
            createLemniscataPath(Rx_, Ry_, w_);
        }
        else
        {
            ROS_ERROR("Invalid path type: %s", path_type_.c_str());
        }

        printGeneratedPaths();
        createPublishers();
    }

    PathPlanner(std::string path_type) : nh_(""), priv_nh_("~")
    {
        loadCommonParameters();

        if (path_type == "LINEAR")
        {
            loadPathFromYaml();
            createLinearPath();
        }
        else if (path_type == "LEMNISCATA")
        {
            loadLemniscataParameters();
            createLemniscataPath(Rx_, Ry_, w_);
        }
        else
        {
            ROS_ERROR("Invalid path type: %s", path_type.c_str());
        }

        printGeneratedPaths();
        createPublishers();
    }

    void printGeneratedPaths()
    {
        // Display the generated paths
        ROS_INFO("Generated %ld paths:", paths_.size());
        for (const auto& path : paths_)
        {
            ROS_INFO("Path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f] with %d points",
                     path.start_point.pose.position.x, path.start_point.pose.position.y, path.start_point.pose.position.z,
                     path.end_point.pose.position.x, path.end_point.pose.position.y, path.end_point.pose.position.z,
                     path.num_points);
        }
    }

    void createPublishers()
    {
        // Create a publisher to publish the goal points
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_name_, 1, true);
        std::string goal_path_topic = goal_topic_name_ + "_path";
        goal_path_pub_ = nh_.advertise<nav_msgs::Path>(goal_path_topic, 1, true);
        // Create a timer to publish the one point of the path as the goal each time it is called
        ros::Duration time_to_publish_goal = ros::Duration(1.0 / frequency_to_publish_goal_);
        goal_pub_timer_ = nh_.createTimer(time_to_publish_goal, &PathPlanner::publishGoal, this);

    }

    void loadCommonParameters()
    {
        /*
         * get the parameters from the parameter server
        */
        priv_nh_.param<std::string>("path_type", path_type_, "LINEAR");
        priv_nh_.param<double>("desired_vel", desired_velocity_, 0.5 /* m/s */);
        priv_nh_.param<double>("pub_frequency", frequency_to_publish_goal_, 60.0 /* Hz */);

        priv_nh_.param<double>("path_resolution", path_resolution_, desired_velocity_ / frequency_to_publish_goal_);
        priv_nh_.param<std::string>("goal_topic_name", goal_topic_name_, "/goal");

        priv_nh_.param<std::string>("frame_id", frame_id_, "world");

    }

    void loadPathFromYaml()
    {
        // Retrieve parameters from the parameter server defined in the .yaml file
        if (priv_nh_.getParam("name", name_))
        {
            ROS_INFO("Path Name: %s", name_.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get 'name' parameter.");
        }
        if (priv_nh_.getParam("frame_id", frame_id_))
        {
            ROS_INFO("Frame ID: %s", frame_id_.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get 'frame_id' parameter.");
        }
        if (priv_nh_.getParam("num_points", num_points_))
        {
            ROS_INFO("Number of Points: %d", num_points_);
        }
        else
        {
            ROS_ERROR("Failed to get 'num_points' parameter.");
        }
        if (priv_nh_.getParam("path_type", path_type_))
        {
            ROS_INFO("Path Type: %s", path_type_.c_str());
        }
        else
        {
            ROS_ERROR("Failed to get 'path_type' parameter.");
        }

        // Load points using XmlRpcValue
        XmlRpc::XmlRpcValue points_param;

        if (priv_nh_.getParam("points", points_param))
        {
            if (points_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < points_param.size(); ++i)
                {
                    if (points_param[i].getType() == XmlRpc::XmlRpcValue::TypeArray && points_param[i].size() == 3)
                    {
                        geometry_msgs::Point point;
                        point.x = static_cast<double>(points_param[i][0]);
                        point.y = static_cast<double>(points_param[i][1]);
                        point.z = static_cast<double>(points_param[i][2]);

                        points_.push_back(point);
                    }
                    else
                    {
                        ROS_WARN("Invalid point format at index %d", i);
                    }
                }
                // Display loaded points
                ROS_INFO("Loaded %ld points:", points_.size());
                for (const auto& point : points_)
                    ROS_INFO("[%.2f, %.2f, %.2f]", point.x, point.y, point.z);
            }
            else
            {
                ROS_ERROR("'points' is not an array!");
            }
        }
        else
        {
            ROS_ERROR("Failed to get 'points' parameter.");
        }

        if (points_.size() < 2)
        {
            ROS_ERROR("Not enough points to generate a path.");
            return;
        }

    }

    void loadLemniscataParameters()
    {
        // Retrieve simple parameters
        priv_nh_.param<double>("Rx", Rx_, 2.0);
        priv_nh_.param<double>("Ry", Ry_, 2.0);
        priv_nh_.param<double>("w", w_, 2*M_PI/40);
        priv_nh_.param<double>("tempo_experimento", time_of_experiment_, 60.0 /* seconds */);

        ROS_INFO("Lemniscata Parameters: Rx = %.2f, Ry = %.2f, w = %.2f", Rx_, Ry_, w_);
    }


    // Publish the points from the path to the goal topic
    void publishGoal(const ros::TimerEvent&)
    {
        // Static variables to keep track of the current path and point
        static int current_path = 0;
        static int current_point = 0;

        if (current_path == 0 && current_point == 0)
        {
            ROS_INFO("Publishing Goal Path...");
            publishGoalPath();
        }
        else
        goal_path_pub_.publish(goal_path_msg_);

        if (current_path < paths_.size() && current_point < paths_[current_path].points.size())
        {
            // Publish the current point as the goal
            ROS_INFO("Publishing goal point %d in path %d", current_point, current_path);

            // Publish the goal point
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.frame_id = frame_id_;
            goal_msg.header.stamp = ros::Time::now();

            goal_msg.pose = paths_[current_path].points[current_point].pose;

            // Publish the goal
            goal_pub_.publish(goal_msg);

            // Increment the current point
            current_point++;
        }

        if (current_path < paths_.size() && current_point >= paths_[current_path].points.size())
        {
            ROS_INFO("All points in path %d have been published.", current_path);
            current_path++;
            current_point = 0;
        }

        // Check if the current path is valid
        if (current_path >= paths_.size())
            ROS_INFO("All paths have been published.");

    }

    void publishGoalPath()
    {
        goal_path_msg_ = nav_msgs::Path();

        ros::Time start_time = ros::Time::now();
        // Publish the whole Path
        goal_path_msg_.header.frame_id = frame_id_;
        goal_path_msg_.header.stamp = start_time;

        ROS_WARN("T1");
        int index = 0;
        for (int i=0; i < paths_.size(); i++)
        {
            ROS_WARN("\nTi: %d", i);
            // Poses
            for (int j=0; j < paths_[i].num_points; j++)
            {
                ROS_WARN("\nTj: %d", j);
                // Each Pose = Header + pose
                ROS_WARN("\nindex: %d", index);
                goal_path_msg_.poses.push_back(geometry_msgs::PoseStamped());
                ROS_WARN("\nPushed");
                // add the point of the path to the goal_path message at the index
                goal_path_msg_.poses[index].header.stamp = start_time ;//+ ros::Duration(index/frequency_to_publish_goal_);
                ROS_WARN("\nTime");
                goal_path_msg_.poses[index].header.frame_id = frame_id_;
                ROS_WARN("\nframe");
                goal_path_msg_.poses[index].header.seq = static_cast<uint32_t>(index);
                ROS_WARN("\nseq");
                goal_path_msg_.poses[index].pose = paths_[i].points[j].pose;
                ROS_WARN("\npose");

                index++;
            }
        }

        ROS_INFO("Publishing Goal Path...");
        goal_path_pub_.publish(goal_path_msg_);
    }

    /**-----------------------------------------------------------------------------------------------------------------------
     **                                                  createLinearPath
     *?  Create a Linear Path from the points loaded from the .yaml file.
     *?  The linear path is a straight line between the points.
     *?  If more than two points are given, create a list of linear paths created with each subsequent points as the start and end
     *?  points of a path.
     *@param none none
     *@param none none
     *@return void
     *-----------------------------------------------------------------------------------------------------------------------**/
    void createLinearPath()
    {
        // Create a linear path from the points
        for (int i = 0; i < (points_.size() - 1); i++)
        {
            Path temp_path;
            temp_path.path_type = LINEAR;
            // Saves the first and last points of the path
            temp_path.start_point.pose.position.x = points_[i].x;
            temp_path.start_point.pose.position.y = points_[i].y;
            temp_path.start_point.pose.position.z = points_[i].z;
            temp_path.start_point.desired_velocity = desired_velocity_;

            temp_path.end_point.pose.position.x = points_[i + 1].x;
            temp_path.end_point.pose.position.y = points_[i + 1].y;
            temp_path.end_point.pose.position.z = points_[i + 1].z;
            temp_path.end_point.desired_velocity = 0.0;

            // Calculate the number of points in the path from the maximum distance in x or y
            // gets the absolute value of the difference between the start and end points
            double delta_x = abs(temp_path.end_point.pose.position.x - temp_path.start_point.pose.position.x);
            double delta_y = abs(temp_path.end_point.pose.position.y - temp_path.start_point.pose.position.y);

            // The number of points in the path is the maximum distance divided by the resolution
            temp_path.num_points = std::max({delta_x, delta_y}) / path_resolution_;

            // Calculate the increments in x and y based on the number of points
            double x_increments = delta_x / temp_path.num_points;
            double y_increments = delta_y / temp_path.num_points;

            // Orientation of the point is the tangent of the path or trajectory
            // For linear paths, the orientation is the angle of the straight line between the start and end points
            double theta = atan2(temp_path.end_point.pose.position.y - temp_path.start_point.pose.position.y,
                                 temp_path.end_point.pose.position.x - temp_path.start_point.pose.position.x);
            // convert theta to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);

            // populate the list of points in the path
            for (int j = 0; j < temp_path.num_points; j++)
            {
                path_point temp_point;
                // x(t) = (1-t)*P + t*Q ; P and Q are the points and t = [0,1]
                double t = (double) j / (double) temp_path.num_points;
                temp_point.pose.position.x = (1 - t) * temp_path.start_point.pose.position.x + (t) * temp_path.end_point.pose.position.x;    // X
                temp_point.pose.position.y = (1 - t) * temp_path.start_point.pose.position.y + (t) * temp_path.end_point.pose.position.y;    // Y
                temp_point.pose.position.z = temp_path.start_point.pose.position.z;                     // Z

                // Saves the orientation of the point as the orientation of the line between the start and end points
                temp_point.pose.orientation = tf2::toMsg(q);

                temp_point.desired_velocity = desired_velocity_;                                // Velocity

                // Add the point to the path
                temp_path.points.push_back(temp_point);
            }

            // Insert the path in the list of paths
            paths_.push_back(temp_path);
        }

    }

    /**-----------------------------------------------------------------------------------------------------------------------
     **                                                  createLemniscataPath
     *?  Create a 2D Lemniscata Path.
     *?  The Lemniscata path is a infinity or '8' figure, described by the equation:
     *?  (x^2 + y^2)^2 = a^2*(x^2 - y^2)
     *?  The Lemniscata path is a curve that can be described by the equation:
     *?  x(t) = a*cos(t) / (1 + sin^2(t)) ; y(t) = a*sin(t)*cos(t) / (1 + sin^2(t))
     *
     *@param none none
     *@param none none
     *@return void
     *-----------------------------------------------------------------------------------------------------------------------**/
    void createLemniscataPath(double Rx, double Ry, double w)
    {
        // trajetoria em lemniscata:
        // Xd = Rcos(wt)
        // Yd = Rsin(2wt)
        // dXd = -Rw*sin(wt)
        // dYd = 2Rw*cos(2wt)
        // w = 2*pi/40

        // Create all the points in the lemniscata path and add them to the list of paths
        Path temp_path;
        temp_path.path_type = LEMNISCATA;

        // vel*t_exp = total_dist -> total_dist/resolution = num_points
        // n = vel*t_exp/(vel/freq)
        temp_path.num_points = frequency_to_publish_goal_*time_of_experiment_;
        ROS_INFO("Number of points in the lemniscata path: %d", temp_path.num_points);

        // Populate the points in the path using the resolution and the time of the experiment
        for (double t = 0; t <= time_of_experiment_; t += time_of_experiment_/temp_path.num_points)
        {
            path_point temp_point;

            temp_point.pose.position.x = Rx*cos(w*t);
            temp_point.pose.position.y = Ry*sin(2*w*t);
            temp_point.pose.position.z = 0.0;

            // Orientation of the point is the tangent of the path or trajectory
            // For lemniscata paths, the orientation is the angle of the tangent to the curve
            double theta = atan2(2*Ry*w*cos(2*w*t), -Rx*w*sin(w*t));
            // convert theta to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            temp_point.pose.orientation = tf2::toMsg(q);

            temp_point.desired_velocity = desired_velocity_;

            // Add the point to the path
            temp_path.points.push_back(temp_point);
        }

        // Insert the path in the list of paths
        paths_.push_back(temp_path);

    }

    ~PathPlanner()
    {
        std::cout << "\n\t.\n\t.\n\t.\nShutting down the Path Planner Node...\n\t.\n\t.\n\t." << std::endl;

        // shutdown the nodehandlers
        // nh_.shutdown();
        // priv_nh_.shutdown();

        std::cout << "Done!." << std::endl;
    }



};
#endif  // diff_control__path_planner_H_


