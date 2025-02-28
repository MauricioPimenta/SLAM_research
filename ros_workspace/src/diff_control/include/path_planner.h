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

// C++ Libraries for handling .yaml files
#include <yaml-cpp/yaml.h>
#include <xmlrpcpp/XmlRpcValue.h>   // to deal with nested parameters inside the .yaml file

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>


class PathPlanner
{
public:
    PathPlanner() : nh_(""), priv_nh_("~")
    {
        /*
         * get the parameters from the parameter server
        */
        loadCommonParameters();
        
        // Load the parameters from the .yaml file
        loadPathFromYaml();
        
        // generate trajectory from loaded points
        if (path_type_ == "LINEAR")
        {
            createLinearPath();
        }
        else if (path_type_ == "LEMNISCATA")
        {
            loadLemniscataParameters();
            createLemniscataPath(R_, w_);
        }
        else
        {
            ROS_ERROR("Invalid path type: %s", path_type_.c_str());
        }

        // Display the generated paths
        ROS_INFO("Generated %ld paths:", paths_.size());
        for (const auto& path : paths_)
        {
            ROS_INFO("Path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f] with %d points",
                     path.start_point.position[0], path.start_point.position[1], path.start_point.position[2],
                     path.end_point.position[0], path.end_point.position[1], path.end_point.position[2],
                     path.num_points);
        }

        // Create a publisher to publish the goal points
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_name_, 1);
        // Create a timer to publish the one point of the path as the goal each time it is called
        ros::Duration time_to_publish_goal = ros::Duration(1.0 / frequency_to_publish_goal_);
        publisher_timer_ = nh_.createTimer(time_to_publish_goal, &PathPlanner::publishGoal, this);

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
            createLemniscataPath(R_, w_);
        }
        else
        {
            ROS_ERROR("Invalid path type: %s", path_type.c_str());
        }
    }

    void loadCommonParameters()
    {
        /*
         * get the parameters from the parameter server
        */
        priv_nh_.param<double>("desired_vel", desired_velocity_, 0.5 /* m/s */);
        priv_nh_.param<double>("pub_frequency", frequency_to_publish_goal_, 60.0 /* Hz */);

        priv_nh_.param<double>("path_resolution", path_resolution_, desired_velocity_ / frequency_to_publish_goal_);
        priv_nh_.param<std::string>("goal_topic_name", goal_topic_name_, "/goal");

    }

    void loadPathFromYaml()
    {
        // Retrieve simple parameters
        std::string name;
        std::string frame_id;
        int num_points;
        std::string path_type_;

        priv_nh_.getParam("name", name);
        priv_nh_.getParam("frame_id", frame_id);
        priv_nh_.getParam("num_points", num_points);
        priv_nh_.getParam("path_type", path_type_);

        ROS_INFO("Path Name: %s", name.c_str());
        ROS_INFO("Frame ID: %s", frame_id.c_str());
        ROS_INFO("Number of Points: %d", num_points);

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
                        std::vector<double> point = {
                            static_cast<double>(points_param[i][0]),
                            static_cast<double>(points_param[i][1]),
                            static_cast<double>(points_param[i][2])
                        };
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
                {
                    ROS_INFO("[%.2f, %.2f, %.2f]", point[0], point[1], point[2]);
                }
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
        priv_nh_.param<double>("R", R_, 1.0);
        priv_nh_.param<double>("w", w_, 2*M_PI/40);
        priv_nh_.param<double>("tempo_experimento", time_of_experiment_, 60.0 /* seconds */);

        ROS_INFO("Lemniscata Parameters: R = %.2f, w = %.2f", R_, w_);
    }


    // Publish the points from the path to the goal topic
    void publishGoal(const ros::TimerEvent&)
    {
        // Static variables to keep track of the current path and point
        static int current_path = 0;
        static int current_point = 0;

        if (current_path < paths_.size() && current_point < paths_[current_path].points.size())
        {
            // Publish the current point as the goal
            ROS_INFO("Publishing goal point %d in path %d", current_point, current_path);

            // Publish the goal point
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.frame_id = frame_id_;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.pose.position.x = paths_[current_path].points[current_point].position[0];
            goal_msg.pose.position.y = paths_[current_path].points[current_point].position[1];
            goal_msg.pose.position.z = paths_[current_path].points[current_point].position[2];
            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;

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
            temp_path.start_point.position = points_[i];
            temp_path.end_point.position = points_[i + 1];
            temp_path.start_point.desired_velocity = desired_velocity_;
            temp_path.end_point.desired_velocity = 0.0;

            // Calculate the number of points in the path from the maximum distance in x or y
            // gets the absolute value of the difference between the start and end points
            double delta_x = abs(temp_path.end_point.position[0] - temp_path.start_point.position[0]);
            double delta_y = abs(temp_path.end_point.position[1] - temp_path.start_point.position[1]);

            // The number of points in the path is the maximum distance divided by the resolution
            temp_path.num_points = std::max({delta_x, delta_y}) / path_resolution_;

            // Calculate the increments in x and y based on the number of points
            double x_increments = delta_x / temp_path.num_points;
            double y_increments = delta_y / temp_path.num_points;

            // populate the list of points in the path
            for (int j = 0; j < temp_path.num_points; j++)
            {
                path_point temp_point;
                // x(t) = (1-t)*P + t*Q ; P and Q are the points and t = [0,1]
                double t = (double) j / (double) temp_path.num_points;
                temp_point.position[0] = (1 - t) * temp_path.start_point.position[0] + (t) * temp_path.end_point.position[0];    // X
                temp_point.position[1] = (1 - t) * temp_path.start_point.position[1] + (t) * temp_path.end_point.position[1];    // Y
                temp_point.position[2] = temp_path.start_point.position[2];                     // Z
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
    void createLemniscataPath(double R, double w)
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
        temp_path.start_point.position = {R*cos(0), R*sin(0), 0.0};
        temp_path.start_point.desired_velocity = desired_velocity_;

        temp_path.end_point.position = {R*cos(time_of_experiment_*w), R*sin(2*time_of_experiment_*w), 0.0};
        temp_path.end_point.desired_velocity = 0.0;

        // Populate the points in the path using the resolution and the time of the experiment



    }

    ~PathPlanner()
    {
        std::cout << "\n\t.\n\t.\n\t.\nShutting down the Path Planner Node...\n\t.\n\t.\n\t." << std::endl;

        // shutdown the nodehandlers
        nh_.shutdown();
        priv_nh_.shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::Timer publisher_timer_;

    // Messages
    geometry_msgs::PoseStamped goal_msg_;

    // Publishers
    ros::Publisher goal_pub_;

    /*
     * Parameters
     */
    std::string path_type_;
    std::string frame_id_;
    int num_points_;
    std::vector<std::vector<double>> points_;

    double desired_velocity_;
    double path_resolution_ = 0.001 /* meters */;
    double time_to_reach_goal_;

    double frequency_to_publish_goal_; // Hz

    std::string goal_topic_name_;


    // Lemniscata Parameters
    double R_;
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
        std::vector<double> position = {0.0, 0.0, 0.0};   // position of the point
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

    Path path_;
    std::vector<Path> paths_;
    int number_of_paths_;
    


};
#endif  // diff_control__path_planner_H_


