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


class path_planner
{
public:
    path_planner() : nh_(""), priv_nh_("~")
    {
        // get the parameters from the parameter server
        priv_nh_.param<std::string>("frame_id", frame_id_, "world_frame");
        priv_nh_.param<int>("num_points", num_points_, 0);

        // Retrieve simple parameters
        std::string name;
        std::string frame_id;
        int num_points;

        nh_.getParam("name", name);
        nh_.getParam("frame_id", frame_id);
        nh_.getParam("num_points", num_points);

        ROS_INFO("Path Name: %s", name.c_str());
        ROS_INFO("Frame ID: %s", frame_id.c_str());
        ROS_INFO("Number of Points: %d", num_points);

        // Load points using XmlRpcValue
        XmlRpc::XmlRpcValue points_param;

        if (nh_.getParam("points", points_param))
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



    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    // Parameters
    std::string frame_id_;
    int num_points_;
    std::vector<std::vector<double>> points_;

    // Trajectory points
    struct trajectory_point
    {
        std::vector<double> position;   // position of the point
        double desired_velocity;        // desired velocity tangent to the trajectory
    };

    // Trajectory
    std::vector<trajectory_point> trajectory_;


};
#endif  // diff_control__path_planner_H_


