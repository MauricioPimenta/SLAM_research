#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
//#include <sensor_analysis/LaserDistances.h>
//#include <sensor_analysis/LaserAngles.h>
#include <vector>
#include <utility>
#include <cmath>

std::vector<double> laser_dist;
std::vector<double> laser_angdeg;
std::vector<double> laser_angdeg_dist;

double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

void Callback_Laser(const sensor_msgs::LaserScan::ConstPtr& message,
                    ros::Publisher& dist_pub, ros::Publisher& ang_pub, ros::Publisher& angdist_pub) {
    double angleIncrement = message->angle_increment;
    double angleMin = message->angle_min;
    double angleMax = message->angle_max;

    std::vector<float> ranges = message->ranges;

    std::vector<double> angleaux;
    for (double angle = angleMin; angle <= angleMax; angle += angleIncrement) {
        angleaux.push_back(rad2deg(angle));
    }

    if (ranges.size() > angleaux.size()) {
        ranges.erase(ranges.begin());
    }
    if (ranges.size() < angleaux.size()) {
        ranges.push_back(0);
    }

    std::vector<double> rangeaux(ranges.begin(), ranges.end());
    for (double& range : rangeaux) {
        range *= 1000;
    }

    std::vector<std::pair<double, double>> MatrizRangesAngles;
    for (size_t i = 0; i < rangeaux.size(); ++i) {
        if (rangeaux[i] != 0) {
            MatrizRangesAngles.emplace_back(rangeaux[i], angleaux[i]);
        }
    }

    laser_dist.clear();
    laser_angdeg.clear();
    laser_angdeg_dist.clear();
    for (const auto& pair : MatrizRangesAngles) {
        laser_dist.push_back(pair.first);
        laser_angdeg.push_back(pair.second);
    }

    // Publicar os vetores laser_dist e laser_angdeg
    sensor_msgs::LaserScan dist_msg;
    sensor_msgs::LaserScan ang_msg;
    sensor_msgs::LaserScan angdeg_dist_msg;

    dist_msg.header.stamp = ros::Time::now();
    ang_msg.header.stamp = ros::Time::now();
    angdeg_dist_msg.header.stamp = ros::Time::now();

    dist_msg.ranges = std::vector<float>(laser_dist.begin(), laser_dist.end());
    ang_msg.intensities = std::vector<float>(laser_angdeg.begin(), laser_angdeg.end());
    angdeg_dist_msg.ranges = std::vector<float>(laser_dist.begin(), laser_dist.end());
    angdeg_dist_msg.intensities = std::vector<float>(laser_angdeg.begin(), laser_angdeg.end());


    dist_pub.publish(dist_msg);
    ang_pub.publish(ang_msg);
    angdist_pub.publish(angdeg_dist_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_processor");
    ros::NodeHandle nh;

    // Publishers para publicar os vetores laser_dist e laser_angdeg

    ros::Publisher dist_pub = nh.advertise<sensor_msgs::LaserScan>("laser_distances", 10);
    ros::Publisher ang_pub = nh.advertise<sensor_msgs::LaserScan>("laser_angles", 10);
    ros::Publisher angdist_pub = nh.advertise<sensor_msgs::LaserScan>("laser_angles_distances", 10);

    // Subscriber para o tópico /L1/scan
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/L1/scan", 10,
        boost::bind(Callback_Laser, _1, boost::ref(dist_pub), boost::ref(ang_pub), boost::ref(angdist_pub)));

    ros::spin();
    return 0;
}