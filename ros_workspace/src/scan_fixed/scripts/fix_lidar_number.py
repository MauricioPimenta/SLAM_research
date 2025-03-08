#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LaserScanAdjuster:
    def __init__(self):
        # Subscribe to the original LIDAR topic
        self.scan_sub = rospy.Subscriber('/L1/scan', LaserScan, self.scan_callback)

        # Publisher for the adjusted scan
        self.scan_pub = rospy.Publisher('/360/scan_fixed', LaserScan, queue_size=10)

    def scan_callback(self, msg):
        # Make a copy of the original scan message
        fixed_scan = LaserScan()
        fixed_scan.header = msg.header
        fixed_scan.header.frame_id = 'laser_fixed'
        fixed_scan.angle_min = msg.angle_min
        fixed_scan.angle_max = msg.angle_max
        fixed_scan.angle_increment = msg.angle_increment
        fixed_scan.time_increment = msg.time_increment
        fixed_scan.scan_time = msg.scan_time
        fixed_scan.range_min = msg.range_min
        fixed_scan.range_max = msg.range_max

        # Check that ranges is not empty and remove the last element
        if len(msg.ranges) > 1:
            fixed_scan.ranges = msg.ranges[:-1]  # all but last element
        else:
            # If there's only one element or none, just pass it through as is
            fixed_scan.ranges = msg.ranges

        # Do the same for intensities if they exist
        if len(msg.intensities) == len(msg.ranges):
            fixed_scan.intensities = msg.intensities[:-1]
        else:
            # If intensities are different length or missing, just copy directly
            fixed_scan.intensities = msg.intensities

        # Publish the modified scan
        self.scan_pub.publish(fixed_scan)


if __name__ == '__main__':
    rospy.init_node('laser_scan_adjuster', anonymous=True)
    adjuster = LaserScanAdjuster()
    rospy.spin()
