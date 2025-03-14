#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LaserScanAdjuster:
    def __init__(self):
        # Subscribe to the original LIDAR topic
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        # Publisher for the adjusted scan
        self.scan_pub_consistent = rospy.Publisher('scan_consistent', LaserScan, queue_size=10)
        self.scan_pub_ = rospy.Publisher('scan_fixed', LaserScan, queue_size=10)

        self.initVariables()


    def initVariables(self):
        # get desired_scan_size from parameter server
        if rospy.has_param('~desired_scan_size'):
            self.desired_scan_size = rospy.get_param('~desired_scan_size')
        else:
            self.desired_scan_size = 460


    def scan_callback(self, msg : LaserScan):
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

        # get the size of ranges
        scan_size = len(msg.ranges)

        expected_size = (int)((fixed_scan.angle_max - fixed_scan.angle_min) / fixed_scan.angle_increment)
        rospy.loginfo_once("Expected size: %d", expected_size)
        rospy.loginfo_once("Scan size: %d", scan_size)


        # check if the scan size is 460. if not, add missing ranges as 0 or remove ranges that are 0.0 or inf
        if scan_size == self.desired_scan_size:
            fixed_scan.ranges = msg.ranges
            fixed_scan.intensities = msg.intensities

            if scan_size != expected_size:
                rospy.logwarn_once("Scan size does not match expected size. Updating angle increment.")
                rospy.logwarn_once("Scan size: %d, Expected size: %d", scan_size, expected_size)

                # update the angle increment to match the scan_size
                fixed_scan.angle_increment = (fixed_scan.angle_max - fixed_scan.angle_min) / scan_size

                rospy.loginfo_once("Updated angle increment: %f", fixed_scan.angle_increment)
        # if scan size is less than 460, add missing ranges regularly spaced within the scan
        # with values equals the mean of the previous and next measurement values within the scan range vector
        elif scan_size < self.desired_scan_size:
            rospy.logwarn_once('scan size smaller than 460. Adding missing measurements')
            fixed_ranges = list(msg.ranges)
            fixed_intensities = list(msg.intensities)
            new_measure_offset = scan_size / (self.desired_scan_size - scan_size)
            rospy.loginfo_once('including measurements at every %dth index', new_measure_offset)

            for i in range(1, (self.desired_scan_size-1)) :

                # if multiple of new_measure_offset, insert the mean of the previous and next measurement values within the scan range vector
                # shifting the other values to the right.

                if i % new_measure_offset == 0 :
                    # if the previous measure is not valid, copy the next measurement value
                    if fixed_ranges[i] == 0.0 or fixed_ranges[i] == float('inf'):
                        fixed_ranges.insert(i, fixed_ranges[i+1])
                        fixed_intensities.insert(i, fixed_intensities[i+1])
                        rospy.loginfo("inserting new measurement at index %d: %f", i, (fixed_ranges[i]))
                        rospy.loginfo("inserting new intensity at index %d: %f", i, (fixed_intensities[i]))

                    # if the next measure is not valid, copy the previous measurement value
                    elif fixed_ranges[i+1] == 0.0 or fixed_ranges[i+1] == float('inf'):
                        fixed_ranges.insert(i, fixed_ranges[i])
                        fixed_intensities.insert(i, fixed_intensities[i])
                        rospy.loginfo("inserting new measurement at index %d: %f", i, (fixed_ranges[i]))
                        rospy.loginfo("inserting new intensity at index %d: %f", i, (fixed_intensities[i]))

                    # if both previous and next measures are valid, insert the mean of the two
                    else:
                        fixed_ranges.insert(i, (fixed_ranges[i-1] + fixed_ranges[i]) / 2)
                        fixed_intensities.insert(i, (fixed_intensities[i-1] + fixed_intensities[i]) / 2)
                        rospy.loginfo("inserting new measurement at index %d: %f", i, (fixed_ranges[i]))
                        rospy.loginfo("inserting new intensity at index %d: %f", i, (fixed_intensities[i]))


            rospy.loginfo_once('updating ranges, intensities and angle increment')
            fixed_scan.ranges = fixed_ranges
            fixed_scan.intensities = fixed_intensities
            fixed_scan.angle_increment = (fixed_scan.angle_max - fixed_scan.angle_min) / self.desired_scan_size
            fixed_scan.time_increment = msg.scan_time / self.desired_scan_size



        # if scan size is greater than 460, remove ranges that are 0.0 or inf
        elif scan_size > self.desired_scan_size :
            fixed_ranges = []
            fixed_intensities = []
            for i in range(0, scan_size):
                if msg.ranges[i] == 0.0 or msg.ranges[i] == float('inf'):
                    continue
                else:
                    fixed_ranges.append(msg.ranges[i])
                    fixed_intensities.append(msg.intensities[i])
            fixed_scan.ranges = fixed_ranges
            fixed_scan.intensities = fixed_intensities


        # # Check that ranges is not empty and remove the last element
        # if len(msg.ranges) > 1:
        #     fixed_scan.ranges = msg.ranges[:-1]  # all but last element
        # else:
        #     # If there's only one element or none, just pass it through as is
        #     fixed_scan.ranges = msg.ranges

        # # Do the same for intensities if they exist
        # if len(msg.intensities) == len(msg.ranges):
        #     fixed_scan.intensities = msg.intensities[:-1]
        # else:
        #     # If intensities are different length or missing, just copy directly
        #     fixed_scan.intensities = msg.intensities



        # Publish the modified scan
        self.scan_pub_consistent.publish(fixed_scan)


if __name__ == '__main__':
    rospy.init_node('laser_scan_adjuster', anonymous=True)
    adjuster = LaserScanAdjuster()
    rospy.spin()
