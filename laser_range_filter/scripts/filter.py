#!/usr/bin/env python
# insert BSD license here
# Gaya's shitty 5 min filter script

import rospy
from sensor_msgs.msg import LaserScan

class LaserRangeFilter:
    def __init__(self):
        self.pub = rospy.Publisher('scan', LaserScan, queue_size=1)
        rospy.Subscriber('scan_raw', LaserScan, self.filter_callback)

    def filter_callback(self, scan_msg):
        filtered_ranges = []
        filtered_intensities = []
        for rng, intensity in zip(scan_msg.ranges, scan_msg.intensities):
            if (rng > 0.4):
                filtered_ranges.append(rng)
                filtered_intensities.append(intensity)
            else:
               filtered_ranges.append(0)
               filtered_intensities.append(0)
        scan_msg.ranges = filtered_ranges
        scan_msg.intensities = filtered_intensities
        self.pub.publish(scan_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('laser_filter')
        laser_range_filter_object = LaserRangeFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
