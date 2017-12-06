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
        filtered_ranges = list(scan_msg.ranges)
        filtered_intensities = list(scan_msg.intensities)
        # cut out everything which is closer than min_range
        # use marker to mark the points that are closer
        marker = 111
        min_range = 0.20
        margin = 2 # margin to the left, and margin to the right, altogether 2xmargin+1
        i = 0
        started_index = [] # there are the indeces of marked regions
        ended_index = []
        while i < len(filtered_ranges):
            if (filtered_ranges[i] < min_range):
                if (len(started_index) <= len(ended_index)):
                    # first point in the region, add its index to start
                    started_index.append(i)
                for j in range(max(i - margin, 0), i):
                    filtered_ranges[j] = marker
                    filtered_intensities[j] = 0
                for j in range(i, min(i + margin + 1, len(filtered_ranges))):
                    filtered_ranges[j] = marker
                    filtered_intensities[j] = 0
                    i += 1
            else:
                if (len(started_index) > len(ended_index)):
                   # first point outside of region, add the index to ended
                   ended_index.append(i - 1)
                i += 1
        if (len(ended_index) < len(started_index)):
            # region ended with the end of the array
            ended_index.append(len(filtered_ranges) - 1)

        # combine regions that are very close to each other
        neighbourhood_margin = 15
        for i in range(1, len(started_index)):
            if (started_index[i] - ended_index[i - 1]) < neighbourhood_margin:
                # combine regions
                ended_index[i - 1] = ended_index[i]
                started_index[i] = started_index[i - 1]
        # started_index = list(set(started_index))
        # ended_index = list(set(ended_index))
        # visualize
        # for i in range(0, len(started_index)):
        #     filtered_ranges[started_index[i]] = 0.2
        #     filtered_intensities[started_index[i]] = 300
        #     filtered_ranges[ended_index[i]] = 0.3
        #     filtered_intensities[ended_index[i]] = 300

        # starting from the marked areas, find their center point and cut
        # all the points in the cut_out_margin to the left and to the right
        cut_out_margin = 60
        for i in range(0, len(started_index)):
            center = int((ended_index[i] - started_index[i]) / 2.0 + started_index[i])
            # visualize
            # filtered_ranges[center] = 0.5
            # filtered_intensities[center] = 0
            for j in range(max(center - cut_out_margin, 0),
                           min(center + cut_out_margin + 1, len(filtered_ranges))):
                filtered_ranges[j] = marker
                filtered_intensities[j] = 0
        
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
