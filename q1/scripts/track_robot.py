#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import String, Header
from messages.msg import Position
from nav_msgs.msg import Odometry

class BeaconMonitor:
    def __init__(self, pub, x, y, beacon_id):
        self._pub = pub
        self._x = x
        self._y = y
        self._beacon_id = beacon_id

    def calcdistance(self, x1, x2, y1, y2):
        xd = x2 - x1
        yd = y2 - y1
        return math.sqrt(xd*xd + yd*yd)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # calculate the distance from robot to the beacon using beacon's location and robot's odom data
        distance = self.calcdistance(x, y, self._x, self._y)

        # publish a message with beacon location and robot's distance
        pos = Position()
        pos.position = "source {} location {} distance {}".format(self._beacon_id, (self._x, self._y), distance)
        # add a time header to synchronize messages
        pos.header = Header()
        pos.header.stamp = rospy.Time.now()
        self._pub.publish(pos)

def main():
    rospy.init_node('track_robot')

    # params to get beacon values
    beacon_id = rospy.get_param("~beacon_id", "beacon")
    x = rospy.get_param("~cord_x", 0)
    y = rospy.get_param("~cord_y", 0)

    # publisher to publish data from each beacon that will publish messages of custom message type beacon
    publisher = rospy.Publisher('{}/range'.format(beacon_id), Position, queue_size=10)
    monitor = BeaconMonitor(publisher,x, y, beacon_id)
    # subscribe to noise odom
    subscriber = rospy.Subscriber('/noisy_odom', Odometry, monitor.callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()